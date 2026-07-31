using System.Runtime.InteropServices;

namespace OpenPowerMac.Shell;

/// <summary>Streaming host audio sink over winmm's waveOut.
///
/// No package reference: the app ships one native DLL and nothing else, and a
/// media stack pulled in for one 16-bit stereo stream would be the largest
/// dependency in the project. waveOut is the oldest and most universally
/// present Windows audio path, and a fixed ring of pre-prepared buffers is all
/// a fixed-format stream needs.
///
/// Everything here runs on the machine's own worker thread — the same thread
/// that calls into the core — so there is no locking and no callback. Buffers
/// are recycled by polling WHDR_DONE, which is what CALLBACK_NULL is for.</summary>
internal sealed class WaveOutSink : IDisposable
{
    // Twelve buffers of 25 ms: 300 ms of slack against a guest that produces
    // audio in 23 ms bursts (half of the codec's 8 KB FIFO) and whose clock
    // slips whenever the emulator hits synchronous host I/O. Short enough that
    // a system beep does not arrive noticeably late.
    private const int Buffers = 12;
    private const int MillisPerBuffer = 25;

    private IntPtr _hwo;
    private readonly IntPtr[] _hdr = new IntPtr[Buffers];
    private readonly IntPtr[] _data = new IntPtr[Buffers];
    private int _bufBytes;
    private int _next;

    /// <summary>Sample rate the device is currently open at (0 = closed).</summary>
    public int Rate { get; private set; }
    /// <summary>Frames dropped because every buffer was still playing. A
    /// non-zero count means the host could not keep up, or the guest produced
    /// a burst larger than the ring — it is the number that tells a stutter
    /// apart from a machine that simply made no sound.</summary>
    public long DroppedFrames { get; private set; }

    public bool IsOpen => _hwo != IntPtr.Zero;

    /// <summary>Bytes the sink can take right now without dropping any.
    ///
    /// ⭐ The caller drains the machine by THIS, not by its own buffer size.
    /// The codec's queue holds about six seconds and is the right place for
    /// audio to wait; taking it out of the machine and then throwing it away
    /// because the device was busy is how a hitch becomes a hole in the
    /// waveform, and a hole is heard as a pop.</summary>
    public int FreeBytes
    {
        get
        {
            if (_hwo == IntPtr.Zero)
                return 0;
            // Buffers are consumed in rotation, so only the run starting at
            // _next is usable without playing them out of order.
            int free = 0;
            for (int k = 0; k < Buffers; k++)
            {
                int idx = (_next + k) % Buffers;
                if ((GetFlags(_hdr[idx]) & WHDR_DONE) == 0)
                    break;
                free += _bufBytes;
            }
            return free;
        }
    }

    /// <summary>Open (or re-open at a new rate). Returns false if the host has
    /// no usable output device, in which case the caller should simply stop
    /// feeding it — a machine with no speakers still boots.</summary>
    public bool Open(int rate)
    {
        if (_hwo != IntPtr.Zero && Rate == rate)
            return true;
        Close();
        if (rate < 4000 || rate > 192000)
            return false;

        var fmt = new WAVEFORMATEX
        {
            wFormatTag = 1,            // WAVE_FORMAT_PCM
            nChannels = 2,
            nSamplesPerSec = (uint)rate,
            wBitsPerSample = 16,
            nBlockAlign = 4,
            nAvgBytesPerSec = (uint)(rate * 4),
            cbSize = 0,
        };
        if (waveOutOpen(out _hwo, WAVE_MAPPER, ref fmt, IntPtr.Zero, IntPtr.Zero,
                        CALLBACK_NULL) != 0)
        {
            _hwo = IntPtr.Zero;
            return false;
        }

        _bufBytes = rate * 4 * MillisPerBuffer / 1000 & ~3; // whole frames
        for (int i = 0; i < Buffers; i++)
        {
            _data[i] = Marshal.AllocHGlobal(_bufBytes);
            _hdr[i] = Marshal.AllocHGlobal(Marshal.SizeOf<WAVEHDR>());
            var h = new WAVEHDR
            {
                lpData = _data[i],
                dwBufferLength = (uint)_bufBytes,
                dwFlags = 0,
            };
            Marshal.StructureToPtr(h, _hdr[i], false);
            waveOutPrepareHeader(_hwo, _hdr[i], (uint)Marshal.SizeOf<WAVEHDR>());
            // A prepared-but-never-written header has neither DONE nor
            // INQUEUE set, so mark it free the way a finished one is.
            SetFlags(_hdr[i], WHDR_PREPARED | WHDR_DONE);
        }
        Rate = rate;
        _next = 0;
        return true;
    }

    public void Close()
    {
        if (_hwo == IntPtr.Zero)
        {
            Rate = 0;
            return;
        }
        waveOutReset(_hwo);
        for (int i = 0; i < Buffers; i++)
        {
            if (_hdr[i] != IntPtr.Zero)
            {
                waveOutUnprepareHeader(_hwo, _hdr[i], (uint)Marshal.SizeOf<WAVEHDR>());
                Marshal.FreeHGlobal(_hdr[i]);
                _hdr[i] = IntPtr.Zero;
            }
            if (_data[i] != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(_data[i]);
                _data[i] = IntPtr.Zero;
            }
        }
        waveOutClose(_hwo);
        _hwo = IntPtr.Zero;
        Rate = 0;
    }

    /// <summary>Queue PCM for playback. <paramref name="pcm"/> holds
    /// 16-bit stereo frames in the GUEST's byte order; set
    /// <paramref name="bigEndian"/> to have them swapped on the way out, which
    /// is what Mac OS produces. Never blocks: when every buffer is still in
    /// flight the excess is dropped and counted.</summary>
    public void Write(byte[] pcm, int count, bool bigEndian)
    {
        if (_hwo == IntPtr.Zero || count < 4)
            return;
        int at = 0;
        while (at < count)
        {
            int idx = _next;
            if ((GetFlags(_hdr[idx]) & WHDR_DONE) == 0)
            {
                DroppedFrames += (count - at) / 4;
                return; // ring full: drop the rest rather than stall the machine
            }
            int n = Math.Min(_bufBytes, count - at) & ~3;
            if (n == 0)
                return;
            if (bigEndian)
            {
                // waveOut wants little-endian samples; the guest wrote
                // big-endian ones.
                if (_swap.Length < n)
                    _swap = new byte[n];
                for (int k = 0; k < n; k += 2)
                {
                    _swap[k] = pcm[at + k + 1];
                    _swap[k + 1] = pcm[at + k];
                }
                Marshal.Copy(_swap, 0, _data[idx], n);
            }
            else
            {
                Marshal.Copy(pcm, at, _data[idx], n);
            }
            SetLengthAndArm(_hdr[idx], (uint)n);
            waveOutWrite(_hwo, _hdr[idx], (uint)Marshal.SizeOf<WAVEHDR>());
            _next = (_next + 1) % Buffers;
            at += n;
        }
    }

    public void Dispose() => Close();

    private byte[] _swap = [];

    // WAVEHDR is written by the driver while we hold it, so the flags and the
    // length are poked in place rather than round-tripped through a managed
    // copy — marshalling the whole structure back and forward would carry the
    // driver's own store to dwFlags away with it.
    private static uint GetFlags(IntPtr hdr) =>
        (uint)Marshal.ReadInt32(hdr, FlagsOffset);

    private static void SetFlags(IntPtr hdr, uint flags) =>
        Marshal.WriteInt32(hdr, FlagsOffset, (int)flags);

    private static void SetLengthAndArm(IntPtr hdr, uint len)
    {
        Marshal.WriteInt32(hdr, LengthOffset, (int)len);
        Marshal.WriteInt32(hdr, FlagsOffset, (int)WHDR_PREPARED); // clear DONE
    }

    private static readonly int FlagsOffset =
        (int)Marshal.OffsetOf<WAVEHDR>(nameof(WAVEHDR.dwFlags));
    private static readonly int LengthOffset =
        (int)Marshal.OffsetOf<WAVEHDR>(nameof(WAVEHDR.dwBufferLength));

    private const uint WAVE_MAPPER = 0xFFFFFFFF;
    private const uint CALLBACK_NULL = 0;
    private const uint WHDR_DONE = 0x00000001;
    private const uint WHDR_PREPARED = 0x00000002;

    [StructLayout(LayoutKind.Sequential)]
    private struct WAVEFORMATEX
    {
        public ushort wFormatTag;
        public ushort nChannels;
        public uint nSamplesPerSec;
        public uint nAvgBytesPerSec;
        public ushort nBlockAlign;
        public ushort wBitsPerSample;
        public ushort cbSize;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct WAVEHDR
    {
        public IntPtr lpData;
        public uint dwBufferLength;
        public uint dwBytesRecorded;
        public IntPtr dwUser;
        public uint dwFlags;
        public uint dwLoops;
        public IntPtr lpNext;
        public IntPtr reserved;
    }

    [DllImport("winmm.dll")]
    private static extern int waveOutOpen(out IntPtr phwo, uint uDeviceID,
                                          ref WAVEFORMATEX pwfx,
                                          IntPtr dwCallback, IntPtr dwInstance,
                                          uint fdwOpen);

    [DllImport("winmm.dll")]
    private static extern int waveOutPrepareHeader(IntPtr hwo, IntPtr pwh, uint cbwh);

    [DllImport("winmm.dll")]
    private static extern int waveOutUnprepareHeader(IntPtr hwo, IntPtr pwh, uint cbwh);

    [DllImport("winmm.dll")]
    private static extern int waveOutWrite(IntPtr hwo, IntPtr pwh, uint cbwh);

    [DllImport("winmm.dll")]
    private static extern int waveOutReset(IntPtr hwo);

    [DllImport("winmm.dll")]
    private static extern int waveOutClose(IntPtr hwo);
}
