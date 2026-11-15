using System.Runtime.InteropServices;

// Same namespace and surface as shell/WaveOutSink.cs (winmm), because
// MachineSession is shared source and drives whichever one is compiled in.
namespace OpenPowerMac.Shell;

/// <summary>Streaming host audio sink over an SDL3 audio stream (PipeWire,
/// PulseAudio or ALSA, whichever SDL finds).
///
/// Everything here runs on the machine's own worker thread, the same thread
/// that calls into the core. SDL pulls from the stream on its own thread;
/// this side only queues and reads how much is still queued.
///
/// The behavior matches the Windows sink so the two front ends sound the
/// same and report the same counters:
///  - a 300 ms ceiling on what is queued (the latency bound), and the caller
///    drains the machine by <see cref="FreeBytes"/>, so the overflow waits in
///    the codec's own queue instead of being thrown away here;
///  - 100 ms of silence queued ahead of a sound that starts from an idle
///    device, because the codec never runs more than its 46 ms FIFO ahead of
///    the play cursor and a host hitch longer than that would otherwise run
///    the device dry mid-sound (heard as a pop);
///  - <see cref="Starvations"/> counts the device found empty with a sound
///    already under way, <see cref="DroppedFrames"/> what did not fit.</summary>
internal sealed class WaveOutSink : IDisposable
{
    private const int MillisCap = 300;
    private const int MillisPrime = 100;

    private const uint SDL_INIT_AUDIO = 0x00000010;
    private const uint SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK = 0xFFFFFFFF;
    private const int SDL_AUDIO_S16LE = 0x8010;

    [StructLayout(LayoutKind.Sequential)]
    private struct SDL_AudioSpec
    {
        public int format;
        public int channels;
        public int freq;
    }

    [DllImport("SDL3")] [return: MarshalAs(UnmanagedType.I1)]
    private static extern bool SDL_SetAppMetadata(string appname, string appversion, string appidentifier);
    [DllImport("SDL3")] [return: MarshalAs(UnmanagedType.I1)]
    private static extern bool SDL_InitSubSystem(uint flags);
    [DllImport("SDL3")] private static extern void SDL_QuitSubSystem(uint flags);
    [DllImport("SDL3")]
    private static extern IntPtr SDL_OpenAudioDeviceStream(uint devid, in SDL_AudioSpec spec, IntPtr callback, IntPtr userdata);
    [DllImport("SDL3")] [return: MarshalAs(UnmanagedType.I1)]
    private static extern bool SDL_ResumeAudioStreamDevice(IntPtr stream);
    [DllImport("SDL3")] [return: MarshalAs(UnmanagedType.I1)]
    private static extern bool SDL_PutAudioStreamData(IntPtr stream, byte[] buf, int len);
    [DllImport("SDL3")] private static extern int SDL_GetAudioStreamQueued(IntPtr stream);
    [DllImport("SDL3")] private static extern void SDL_DestroyAudioStream(IntPtr stream);

    private IntPtr _stream;
    private bool _sdlInit;
    private int _capBytes, _primeBytes;
    private bool _playing;
    private byte[] _swap = [];

    /// <summary>Sample rate the device is currently open at (0 = closed).</summary>
    public int Rate { get; private set; }
    /// <summary>Frames that did not fit under the latency ceiling.</summary>
    public long DroppedFrames { get; private set; }
    /// <summary>Times the device ran dry with a sound already under way.</summary>
    public long Starvations { get; private set; }

    public bool IsOpen => _stream != IntPtr.Zero;

    /// <summary>Bytes the sink can take right now without dropping any.</summary>
    public int FreeBytes
    {
        get
        {
            if (_stream == IntPtr.Zero)
                return 0;
            int queued = Math.Max(0, SDL_GetAudioStreamQueued(_stream));
            // Idle: Write spends the prime on the cushion first.
            if (queued == 0)
                return (_capBytes - _primeBytes) & ~3;
            return Math.Max(0, _capBytes - queued) & ~3;
        }
    }

    /// <summary>Open (or re-open at a new rate). False when the host has no
    /// usable output: the caller stops feeding it, and the machine still runs.</summary>
    public bool Open(int rate)
    {
        if (_stream != IntPtr.Zero && Rate == rate)
            return true;
        Close();
        if (rate < 4000 || rate > 192000)
            return false;
        try
        {
            if (!_sdlInit)
            {
                SDL_SetAppMetadata("OpenPowerMac", OpenPowerMac.Gui.Platform.NativeLibs.AppVersion, "io.github.codingncaffeine.OpenPowerMac");
                _sdlInit = SDL_InitSubSystem(SDL_INIT_AUDIO);
                if (!_sdlInit)
                    return false;
            }
            var spec = new SDL_AudioSpec { format = SDL_AUDIO_S16LE, channels = 2, freq = rate };
            _stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, in spec, IntPtr.Zero, IntPtr.Zero);
            if (_stream == IntPtr.Zero)
                return false;
            SDL_ResumeAudioStreamDevice(_stream);
        }
        catch (Exception ex) when (ex is DllNotFoundException or EntryPointNotFoundException)
        {
            // No SDL3 on this host: a machine with no speakers still boots.
            _stream = IntPtr.Zero;
            return false;
        }
        Rate = rate;
        _capBytes = rate * 4 * MillisCap / 1000 & ~3;
        _primeBytes = rate * 4 * MillisPrime / 1000 & ~3;
        _playing = false;
        return true;
    }

    public void Close()
    {
        if (_stream != IntPtr.Zero)
        {
            SDL_DestroyAudioStream(_stream);
            _stream = IntPtr.Zero;
        }
        Rate = 0;
        _playing = false;
    }

    /// <summary>Queue PCM. <paramref name="pcm"/> holds 16-bit stereo frames
    /// in the GUEST's byte order; <paramref name="bigEndian"/> swaps them on
    /// the way out, which is what Mac OS produces. Never blocks: what does not
    /// fit under the ceiling is dropped and counted.</summary>
    public void Write(byte[] pcm, int count, bool bigEndian)
    {
        if (_stream == IntPtr.Zero || count < 4)
            return;
        int queued = Math.Max(0, SDL_GetAudioStreamQueued(_stream));
        if (queued == 0)
        {
            if (_playing)
                Starvations++;
            var silence = new byte[_primeBytes];
            SDL_PutAudioStreamData(_stream, silence, silence.Length);
            queued = _primeBytes;
        }
        _playing = true;
        int n = Math.Min(count, Math.Max(0, _capBytes - queued)) & ~3;
        if (n < count)
            DroppedFrames += (count - n) / 4;
        if (n == 0)
            return;
        if (_swap.Length < n)
            _swap = new byte[n];
        if (bigEndian)
        {
            for (int k = 0; k < n; k += 2)
            {
                _swap[k] = pcm[k + 1];
                _swap[k + 1] = pcm[k];
            }
        }
        else
        {
            Buffer.BlockCopy(pcm, 0, _swap, 0, n);
        }
        SDL_PutAudioStreamData(_stream, _swap, n);
    }

    public void Dispose()
    {
        Close();
        if (_sdlInit)
        {
            SDL_QuitSubSystem(SDL_INIT_AUDIO);
            _sdlInit = false;
        }
    }
}
