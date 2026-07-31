using System.IO;

namespace OpenPowerMac.Shell;

/// <summary>Writes everything the shell hands to the speaker to a WAV file.
///
/// ⭐ THE APP AND g4run ARE TWO HARNESSES AND THEY DRIFT. g4run's --wav-out
/// captures what the CODEC produced under instruction pacing; this captures
/// what the APP played under real-time pacing, which is a different timeline
/// with different hitches — and the app is where the audio is actually heard.
/// Without it, a report of "a pop near the end" can only be answered by
/// reasoning about the other harness's output.
///
/// Off unless OPM_AUDIO_WAV names a path. Costs nothing when unset.</summary>
internal sealed class AudioCapture : IDisposable
{
    private FileStream? _f;
    private int _rate;
    private uint _bytes;

    public static AudioCapture? FromEnvironment()
    {
        string? path = Environment.GetEnvironmentVariable("OPM_AUDIO_WAV");
        return string.IsNullOrWhiteSpace(path) ? null : new AudioCapture(path);
    }

    private readonly string _path;
    private AudioCapture(string path) => _path = path;

    /// <summary>Append 16-bit stereo frames. <paramref name="bigEndian"/>
    /// samples are swapped, so the file always holds what the speaker heard
    /// rather than what the guest wrote.</summary>
    public void Write(byte[] pcm, int count, int rate, bool bigEndian)
    {
        if (_f == null)
        {
            try { _f = File.Create(_path); }
            catch { return; }
            _rate = rate;
            _f.Write(new byte[44], 0, 44); // header patched on close
        }
        if (count < 2)
            return;
        var buf = new byte[count & ~1];
        for (int k = 0; k + 1 < buf.Length; k += 2)
        {
            buf[k] = bigEndian ? pcm[k + 1] : pcm[k];
            buf[k + 1] = bigEndian ? pcm[k] : pcm[k + 1];
        }
        _f.Write(buf, 0, buf.Length);
        _bytes += (uint)buf.Length;
    }

    public void Dispose()
    {
        if (_f == null)
            return;
        try
        {
            _f.Seek(0, SeekOrigin.Begin);
            var w = new BinaryWriter(_f);
            w.Write("RIFF"u8.ToArray());
            w.Write(36u + _bytes);
            w.Write("WAVEfmt "u8.ToArray());
            w.Write(16u);
            w.Write((ushort)1);
            w.Write((ushort)2);
            w.Write((uint)_rate);
            w.Write((uint)(_rate * 4));
            w.Write((ushort)4);
            w.Write((ushort)16);
            w.Write("data"u8.ToArray());
            w.Write(_bytes);
            w.Flush();
        }
        catch { /* a lost capture is not worth taking the machine down for */ }
        _f.Dispose();
        _f = null;
    }
}
