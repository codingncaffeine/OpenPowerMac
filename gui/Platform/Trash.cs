using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;

namespace OpenPowerMac.Gui.Platform;

/// <summary>The desktop's Trash, as the freedesktop.org Trash specification
/// (version 1.0) lays it out, so that what goes in shows up in the file
/// manager's Trash and can be put back from there.
///
/// A trashed item is two things written in this order: a record
/// `info/NAME.trashinfo` naming where it came from and when, created so that
/// it fails if the name is taken (that is what reserves NAME), and then the
/// item itself renamed to `files/NAME`. The directory is the home trash,
/// `$XDG_DATA_HOME/Trash`, for anything on the home trash's own file system;
/// for another file system it is that volume's `.Trash-$uid` at its mount
/// point (or the administrator's `.Trash/$uid` where one is set up), because
/// a rename cannot cross file systems and the specification puts the Trash
/// where the item already is.</summary>
internal static class Trash
{
    [DllImport("libc", SetLastError = true)]
    private static extern uint geteuid();

    /// <summary>Move a file or a folder to the Trash. Null when it went;
    /// otherwise why it did not (and it is where it was).</summary>
    public static string? Move(string path)
    {
        // This front end is the Linux one; the layout below is the Unix
        // desktop's and uses Unix file modes.
        if (OperatingSystem.IsWindows())
            return "this front end has no Trash on Windows";
        try
        {
            path = Path.GetFullPath(path).TrimEnd('/');
            bool isDir = Directory.Exists(path);
            if (!isDir && !File.Exists(path))
                return "it is not there";
            string dataHome = Environment.GetEnvironmentVariable("XDG_DATA_HOME") is { Length: > 0 } d
                ? d
                : Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
                               ".local", "share");
            var mounts = MountPoints();
            string top = MountPointOf(path, mounts);
            string trash, recorded;
            if (top == MountPointOf(dataHome, mounts))
            {
                trash = Path.Combine(dataHome, "Trash");
                recorded = path; // the home trash records absolute paths
            }
            else
            {
                uint uid = geteuid();
                string admin = Path.Combine(top, ".Trash");
                // The administrator's directory counts only as the
                // specification says: a real directory with the sticky bit.
                bool adminOk = Directory.Exists(admin)
                               && new DirectoryInfo(admin).LinkTarget == null
                               && (File.GetUnixFileMode(admin) & UnixFileMode.StickyBit) != 0;
                trash = adminOk ? Path.Combine(admin, uid.ToString())
                                : Path.Combine(top, $".Trash-{uid}");
                // A volume's trash records paths relative to the volume.
                recorded = Path.GetRelativePath(top, path);
            }
            string files = Path.Combine(trash, "files"), info = Path.Combine(trash, "info");
            const UnixFileMode Mine = UnixFileMode.UserRead | UnixFileMode.UserWrite
                                      | UnixFileMode.UserExecute;
            Directory.CreateDirectory(trash, Mine);
            Directory.CreateDirectory(files, Mine);
            Directory.CreateDirectory(info, Mine);

            string baseName = Path.GetFileName(path);
            string record = "[Trash Info]\nPath=" + Escape(recorded) + "\nDeletionDate="
                            + DateTime.Now.ToString("yyyy-MM-dd'T'HH:mm:ss") + "\n";
            for (int n = 1; n < 10000; n++)
            {
                string name = n == 1 ? baseName : $"{baseName}.{n}";
                string infoFile = Path.Combine(info, name + ".trashinfo");
                string target = Path.Combine(files, name);
                try
                {
                    using var f = new FileStream(infoFile, FileMode.CreateNew, FileAccess.Write);
                    f.Write(Encoding.UTF8.GetBytes(record));
                }
                catch (IOException) when (File.Exists(infoFile))
                {
                    continue; // that name is taken: try the next
                }
                try
                {
                    if (File.Exists(target) || Directory.Exists(target))
                        throw new IOException("the Trash already holds an item of that name");
                    if (isDir)
                        Directory.Move(path, target);
                    else
                        File.Move(path, target);
                    return null;
                }
                catch (Exception ex)
                {
                    try { File.Delete(infoFile); } catch { }
                    return ex.Message;
                }
            }
            return "no free name in the Trash";
        }
        catch (Exception ex)
        {
            return ex.Message;
        }
    }

    // RFC 2396 escaping of each name in the path, the separators kept.
    private static string Escape(string p) =>
        string.Join('/', p.Split('/').Select(Uri.EscapeDataString));

    // The mount points of this process, longest first.
    private static List<string> MountPoints()
    {
        var list = new List<string>();
        try
        {
            foreach (var line in File.ReadLines("/proc/self/mountinfo"))
            {
                var f = line.Split(' ');
                if (f.Length > 4)
                    list.Add(Unescape(f[4]));
            }
        }
        catch { }
        if (!list.Contains("/"))
            list.Add("/");
        return list.OrderByDescending(m => m.Length).ToList();
    }

    // mountinfo writes a space, a tab, a newline and a backslash as \ooo.
    private static string Unescape(string s)
    {
        if (!s.Contains('\\'))
            return s;
        var bytes = new List<byte>();
        var raw = Encoding.UTF8.GetBytes(s);
        for (int i = 0; i < raw.Length; i++)
        {
            if (raw[i] == (byte)'\\' && i + 3 < raw.Length
                && raw[i + 1] is >= (byte)'0' and <= (byte)'3'
                && raw[i + 2] is >= (byte)'0' and <= (byte)'7'
                && raw[i + 3] is >= (byte)'0' and <= (byte)'7')
            {
                bytes.Add((byte)(((raw[i + 1] - '0') << 6) | ((raw[i + 2] - '0') << 3)
                                 | (raw[i + 3] - '0')));
                i += 3;
            }
            else
                bytes.Add(raw[i]);
        }
        return Encoding.UTF8.GetString(bytes.ToArray());
    }

    private static string MountPointOf(string path, List<string> mounts)
    {
        foreach (var m in mounts)
            if (m == "/" || path == m || path.StartsWith(m + "/", StringComparison.Ordinal))
                return m;
        return "/";
    }
}
