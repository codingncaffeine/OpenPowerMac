using Avalonia.Controls;
using Avalonia.Platform.Storage;

namespace OpenPowerMac.Gui.Dialogs;

/// <summary>The desktop's own file dialogs (the XDG portal on KDE and GNOME),
/// reached through Avalonia's storage provider and answered in line, like the
/// Windows ones: the caller acts on the result on the next line. Each picker
/// starts in the folder of the file it is replacing, when there is one.</summary>
internal static class FilePicker
{
    public static string? Open(Window owner, string title, string filter, string? current = null)
    {
        IStorageProvider sp = owner.StorageProvider;
        var options = new FilePickerOpenOptions
        {
            Title = title,
            AllowMultiple = false,
            FileTypeFilter = ParseFilter(filter),
            SuggestedStartLocation = StartFolder(sp, current),
        };
        IReadOnlyList<IStorageFile> files = ModalLoop.Wait(sp.OpenFilePickerAsync(options));
        return files.Count > 0 ? files[0].TryGetLocalPath() : null;
    }

    /// <summary>Where to write a new file. <paramref name="overwritePrompt"/>
    /// false when the caller asks its own, better question.</summary>
    public static string? Save(Window owner, string title, string filter, string suggestedName,
                               string defaultExt, string? current = null, bool overwritePrompt = true)
    {
        IStorageProvider sp = owner.StorageProvider;
        var options = new FilePickerSaveOptions
        {
            Title = title,
            FileTypeChoices = ParseFilter(filter),
            DefaultExtension = defaultExt.TrimStart('.'),
            SuggestedFileName = suggestedName,
            ShowOverwritePrompt = overwritePrompt,
            SuggestedStartLocation = StartFolder(sp, current),
        };
        IStorageFile? file = ModalLoop.Wait(sp.SaveFilePickerAsync(options));
        return file?.TryGetLocalPath();
    }

    public static string? OpenFolder(Window owner, string title, string? initial)
    {
        IStorageProvider sp = owner.StorageProvider;
        var options = new FolderPickerOpenOptions { Title = title, AllowMultiple = false };
        if (!string.IsNullOrEmpty(initial) && Directory.Exists(initial))
            options.SuggestedStartLocation = ModalLoop.Wait(sp.TryGetFolderFromPathAsync(initial));
        IReadOnlyList<IStorageFolder> folders = ModalLoop.Wait(sp.OpenFolderPickerAsync(options));
        return folders.Count > 0 ? folders[0].TryGetLocalPath() : null;
    }

    private static IStorageFolder? StartFolder(IStorageProvider sp, string? current)
    {
        if (string.IsNullOrWhiteSpace(current))
            return null;
        string? dir = Directory.Exists(current) ? current : Path.GetDirectoryName(current);
        return string.IsNullOrEmpty(dir) || !Directory.Exists(dir)
            ? null : ModalLoop.Wait(sp.TryGetFolderFromPathAsync(dir));
    }

    // "Description (*.a;*.b)|*.a;*.b|All files (*.*)|*.*" -> picker file types.
    // Linux pickers match case-sensitively and period media is as often named
    // .ROM or .ISO as .rom or .iso, so every pattern also goes in upper case.
    private static List<FilePickerFileType> ParseFilter(string filter)
    {
        var types = new List<FilePickerFileType>();
        string[] parts = filter.Split('|');
        for (int i = 0; i + 1 < parts.Length; i += 2)
        {
            var patterns = new List<string>();
            foreach (string raw in parts[i + 1].Split(';', StringSplitOptions.RemoveEmptyEntries))
            {
                string p = raw.Trim() == "*.*" ? "*" : raw.Trim();
                if (!patterns.Contains(p)) patterns.Add(p);
                string upper = p.ToUpperInvariant();
                if (!patterns.Contains(upper)) patterns.Add(upper);
            }
            types.Add(new FilePickerFileType(parts[i]) { Patterns = patterns });
        }
        return types;
    }
}
