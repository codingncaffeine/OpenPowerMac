using System.Windows.Input;

namespace OpenPowerMac.Shell;

/// <summary>Host key to USB HID usage code.
///
/// A keyboard is not a text device. It sends a usage going down and the same
/// usage coming up, and the GUEST decides what character that is — which is
/// why routing typed text instead left Backspace, Tab, the arrows, Delete,
/// the function keys and every modifier with no way through at all, and left
/// letters depending on whether WPF happened to raise a TextInput event.
///
/// Usages are HID Usage Table 1.12 chapter 10, which is also what the boot
/// protocol Mac OS binds to expects. 0xE0-0xE7 are the modifiers.</summary>
internal static class HidKeys
{
    /// <summary>0 when the key has no HID usage worth sending.</summary>
    public static uint Usage(Key k) => k switch
    {
        >= Key.A and <= Key.Z => (uint)(4 + (k - Key.A)),

        // The number row: 1-9 are contiguous from usage 30, and 0 follows
        // them rather than preceding them.
        >= Key.D1 and <= Key.D9 => (uint)(30 + (k - Key.D1)),
        Key.D0 => 39,

        Key.Return => 40,
        Key.Escape => 41,
        Key.Back => 42,
        Key.Tab => 43,
        Key.Space => 44,
        Key.OemMinus => 45,
        Key.OemPlus => 46,
        Key.OemOpenBrackets => 47,
        Key.OemCloseBrackets => 48,
        Key.OemPipe => 49,
        Key.OemSemicolon => 51,
        Key.OemQuotes => 52,
        Key.OemTilde => 53,
        Key.OemComma => 54,
        Key.OemPeriod => 55,
        Key.OemQuestion => 56,
        Key.CapsLock => 57,

        >= Key.F1 and <= Key.F12 => (uint)(58 + (k - Key.F1)),

        Key.PrintScreen => 70,
        Key.Scroll => 71,
        Key.Pause => 72,
        Key.Insert => 73,
        Key.Home => 74,
        Key.PageUp => 75,
        Key.Delete => 76,
        Key.End => 77,
        Key.Next => 78, // Page Down
        Key.Right => 79,
        Key.Left => 80,
        Key.Down => 81,
        Key.Up => 82,

        Key.NumLock => 83,
        Key.Divide => 84,
        Key.Multiply => 85,
        Key.Subtract => 86,
        Key.Add => 87,
        // The keypad's 1-9 are contiguous from usage 89 and, like the number
        // row, its 0 comes after them.
        >= Key.NumPad1 and <= Key.NumPad9 => (uint)(89 + (k - Key.NumPad1)),
        Key.NumPad0 => 98,
        Key.Decimal => 99,

        // Modifiers. Left Alt is mapped to COMMAND rather than Option, and
        // deliberately: Command is the Mac's primary modifier and the key
        // that would carry it faithfully — Windows — is swallowed by the host
        // before WPF sees the combination, so a faithful map would leave
        // Command-anything unreachable. Right Alt stays Option, and both
        // Windows keys are Command for anyone whose habit says so.
        Key.LeftCtrl => 0xE0,
        Key.LeftShift => 0xE1,
        Key.RightAlt => 0xE2,  // Option
        Key.LeftAlt => 0xE3,   // Command
        Key.LWin => 0xE3,
        Key.RightCtrl => 0xE4,
        Key.RightShift => 0xE5,
        Key.RWin => 0xE7,

        _ => 0,
    };
}
