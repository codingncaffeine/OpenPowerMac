using Avalonia.Input;

namespace OpenPowerMac.Gui.Platform;

/// <summary>Host key to USB HID usage code (HID Usage Tables, keyboard page).
///
/// A keyboard is not a text device: it sends a usage going down and the same
/// usage coming up, and the GUEST decides what character that is. Keys are
/// read by physical position (<see cref="PhysicalKey"/>), so the guest applies
/// its own layout exactly as it would to a real USB keyboard.
///
/// Modifiers follow the Windows front end (shell/HidKeys.cs) so the two feel
/// the same: Left Alt is COMMAND, Right Alt is Option, and both Super keys
/// are Command too.</summary>
internal static class HidKeys
{
    /// <summary>0 when the key has no HID usage worth sending.</summary>
    public static uint Usage(PhysicalKey k) => k switch
    {
        >= PhysicalKey.A and <= PhysicalKey.Z => (uint)(4 + (k - PhysicalKey.A)),
        // The number row: 1-9 are contiguous from usage 30 and 0 follows them.
        >= PhysicalKey.Digit1 and <= PhysicalKey.Digit9 => (uint)(30 + (k - PhysicalKey.Digit1)),
        PhysicalKey.Digit0 => 39,

        PhysicalKey.Enter => 40,
        PhysicalKey.Escape => 41,
        PhysicalKey.Backspace => 42,
        PhysicalKey.Tab => 43,
        PhysicalKey.Space => 44,
        PhysicalKey.Minus => 45,
        PhysicalKey.Equal => 46,
        PhysicalKey.BracketLeft => 47,
        PhysicalKey.BracketRight => 48,
        PhysicalKey.Backslash => 49,
        PhysicalKey.Semicolon => 51,
        PhysicalKey.Quote => 52,
        PhysicalKey.Backquote => 53,
        PhysicalKey.Comma => 54,
        PhysicalKey.Period => 55,
        PhysicalKey.Slash => 56,
        PhysicalKey.CapsLock => 57,

        >= PhysicalKey.F1 and <= PhysicalKey.F12 => (uint)(58 + (k - PhysicalKey.F1)),

        PhysicalKey.PrintScreen => 70,
        PhysicalKey.ScrollLock => 71,
        PhysicalKey.Pause => 72,
        PhysicalKey.Insert => 73,
        PhysicalKey.Home => 74,
        PhysicalKey.PageUp => 75,
        PhysicalKey.Delete => 76,
        PhysicalKey.End => 77,
        PhysicalKey.PageDown => 78,
        PhysicalKey.ArrowRight => 79,
        PhysicalKey.ArrowLeft => 80,
        PhysicalKey.ArrowDown => 81,
        PhysicalKey.ArrowUp => 82,

        PhysicalKey.NumLock => 83,
        PhysicalKey.NumPadDivide => 84,
        PhysicalKey.NumPadMultiply => 85,
        PhysicalKey.NumPadSubtract => 86,
        PhysicalKey.NumPadAdd => 87,
        PhysicalKey.NumPadEnter => 88,
        // The keypad's 1-9 are contiguous from usage 89 and its 0 follows them.
        >= PhysicalKey.NumPad1 and <= PhysicalKey.NumPad9 => (uint)(89 + (k - PhysicalKey.NumPad1)),
        PhysicalKey.NumPad0 => 98,
        PhysicalKey.NumPadDecimal => 99,
        PhysicalKey.IntlBackslash => 100,
        PhysicalKey.NumPadEqual => 103,
        PhysicalKey.Help => 117,

        PhysicalKey.ControlLeft => 0xE0,
        PhysicalKey.ShiftLeft => 0xE1,
        PhysicalKey.AltRight => 0xE2,   // Option
        PhysicalKey.AltLeft => 0xE3,    // Command
        PhysicalKey.MetaLeft => 0xE3,   // Command
        PhysicalKey.ControlRight => 0xE4,
        PhysicalKey.ShiftRight => 0xE5,
        PhysicalKey.MetaRight => 0xE7,  // Command

        _ => 0,
    };

    /// <summary>The usage for a Linux kernel (evdev) key code, which is what
    /// the X11 keyboard grab delivers.</summary>
    public static uint UsageForEvdev(int code) =>
        code >= 0 && code < Evdev.Length ? Usage(Evdev[code]) : 0;

    /// <summary>Escape's kernel code: releases the capture.</summary>
    public const int EvdevEscape = 1;

    private static readonly PhysicalKey[] Evdev = BuildEvdev();

    // linux/input-event-codes.h, the keys a desktop keyboard carries.
    private static PhysicalKey[] BuildEvdev()
    {
        var t = new PhysicalKey[140];
        void Run(int first, params PhysicalKey[] keys)
        {
            for (int i = 0; i < keys.Length; i++) t[first + i] = keys[i];
        }
        t[1] = PhysicalKey.Escape;
        Run(2, PhysicalKey.Digit1, PhysicalKey.Digit2, PhysicalKey.Digit3, PhysicalKey.Digit4, PhysicalKey.Digit5,
               PhysicalKey.Digit6, PhysicalKey.Digit7, PhysicalKey.Digit8, PhysicalKey.Digit9, PhysicalKey.Digit0,
               PhysicalKey.Minus, PhysicalKey.Equal, PhysicalKey.Backspace, PhysicalKey.Tab);
        Run(16, PhysicalKey.Q, PhysicalKey.W, PhysicalKey.E, PhysicalKey.R, PhysicalKey.T, PhysicalKey.Y,
                PhysicalKey.U, PhysicalKey.I, PhysicalKey.O, PhysicalKey.P,
                PhysicalKey.BracketLeft, PhysicalKey.BracketRight, PhysicalKey.Enter, PhysicalKey.ControlLeft);
        Run(30, PhysicalKey.A, PhysicalKey.S, PhysicalKey.D, PhysicalKey.F, PhysicalKey.G, PhysicalKey.H,
                PhysicalKey.J, PhysicalKey.K, PhysicalKey.L, PhysicalKey.Semicolon, PhysicalKey.Quote,
                PhysicalKey.Backquote, PhysicalKey.ShiftLeft, PhysicalKey.Backslash);
        Run(44, PhysicalKey.Z, PhysicalKey.X, PhysicalKey.C, PhysicalKey.V, PhysicalKey.B, PhysicalKey.N,
                PhysicalKey.M, PhysicalKey.Comma, PhysicalKey.Period, PhysicalKey.Slash, PhysicalKey.ShiftRight,
                PhysicalKey.NumPadMultiply, PhysicalKey.AltLeft, PhysicalKey.Space, PhysicalKey.CapsLock);
        Run(59, PhysicalKey.F1, PhysicalKey.F2, PhysicalKey.F3, PhysicalKey.F4, PhysicalKey.F5,
                PhysicalKey.F6, PhysicalKey.F7, PhysicalKey.F8, PhysicalKey.F9, PhysicalKey.F10,
                PhysicalKey.NumLock, PhysicalKey.ScrollLock);
        Run(71, PhysicalKey.NumPad7, PhysicalKey.NumPad8, PhysicalKey.NumPad9, PhysicalKey.NumPadSubtract,
                PhysicalKey.NumPad4, PhysicalKey.NumPad5, PhysicalKey.NumPad6, PhysicalKey.NumPadAdd,
                PhysicalKey.NumPad1, PhysicalKey.NumPad2, PhysicalKey.NumPad3, PhysicalKey.NumPad0,
                PhysicalKey.NumPadDecimal);
        t[86] = PhysicalKey.IntlBackslash;
        t[87] = PhysicalKey.F11;
        t[88] = PhysicalKey.F12;
        Run(96, PhysicalKey.NumPadEnter, PhysicalKey.ControlRight, PhysicalKey.NumPadDivide,
                PhysicalKey.PrintScreen, PhysicalKey.AltRight);
        Run(102, PhysicalKey.Home, PhysicalKey.ArrowUp, PhysicalKey.PageUp, PhysicalKey.ArrowLeft,
                 PhysicalKey.ArrowRight, PhysicalKey.End, PhysicalKey.ArrowDown, PhysicalKey.PageDown,
                 PhysicalKey.Insert, PhysicalKey.Delete);
        t[117] = PhysicalKey.NumPadEqual;
        t[119] = PhysicalKey.Pause;
        t[125] = PhysicalKey.MetaLeft;
        t[126] = PhysicalKey.MetaRight;
        t[138] = PhysicalKey.Help;
        return t;
    }
}
