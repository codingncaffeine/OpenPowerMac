using System.Text;

namespace OpenPowerMac.Shell;

/// <summary>Minimal VT interpreter for the serial console. Open Firmware's
/// line editor drives the terminal with CSI sequences (insert-char, cursor
/// forward/back, erase) rather than plain text, so a raw append log renders
/// as garbage. This keeps a line buffer + cursor and applies the handful of
/// controls the firmware actually uses; anything else is swallowed.</summary>
public sealed class TerminalBuffer
{
    private readonly List<StringBuilder> _lines = [new()];
    private int _row, _col;
    private bool _dirty;

    private enum St { Normal, Esc, Csi }
    private St _st = St.Normal;
    private readonly StringBuilder _csi = new();

    public void Feed(string s)
    {
        foreach (char c in s)
            FeedChar(c);
    }

    private void FeedChar(char c)
    {
        switch (_st)
        {
            case St.Esc:
                if (c == '[') { _st = St.Csi; _csi.Clear(); }
                else { _st = St.Normal; FeedChar(c); }
                return;
            case St.Csi:
                if (c >= '@' && c <= '~') { _st = St.Normal; ApplyCsi(c); }
                else _csi.Append(c);
                return;
        }
        switch (c)
        {
            case '\x1B': _st = St.Esc; return;
            case '\r': _col = 0; return;
            case '\n':
                _row++;
                while (_row >= _lines.Count) _lines.Add(new StringBuilder());
                Trim();
                _dirty = true;
                return;
            case '\b': if (_col > 0) _col--; return;
            case '\t': _col = (_col & ~7) + 8; return;
            default:
                if (c < ' ') return; // BEL and friends
                Put(c);
                return;
        }
    }

    private void ApplyCsi(char final)
    {
        int n = 1;
        var parts = _csi.ToString().Split(';');
        if (parts.Length > 0 && int.TryParse(parts[0], out int v) && v > 0)
            n = v;
        var line = _lines[_row];
        switch (final)
        {
            case '@': // insert blanks at cursor
                PadTo(_col);
                line.Insert(_col, new string(' ', n));
                _dirty = true;
                break;
            case 'C': _col += n; break;
            case 'D': _col = Math.Max(0, _col - n); break;
            case 'K': // erase to end of line
                if (_col < line.Length) { line.Length = _col; _dirty = true; }
                break;
            case 'J': // erase to end of screen
                if (_col < line.Length) line.Length = _col;
                if (_row + 1 < _lines.Count)
                    _lines.RemoveRange(_row + 1, _lines.Count - _row - 1);
                _dirty = true;
                break;
            case 'P': // delete chars at cursor
                if (_col < line.Length)
                {
                    line.Remove(_col, Math.Min(n, line.Length - _col));
                    _dirty = true;
                }
                break;
            // 'H'/'m'/others: absolute motion makes no sense against a
            // scrollback log — ignore rather than corrupt earlier output.
        }
    }

    private void Put(char c)
    {
        var line = _lines[_row];
        PadTo(_col);
        if (_col < line.Length) line[_col] = c;
        else line.Append(c);
        _col++;
        _dirty = true;
    }

    private void PadTo(int col)
    {
        var line = _lines[_row];
        while (line.Length < col) line.Append(' ');
    }

    private void Trim()
    {
        const int max = 5000;
        if (_lines.Count <= max) return;
        int drop = _lines.Count - max;
        _lines.RemoveRange(0, drop);
        _row = Math.Max(0, _row - drop);
    }

    /// <summary>True once since the last call if the visible text changed.</summary>
    public bool TakeDirty()
    {
        bool d = _dirty;
        _dirty = false;
        return d;
    }

    public string Render()
    {
        var sb = new StringBuilder();
        for (int i = 0; i < _lines.Count; i++)
        {
            if (i > 0) sb.Append('\n');
            sb.Append(_lines[i]);
        }
        return sb.ToString();
    }
}
