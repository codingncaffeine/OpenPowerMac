using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;

namespace OpenPowerMac.Gui.Dialogs;

public enum MessageBoxButton { OK, YesNo }
public enum MessageBoxImage { None, Information, Warning, Error, Question }
public enum MessageBoxResult { None, OK, Yes, No }

/// <summary>
/// A modal message box in the app's own theme. Returns when the person has
/// answered, like the Windows one, so a caller can act on the answer directly.
/// Closing the box without choosing counts as the safe answer: OK for a notice,
/// No for a question.
/// </summary>
internal static class MessageBox
{
    public static MessageBoxResult Show(string text, string caption,
                                        MessageBoxButton buttons, MessageBoxImage image) =>
        Show(null, text, caption, buttons, image);

    public static MessageBoxResult Show(Window? owner, string text, string caption,
                                        MessageBoxButton buttons = MessageBoxButton.OK,
                                        MessageBoxImage image = MessageBoxImage.None)
    {
        MessageBoxResult result = buttons == MessageBoxButton.YesNo
            ? MessageBoxResult.No : MessageBoxResult.OK;

        var dialog = new Window
        {
            Title = caption,
            SizeToContent = SizeToContent.WidthAndHeight,
            CanResize = false,
            ShowInTaskbar = false,
            MinWidth = 340,
            MaxWidth = 560,
            WindowStartupLocation = owner is null
                ? WindowStartupLocation.CenterScreen : WindowStartupLocation.CenterOwner,
        };

        var buttonRow = new StackPanel
        {
            Orientation = Orientation.Horizontal,
            HorizontalAlignment = HorizontalAlignment.Right,
            Spacing = 10,
            Margin = new Thickness(0, 18, 0, 0),
        };

        Button AddButton(string label, MessageBoxResult answer, bool accent, bool isDefault, bool isCancel)
        {
            var b = new Button
            {
                Content = label,
                MinWidth = 90,
                IsDefault = isDefault,
                IsCancel = isCancel,
                HorizontalContentAlignment = HorizontalAlignment.Center,
            };
            if (accent) b.Classes.Add("accent");
            b.Click += (_, _) => { result = answer; dialog.Close(); };
            buttonRow.Children.Add(b);
            return b;
        }

        Button first;
        if (buttons == MessageBoxButton.YesNo)
        {
            AddButton("No", MessageBoxResult.No, false, false, true);
            first = AddButton("Yes", MessageBoxResult.Yes, true, true, false);
        }
        else
        {
            first = AddButton("OK", MessageBoxResult.OK, true, true, true);
        }

        var body = new Grid
        {
            ColumnDefinitions = new ColumnDefinitions("Auto,*"),
            RowDefinitions = new RowDefinitions("Auto,Auto"),
            Margin = new Thickness(20),
        };
        if (Glyph(image) is { } glyph)
        {
            var mark = new Border
            {
                Width = 30,
                Height = 30,
                CornerRadius = new CornerRadius(15),
                Margin = new Thickness(0, 0, 14, 0),
                VerticalAlignment = VerticalAlignment.Top,
                Child = new TextBlock
                {
                    Text = glyph,
                    FontWeight = FontWeight.Bold,
                    FontSize = 16,
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Center,
                },
            };
            mark.Classes.Add("message-glyph");
            if (image is MessageBoxImage.Warning or MessageBoxImage.Error) mark.Classes.Add("alert");
            body.Children.Add(mark);
        }
        var message = new TextBlock
        {
            Text = text,
            TextWrapping = TextWrapping.Wrap,
            MaxWidth = 460,
            VerticalAlignment = VerticalAlignment.Center,
        };
        Grid.SetColumn(message, 1);
        body.Children.Add(message);
        Grid.SetRow(buttonRow, 1);
        Grid.SetColumnSpan(buttonRow, 2);
        body.Children.Add(buttonRow);
        dialog.Content = body;

        dialog.Opened += (_, _) => first.Focus(NavigationMethod.Tab);

        if (owner is { IsVisible: true })
        {
            ModalLoop.Wait(dialog.ShowDialog(owner));
        }
        else
        {
            var closed = new TaskCompletionSource();
            dialog.Closed += (_, _) => closed.TrySetResult();
            dialog.Show();
            ModalLoop.Wait(closed.Task);
        }
        return result;
    }

    private static string? Glyph(MessageBoxImage image) => image switch
    {
        MessageBoxImage.Information => "i",
        MessageBoxImage.Question => "?",
        MessageBoxImage.Warning => "!",
        MessageBoxImage.Error => "×",
        _ => null,
    };
}
