using Avalonia.Threading;

namespace OpenPowerMac.Gui.Dialogs;

/// <summary>
/// Waits for a UI task by running the dispatcher until it finishes. Avalonia's
/// dialogs and pickers are asynchronous; the front end asks its questions the way
/// a modal dialog does, in the middle of a decision ("restart now?", "which
/// file?"), and acts on the answer on the next line. Running a nested loop keeps
/// that shape: the window keeps drawing and the emulation thread keeps running
/// while the question is up, exactly as under a modal dialog.
/// </summary>
internal static class ModalLoop
{
    public static T Wait<T>(Task<T> task)
    {
        if (!task.IsCompleted)
        {
            using var done = new CancellationTokenSource();
            task.ContinueWith(_ => Dispatcher.UIThread.Post(done.Cancel), TaskScheduler.Default);
            Dispatcher.UIThread.MainLoop(done.Token);
        }
        return task.GetAwaiter().GetResult();
    }

    public static void Wait(Task task) =>
        Wait(task.ContinueWith(t => { t.GetAwaiter().GetResult(); return true; }, TaskScheduler.Default));
}
