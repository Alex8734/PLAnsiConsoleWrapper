using System.Diagnostics;
using System.Text;
using System.Threading.Tasks;

using ConsoleApp1keineAhnung2;

Console.Clear();
Console.OutputEncoding = Encoding.UTF8;
ConsoleHelper.EnableEscapeSequences();

ProcessStartInfo startInfo = new ProcessStartInfo
{
    FileName = "swipl",
    RedirectStandardOutput = true,
    RedirectStandardInput = true,
    UseShellExecute = false,
    CreateNoWindow = false,
};

Process process = new Process
{
    StartInfo = startInfo,
};

process.Start();

// Start a task to pipe the output of the process to the console
Task.Run(() =>
{
    char[] buffer = new char[256];
    int charsRead;
    while ((charsRead = process.StandardOutput.Read(buffer, 0, buffer.Length)) > 0)
    {
        Console.Write(new string(buffer, 0, charsRead));
    }
});

// Start a task to pipe the input of the console to the process
Task.Run(() =>
{
    string line;
    while ((line = Console.ReadLine()) != null)
    {
        process.StandardInput.WriteLine(line);
    }
});

process.WaitForExit();