using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace OpenSCADEnhanced
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            Log("System Ready");
        }

        private void BtnExplorer_Click(object sender, RoutedEventArgs e) => SwitchPanel(ViewExplorer);
        private void BtnSearch_Click(object sender, RoutedEventArgs e) => SwitchPanel(ViewSearch);
        private void BtnGit_Click(object sender, RoutedEventArgs e) => SwitchPanel(ViewGit);

        private void SwitchPanel(UIElement targetView)
        {
            // Якщо панель закрита - відкриваємо
            if (SidePanelContainer.Visibility == Visibility.Collapsed)
            {
                SidePanelContainer.Visibility = Visibility.Visible;
                ShowView(targetView);
            }
            // Якщо відкрита і натиснули на ту ж саму - закриваємо
            else if (targetView.Visibility == Visibility.Visible)
            {
                SidePanelContainer.Visibility = Visibility.Collapsed;
            }
            // Якщо відкрита інша вкладка - перемикаємо
            else
            {
                ShowView(targetView);
            }
        }

        private void ShowView(UIElement view)
        {
            ViewExplorer.Visibility = Visibility.Collapsed;
            ViewSearch.Visibility = Visibility.Collapsed;
            ViewGit.Visibility = Visibility.Collapsed;
            view.Visibility = Visibility.Visible;
        }

        // --- Windows Logic ---
        private void BtnSettings_Click(object sender, RoutedEventArgs e)
        {
            SettingsWindow settings = new SettingsWindow();
            settings.Owner = this; // Робимо вікно залежним
            settings.ShowDialog(); // Відкриваємо як модальне
        }

        private void BtnProfile_Click(object sender, RoutedEventArgs e)
        {
            ProfileWindow profile = new ProfileWindow();
            profile.Owner = this;
            profile.ShowDialog();
        }

        // --- Workspace Logic ---
        private void BtnRender_Click(object sender, RoutedEventArgs e)
        {
            Log("Compiling design... (CSG generation started)");
            // Емуляція затримки
            Log("Parsing geometry...");
            Log("Rendering finished. Faces: 24. Time: 0.05s");
        }

        private void BtnReset_Click(object sender, RoutedEventArgs e)
        {
            Log("Camera view reset to default.");
        }

        private void BtnGrid_Click(object sender, RoutedEventArgs e)
        {
            Log("Grid toggled.");
        }

        private void BtnClearLog_Click(object sender, RoutedEventArgs e)
        {
            ConsoleOutput.Text = "";
        }

        private void File_Click(object sender, RoutedEventArgs e)
        {
            Log("Switched active file.");
        }

        private void Log(string message)
        {
            ConsoleOutput.AppendText($"> {message}\n");
            ConsoleOutput.ScrollToEnd();
        }
    }
}