using System.Windows;


namespace OpenSCADEnhanced
{
    public partial class ProfileWindow : Window
    {
        public ProfileWindow()
        {
            InitializeComponent();
        }
        private void OK_Click(object sender, RoutedEventArgs e)
        {
            this.Close();
        }
    }
}
