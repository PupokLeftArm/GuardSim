using System;
using System.Windows;
using System.Windows.Controls;

namespace GuardSim
{
    public class DroneViewEventArgs : EventArgs
    {
        public string DroneId { get; set; }
    }

    public class CameraViewEventArgs : EventArgs
    {
        public string CameraId { get; set; }
    }

    public partial class SensorPanelControl : UserControl
    {
        public static readonly DependencyProperty SensorSimulatorProperty =
            DependencyProperty.Register("SensorSimulator", typeof(SensorDataSimulator),
                typeof(SensorPanelControl), new PropertyMetadata(null, OnSensorSimulatorChanged));

        public event EventHandler<DroneViewEventArgs> DroneViewRequested;

        private void DroneViewBtn_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button btn && btn.Tag is string droneId)
                DroneViewRequested?.Invoke(this, new DroneViewEventArgs { DroneId = droneId });
        }

        public SensorDataSimulator SensorSimulator
        {
            get { return (SensorDataSimulator)GetValue(SensorSimulatorProperty); }
            set { SetValue(SensorSimulatorProperty, value); }
        }

        public event EventHandler<CameraViewEventArgs> CameraViewRequested;

        private static void OnSensorSimulatorChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            var control = d as SensorPanelControl;
            control?.UpdateSensorDisplays();
        }

        public SensorPanelControl()
        {
            InitializeComponent();
        }

        public class DroneViewEventArgs : EventArgs
        {
            public string DroneId { get; set; }
        }

        private void UpdateSensorDisplays()
        {
            if (SensorSimulator == null) return;

            RadarItemsControl.ItemsSource = SensorSimulator.Radars;
            CameraItemsControl.ItemsSource = SensorSimulator.Cameras;
            DroneItemsControl.ItemsSource = SensorSimulator.Drones;
        }

        private void CameraViewBtn_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button btn && btn.Tag is string cameraId)
                CameraViewRequested?.Invoke(this, new CameraViewEventArgs { CameraId = cameraId });
        }
    }
}