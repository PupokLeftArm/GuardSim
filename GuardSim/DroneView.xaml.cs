using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace GuardSim
{
    public partial class DroneViewWindow : Window
    {
        private DroneSensor _drone;
        private System.Timers.Timer _updateTimer;
        private Random _rand = new Random();

        public DroneViewWindow(DroneSensor drone)
        {
            InitializeComponent();
            _drone = drone;
            DroneNameText.Text = $"БПЛА: {drone.SensorName}";

            _updateTimer = new System.Timers.Timer(33.33); // ~30 FPS
            _updateTimer.Elapsed += (s, e) => Dispatcher.Invoke(UpdateFrame);
            _updateTimer.Start();
        }

        private void UpdateFrame()
        {
            VideoCanvas.Children.Clear();

            var bg = new Rectangle
            {
                Width = VideoCanvas.ActualWidth,
                Height = VideoCanvas.ActualHeight,
                Fill = new SolidColorBrush(Color.FromRgb(12, 16, 20))
            };
            VideoCanvas.Children.Add(bg);

            DrawGrid();

            DrawNoise();

            if (_drone.HasDetection)
            {
                var targetPosition = _drone.LastDetectedPosition;
                double distance = GetDistance(_drone.Position, targetPosition);

                if (distance < _drone.DetectionRange)
                {
                    DrawTarget(targetPosition, _drone.DetectionConfidence);
                    DetectionStatusText.Text = "⚫ ОБНАРУЖЕНО";
                    DetectionStatusText.Foreground = Brushes.LimeGreen;
                }
                else
                {
                    DetectionStatusText.Text = "⚫ Цель неактуальна";
                    DetectionStatusText.Foreground = Brushes.Gray;
                }
            }
            else
            {
                DetectionStatusText.Text = "⚫ Нет обнаружения";
                DetectionStatusText.Foreground = Brushes.Gray;
            }

            TimestampText.Text = DateTime.Now.ToString("HH:mm:ss.fff");
            PositionText.Text = $"X: {_drone.LastDetectedPosition.X:F0}, Y: {_drone.LastDetectedPosition.Y:F0}";
            ConfidenceText.Text = $"Уверенность: {(_drone.DetectionConfidence * 100):F1}%";
        }

        private double GetDistance(Point p1, Point p2)
        {
            return Math.Sqrt(Math.Pow(p2.X - p1.X, 2) + Math.Pow(p2.Y - p1.Y, 2));
        }

        private void DrawGrid()
        {
            double w = Math.Max(1, VideoCanvas.ActualWidth);
            double h = Math.Max(1, VideoCanvas.ActualHeight);
            for (int i = 0; i < 12; i++)
            {
                double x = i * w / 12.0;
                var v = new Line { X1 = x, Y1 = 0, X2 = x, Y2 = h, Stroke = new SolidColorBrush(Color.FromArgb(35, 120, 200, 255)), StrokeThickness = 1 };
                VideoCanvas.Children.Add(v);
            }
            for (int j = 0; j < 8; j++)
            {
                double y = j * h / 8.0;
                var l = new Line { X1 = 0, Y1 = y, X2 = w, Y2 = y, Stroke = new SolidColorBrush(Color.FromArgb(35, 120, 200, 255)), StrokeThickness = 1 };
                VideoCanvas.Children.Add(l);
            }
        }

        private void DrawNoise()
        {
            int n = (int)(VideoCanvas.ActualWidth * VideoCanvas.ActualHeight * 0.0006);
            for (int i = 0; i < n; i++)
            {
                double x = _rand.NextDouble() * Math.Max(1, VideoCanvas.ActualWidth);
                double y = _rand.NextDouble() * Math.Max(1, VideoCanvas.ActualHeight);
                byte b = (byte)_rand.Next(40, 90);
                var px = new Rectangle { Width = 2, Height = 2, Fill = new SolidColorBrush(Color.FromRgb(b, b, b)) };
                Canvas.SetLeft(px, x);
                Canvas.SetTop(px, y);
                VideoCanvas.Children.Add(px);
            }
        }

        private void DrawTarget(Point mapPos, double conf)
        {
            double w = Math.Max(1, VideoCanvas.ActualWidth);
            double h = Math.Max(1, VideoCanvas.ActualHeight);
            double sx = (mapPos.X / 1920.0) * w;
            double sy = (mapPos.Y / 1080.0) * h;

            const double r = 18;
            var circle = new Ellipse { Width = r * 2, Height = r * 2, Stroke = Brushes.DeepSkyBlue, StrokeThickness = 2 };
            Canvas.SetLeft(circle, sx - r);
            Canvas.SetTop(circle, sy - r);
            VideoCanvas.Children.Add(circle);

            var crossH = new Line { X1 = sx - r, Y1 = sy, X2 = sx + r, Y2 = sy, Stroke = Brushes.DeepSkyBlue, StrokeThickness = 1 };
            var crossV = new Line { X1 = sx, Y1 = sy - r, X2 = sx, Y2 = sy + r, Stroke = Brushes.DeepSkyBlue, StrokeThickness = 1 };
            VideoCanvas.Children.Add(crossH);
            VideoCanvas.Children.Add(crossV);

            var info = new TextBlock
            {
                Text = $"TARGET\n{conf * 100:F0}%",
                Foreground = Brushes.DeepSkyBlue,
                FontSize = 9,
                Background = new SolidColorBrush(Color.FromArgb(180, 0, 0, 0)),
                Padding = new Thickness(4, 2, 4, 2)
            };
            Canvas.SetLeft(info, sx + r + 6);
            Canvas.SetTop(info, sy - r);
            VideoCanvas.Children.Add(info);
        }

        private void CloseBtn_Click(object sender, RoutedEventArgs e)
        {
            _updateTimer?.Stop();
            _updateTimer?.Dispose();
            Close();
        }

        protected override void OnClosed(EventArgs e)
        {
            _updateTimer?.Stop();
            _updateTimer?.Dispose();
            base.OnClosed(e);
        }
    }
}