using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace GuardSim
{

    public partial class CameraViewWindow : Window
    {
        private VideoCameraSensor _camera;
        private System.Timers.Timer _updateTimer;
        private Random _random;
        private Point _lastIntruderPosition;

        public CameraViewWindow(VideoCameraSensor camera)
        {
            InitializeComponent();
            _camera = camera;
            _random = new Random();

            CameraNameText.Text = $"Камера: {camera.SensorName}";

            _updateTimer = new System.Timers.Timer(33.33);
            _updateTimer.Elapsed += UpdateVideoFrame;
            _updateTimer.Start();
        }

        private void UpdateVideoFrame(object sender, System.Timers.ElapsedEventArgs e)
        {
            Dispatcher.Invoke(() =>
            {
                VideoCanvas.Children.Clear();

                var bgRect = new Rectangle
                {
                    Width = VideoCanvas.ActualWidth,
                    Height = VideoCanvas.ActualHeight,
                    Fill = new SolidColorBrush(Color.FromRgb(20, 20, 30))
                };
                VideoCanvas.Children.Add(bgRect);
                DrawVideoNoise();

                if (_camera.HasDetection && _camera.IsActivated)
                {
                    double distance = GetDistance(_camera.Position, _camera.LastDetectedPosition);

                    if (distance < _camera.DetectionRange)
                    {
                        DrawDetectionBox(_camera.LastDetectedPosition);
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
                    DetectionStatusText.Foreground = Brushes.Red;
                }

                TimestampText.Text = DateTime.Now.ToString("HH:mm:ss.fff");
                PositionText.Text = $"X: {_camera.LastDetectedPosition.X:F0}, Y: {_camera.LastDetectedPosition.Y:F0}";
                ConfidenceText.Text = $"Уверенность: {(_camera.DetectionConfidence * 100):F1}%";
            });
        }
        private void DrawVideoNoise()
        {
            int noisePixels = (int)(VideoCanvas.ActualWidth * VideoCanvas.ActualHeight * 0.001);
            for (int i = 0; i < noisePixels; i++)
            {
                double x = _random.NextDouble() * VideoCanvas.ActualWidth;
                double y = _random.NextDouble() * VideoCanvas.ActualHeight;
                byte brightness = (byte)_random.Next(50, 100);

                var pixel = new Rectangle
                {
                    Width = 2,
                    Height = 2,
                    Fill = new SolidColorBrush(Color.FromRgb(brightness, brightness, brightness))
                };

                Canvas.SetLeft(pixel, x);
                Canvas.SetTop(pixel, y);
                VideoCanvas.Children.Add(pixel);
            }
        }
        private void DrawDetectionBox(Point detectedPos)
        {
            double canvasWidth = VideoCanvas.ActualWidth;
            double canvasHeight = VideoCanvas.ActualHeight;

            double scaledX = (detectedPos.X / 1920.0) * canvasWidth;
            double scaledY = (detectedPos.Y / 1080.0) * canvasHeight;

            const double boxSize = 30;

            var detectionBox = new Rectangle
            {
                Width = boxSize,
                Height = boxSize,
                Stroke = Brushes.Lime,
                StrokeThickness = 2,
                Fill = null
            };

            Canvas.SetLeft(detectionBox, scaledX - boxSize / 2);
            Canvas.SetTop(detectionBox, scaledY - boxSize / 2);
            VideoCanvas.Children.Add(detectionBox);

            var crossH = new Line
            {
                X1 = scaledX - 15,
                Y1 = scaledY,
                X2 = scaledX + 15,
                Y2 = scaledY,
                Stroke = Brushes.Lime,
                StrokeThickness = 1
            };

            var crossV = new Line
            {
                X1 = scaledX,
                Y1 = scaledY - 15,
                X2 = scaledX,
                Y2 = scaledY + 15,
                Stroke = Brushes.Lime,
                StrokeThickness = 1
            };

            VideoCanvas.Children.Add(crossH);
            VideoCanvas.Children.Add(crossV);

            var signalBox = new TextBlock
            {
                Text = $"ОБЪЕКТ\n{(_camera.DetectionConfidence * 100):F0}%",
                Foreground = Brushes.Lime,
                FontSize = 9,
                Background = new SolidColorBrush(Color.FromArgb(200, 0, 0, 0)),
                Padding = new Thickness(4, 2, 4, 2)
            };

            Canvas.SetLeft(signalBox, scaledX + boxSize / 2 + 5);
            Canvas.SetTop(signalBox, scaledY - boxSize / 2);
            VideoCanvas.Children.Add(signalBox);
        }

        private void CloseBtn_Click(object sender, RoutedEventArgs e)
        {
            _camera.Deactivate();
            _updateTimer?.Stop();
            _updateTimer?.Dispose();
            Close();
        }

        protected override void OnClosed(EventArgs e)
        {
            _camera.Deactivate();
            _updateTimer?.Stop();
            _updateTimer?.Dispose();
            base.OnClosed(e);
        }
        public void SetIntruderPosition(Point position)
        {
            _lastIntruderPosition = position;
        }

        private double GetDistance(Point p1, Point p2)
        {
            return Math.Sqrt(Math.Pow(p2.X - p1.X, 2) + Math.Pow(p2.Y - p1.Y, 2));
        }
    }
}
