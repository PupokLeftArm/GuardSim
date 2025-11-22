using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.Windows.Input;

namespace GuardSim
{
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;
        private Layout _layout;
        private ClockGenerator _clockGenerator;
        private TrajectoryGenerator _trajectoryGenerator;
        private SensorDataSimulator _sensorSimulator;
        private Dictionary<string, CameraViewWindow> _cameraWindows = new Dictionary<string, CameraViewWindow>();
        private DateTime _simulationStartTime;
        private int _detectionLogCount = 0;
        private readonly Dictionary<string, DroneViewWindow> _droneWindows = new Dictionary<string, DroneViewWindow>();
        private MultiIntruderSimulator _multiIntruderSimulator;
        private bool _isMultiSimulationRunning = false;
        private SimulationResults _lastResults;

        private bool _isEditMode = false;
        private readonly Dictionary<UIElement, SensorBase> _markerToSensor = new Dictionary<UIElement, SensorBase>();
        private UIElement _draggedElement = null;
        private SensorBase _draggedSensor = null;
        private Point _dragStartMouse;
        private Point _dragStartSensorPos;
        private List<Point> _dragStartDroneRoute = null;
        private const double MarkerSize = 14.0;
        private FrameworkElement _dragWpEl = null;
        private DroneSensor _dragWpDrone = null;
        private int _dragWpIndex = -1;
        private Point _dragWpStartMouse;
        private Point _dragWpStartPoint;
        private int _radarSeq = 1, _camSeq = 1, _droneSeq = 1;
        private SensorBase _selectedSensor;
        private DroneSensor _selectedWpDrone = null;
        private int _selectedWpIndex = -1;

        public SensorDataSimulator SensorSimulator
        {
            get => _sensorSimulator;
            private set
            {
                if (_sensorSimulator != value)
                {
                    _sensorSimulator = value;
                    OnPropertyChanged(nameof(SensorSimulator));
                }
            }
        }

        public MainWindow()
        {
            InitializeComponent();
            Loaded += OnLoaded;
            _clockGenerator = new ClockGenerator(100); // 100ms = 10 FPS
            _clockGenerator.Tick += OnClockTick;
            DataContext = this;
            this.Loaded += (s, e) =>
            {
                var sensorPanel = this.FindName("SensorPanel") as SensorPanelControl;
                if (sensorPanel != null)
                {
                    sensorPanel.DroneViewRequested += SensorPanel_DroneViewRequested;
                    sensorPanel.CameraViewRequested += SensorPanel_CameraViewRequested;
                }
            };
        }

        private void SensorPanel_CameraViewRequested(object sender, CameraViewEventArgs e)
        {
            if (_sensorSimulator == null || string.IsNullOrEmpty(e.CameraId)) return;

            var cam = _sensorSimulator.Cameras.FirstOrDefault(c => c.SensorId == e.CameraId);
            if (cam == null) return;

            if (_cameraWindows.TryGetValue(e.CameraId, out var opened))
            {
                opened.Close();
                _cameraWindows.Remove(e.CameraId);
                return;
            }

            var wnd = new CameraViewWindow(cam) { Owner = this };
            wnd.Closed += (_, __) => _cameraWindows.Remove(e.CameraId);
            _cameraWindows[e.CameraId] = wnd;
            wnd.Show();
        }



        private List<Point> ToPoints(List<double[]> points)
        {
            var pointList = new List<Point>();
            foreach (var p in points)
            {
                if (p.Length >= 2)
                {
                    pointList.Add(new Point(p[0], p[1]));
                }
            }
            return pointList;
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            Overlay.Background = Brushes.Transparent;
            Overlay.IsHitTestVisible = true;
            MapImage.IsHitTestVisible = false;
            Panel.SetZIndex(Overlay, 100);

            string layoutPath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "Maps", "map.layout.json");
            if (!File.Exists(layoutPath))
            {
                MessageBox.Show("Нет файла разметки: Assets/Maps/map.layout.json");
                return;
            }

            try
            {
                _layout = JsonConvert.DeserializeObject<Layout>(File.ReadAllText(layoutPath));
                if (_layout.Perimeter != null)
                {
                    if (string.IsNullOrWhiteSpace(_layout.Perimeter.Fill)) _layout.Perimeter.Fill = "#20FFFF00";
                    if (string.IsNullOrWhiteSpace(_layout.Perimeter.Stroke)) _layout.Perimeter.Stroke = "#FFFFC107";
                    if (_layout.Perimeter.StrokeThickness <= 0) _layout.Perimeter.StrokeThickness = 2.0;
                }

                if (_layout.Zones != null)
                {
                    foreach (var z in _layout.Zones)
                    {
                        if (string.IsNullOrWhiteSpace(z.Fill)) z.Fill = "#664296F3";
                        if (string.IsNullOrWhiteSpace(z.Stroke)) z.Stroke = "#FF4296F3";
                        if (z.StrokeThickness <= 0) z.StrokeThickness = 1.6;
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Ошибка JSON: " + ex.Message);
                return;
            }

            if (_layout == null)
            {
                MessageBox.Show("Пустой layout");
                return;
            }

            string imagePath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, _layout.Image);
            if (!File.Exists(imagePath))
            {
                MessageBox.Show("Нет картинки: " + _layout.Image);
                return;
            }

            BitmapImage bmp = new BitmapImage(new Uri(imagePath));
            MapImage.Source = bmp;
            Scene.Width = bmp.PixelWidth;
            Scene.Height = bmp.PixelHeight;

            _trajectoryGenerator = new TrajectoryGenerator(
                ToPoints(_layout.Perimeter.Points),
                _layout.Zones,
                5, 9,
                this);

            InitializeSensorSystem();
            Redraw();
        }

        private void InitializeSensorSystem()
        {
            _sensorSimulator = new SensorDataSimulator();

            string configPath = System.IO.Path.Combine(
                AppDomain.CurrentDomain.BaseDirectory,
                "Assets",
                "Config",
                "sensors_config.json");

            try
            {
                if (!System.IO.File.Exists(configPath))
                {
                    MessageBox.Show($"⚠️ Конфиг файл не найден: {configPath}\n\n" +
                                  "Используются параметры по умолчанию.",
                                  "Внимание", MessageBoxButton.OK, MessageBoxImage.Warning);
                    LoadDefaultSensors();
                    return;
                }

                string jsonContent = System.IO.File.ReadAllText(configPath);
                dynamic config = JsonConvert.DeserializeObject(jsonContent);

                if (config["Radars"] != null)
                {
                    foreach (var radar in config["Radars"])
                    {
                        string sensorId = radar["SensorId"]?.ToString() ?? "RADAR_UNKNOWN";
                        string sensorName = radar["SensorName"]?.ToString() ?? "Неизвестный радар";
                        double posX = radar["PositionX"] ?? 0.0;
                        double posY = radar["PositionY"] ?? 0.0;
                        double range = radar["DetectionRange"] ?? 200.0;
                        double noise = radar["NoiseLevel"] ?? 2.0;

                        _sensorSimulator.AddRadar(sensorId, sensorName, new Point(posX, posY), range, noise);
                        System.Diagnostics.Debug.WriteLine($"✅ Загружен: {sensorName}");
                    }
                }

                if (config["Cameras"] != null)
                {
                    foreach (var camera in config["Cameras"])
                    {
                        string sensorId = camera["SensorId"]?.ToString() ?? "CAMERA_UNKNOWN";
                        string sensorName = camera["SensorName"]?.ToString() ?? "Неизвестная камера";
                        double posX = camera["PositionX"] ?? 0.0;
                        double posY = camera["PositionY"] ?? 0.0;
                        double range = camera["DetectionRange"] ?? 100.0;
                        double fov = camera["FieldOfView"] ?? 90.0;
                        double angle = camera["ViewAngle"] != null ? (double)camera["ViewAngle"] : 0.0;
                        double noise = camera["NoiseLevel"] ?? 3.0;

                        _sensorSimulator.AddCamera(sensorId, sensorName, new Point(posX, posY), range, fov, angle, noise);
                        System.Diagnostics.Debug.WriteLine($"✅ Загружена: {sensorName}");
                    }
                }
                if (config["Drones"] != null)
                {
                    foreach (var drone in config["Drones"])
                    {
                        string sensorId = drone["SensorId"]?.ToString() ?? "DRONE_UNKNOWN";
                        string sensorName = drone["SensorName"]?.ToString() ?? "Неизвестный БПЛА";
                        double scanRadius = drone["ScanRadius"] ?? 300.0;
                        double noiseLevel = drone["NoiseLevel"] ?? 2.5;

                        var patrolRoute = new List<Point>();
                        if (drone["PatrolRoute"] != null)
                        {
                            foreach (var point in drone["PatrolRoute"])
                            {
                                double x = point["X"] ?? 0.0;
                                double y = point["Y"] ?? 0.0;
                                patrolRoute.Add(new Point(x, y));
                            }
                        }

                        if (patrolRoute.Count == 0)
                        {
                            patrolRoute = GenerateDronePatrolRoute();
                        }

                        _sensorSimulator.AddDrone(sensorId, sensorName, patrolRoute, scanRadius, noiseLevel);
                        System.Diagnostics.Debug.WriteLine($"✅ Загружен: {sensorName}");
                    }
                }

                MessageBox.Show($"✅ Конфиг успешно загружен!\n\n" +
                              $"Радаров: {_sensorSimulator.Radars.Count}\n" +
                              $"Камер: {_sensorSimulator.Cameras.Count}\n" +
                              $"БПЛА: {_sensorSimulator.Drones.Count}",
                              "Информация", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"❌ Ошибка при загрузке конфига:\n{ex.Message}\n\n" +
                              "Используются параметры по умолчанию.",
                              "Ошибка", MessageBoxButton.OK, MessageBoxImage.Error);
                LoadDefaultSensors();
            }

            OnPropertyChanged(nameof(SensorSimulator));

            foreach (var sensor in _sensorSimulator.GetAllSensors())
            {
                _sensorSimulator.UpdateSensorStatus(sensor, false);
            }
        }

        private void LoadDefaultSensors()
        {
            _sensorSimulator.AddRadar("RADAR_1", "Радар #1 (СЗ)", new Point(300, 450), 200, 2.0);
            _sensorSimulator.AddRadar("RADAR_2", "Радар #2 (СВ)", new Point(Scene.Width - 390, 450), 250, 2.0);
            _sensorSimulator.AddRadar("RADAR_3", "Радар #3 (ЮВ)", new Point(700, Scene.Height - 200), 220, 2.5);
            _sensorSimulator.AddRadar("RADAR_4", "Радар #4 (С)", new Point(750, 300), 180, 3.0);

            _sensorSimulator.AddCamera("CAMERA_1", "Камера #1 (Гаражи)", new Point(400, 450), 100, 270, 3.0);
            _sensorSimulator.AddCamera("CAMERA_2", "Камера #2 (АКПП 1)", new Point(Scene.Width - 305, Scene.Height - 205), 80, 90, 3.0);
            _sensorSimulator.AddCamera("CAMERA_3", "Камера #3 (АКПП 2)", new Point(Scene.Width - 465, Scene.Height - 205), 80, 80, 4.2);
            _sensorSimulator.AddCamera("CAMERA_4", "Камера #4 (АКПП 4)", new Point(Scene.Width - 662, Scene.Height - 190), 90, 100, 2.0);
            _sensorSimulator.AddCamera("CAMERA_5", "Камера #5 (АКПП 5)", new Point(555, Scene.Height - 190), 90, 120, 2.0);
            _sensorSimulator.AddCamera("CAMERA_6", "Камера #6 (АКПП 6)", new Point(280, 755), 90, 180, 2.0);

            var droneRoute = GenerateDronePatrolRoute();
            _sensorSimulator.AddDrone("DRONE_1", "БПЛА-Орлан-10", droneRoute, 300, 2.5);

            System.Diagnostics.Debug.WriteLine("⚠️ Использованы параметры по умолчанию");
        }

        private List<Point> GenerateDronePatrolRoute()
        {
            var route = new List<Point>();

            double margin = 150;
            route.Add(new Point(margin, margin));
            route.Add(new Point(Scene.Width - margin, margin));
            route.Add(new Point(Scene.Width - margin, Scene.Height - margin));
            route.Add(new Point(margin, Scene.Height - margin));

            return route;
        }

        private void Redraw()
        {
            Overlay.Children.Clear();
            if (_layout == null) return;

            if (_layout.Perimeter != null && _layout.Perimeter.Points != null && _layout.Perimeter.Points.Count >= 3)
            {
                Polygon p = MakePolygon(_layout.Perimeter.Points, _layout.Perimeter.Fill, _layout.Perimeter.Stroke, _layout.Perimeter.StrokeThickness);
                Overlay.Children.Add(p);
                AddLabel(p, "Периметр", Brushes.Khaki);
            }

            if (_layout.Zones != null)
            {
                foreach (var z in _layout.Zones)
                {
                    if (z.Points == null || z.Points.Count < 3) continue;
                    Polygon p = MakePolygon(z.Points, z.Fill, z.Stroke, z.StrokeThickness);
                    Overlay.Children.Add(p);
                }
            }

            DrawSensorCoverage();

            var trajectoryCopy = _trajectoryGenerator.Trajectory.ToList();
            if (trajectoryCopy.Count > 1)
            {
                Polyline polyline = new Polyline
                {
                    Stroke = Brushes.Red,
                    StrokeThickness = 2,
                    Points = new PointCollection(trajectoryCopy),
                    Opacity = 0.7
                };
                Overlay.Children.Add(polyline);
            }

            if (trajectoryCopy.Count > 0)
            {
                var currentPos = trajectoryCopy[trajectoryCopy.Count - 1];
                Ellipse currentEllipse = new Ellipse
                {
                    Width = 16,
                    Height = 16,
                    Fill = Brushes.Yellow,
                    Stroke = Brushes.Red,
                    StrokeThickness = 3
                };

                Canvas.SetLeft(currentEllipse, currentPos.X - 8);
                Canvas.SetTop(currentEllipse, currentPos.Y - 8);
                Overlay.Children.Add(currentEllipse);

                TextBlock speedInfo = new TextBlock
                {
                    Text = $"Скорость: {_trajectoryGenerator.CurrentSpeed:F1}",
                    Foreground = Brushes.White,
                    Background = Brushes.Black,
                    FontSize = 10,
                    Padding = new Thickness(4)
                };
                Canvas.SetLeft(speedInfo, currentPos.X + 10);
                Canvas.SetTop(speedInfo, currentPos.Y - 20);
                Overlay.Children.Add(speedInfo);
            }

            DrawSensorPositions();
            UpdateStatistics();
        }

        private void DrawSensorCoverage()
        {
            if (_sensorSimulator == null) return;

            if (_layout.Perimeter != null)
            {
                var perimeterPolygon = MakePolygon(_layout.Perimeter.Points, _layout.Perimeter.Fill, _layout.Perimeter.Stroke, _layout.Perimeter.StrokeThickness);
                Overlay.Children.Add(perimeterPolygon);
            }

            if (_layout.Zones != null)
            {
                foreach (var zone in _layout.Zones)
                {
                    var zonePolygon = MakePolygon(zone.Points, zone.Fill, zone.Stroke, zone.StrokeThickness);
                    Overlay.Children.Add(zonePolygon);
                }
            }

            foreach (var radar in _sensorSimulator.Radars)
            {
                Ellipse coverage = new Ellipse
                {
                    Width = radar.DetectionRange * 2,
                    Height = radar.DetectionRange * 2,
                    Stroke = Brushes.DarkRed,
                    StrokeThickness = 2,
                    Fill = null,
                    Opacity = 0.3,
                    StrokeDashArray = new DoubleCollection { 5, 5 }
                };

                Canvas.SetLeft(coverage, radar.Position.X - radar.DetectionRange);
                Canvas.SetTop(coverage, radar.Position.Y - radar.DetectionRange);
                Overlay.Children.Add(coverage);

                double mid = radar.CurrentScanAngle;
                double half = radar.BeamWidth / 2.0;
                double start = mid - half;
                double end = mid + half;

                var sectorFill = new SolidColorBrush(Color.FromArgb(100, 0, 0, 255));
                var sectorStroke = new SolidColorBrush(Color.FromArgb(255, 255, 0, 0));

                var beamSector = MakeSector(
                    radar.Position,
                    radar.DetectionRange,
                    start, end,
                    sectorFill,
                    sectorStroke,
                    strokeThickness: 1.0,
                    opacity: 0.55);

                Overlay.Children.Add(beamSector);

                double midRad = mid * Math.PI / 180.0;
                Point dirEnd = new Point(
                    radar.Position.X + radar.DetectionRange * Math.Cos(midRad),
                    radar.Position.Y + radar.DetectionRange * Math.Sin(midRad));

                var dirLine = new Line
                {
                    X1 = radar.Position.X,
                    Y1 = radar.Position.Y,
                    X2 = dirEnd.X,
                    Y2 = dirEnd.Y,
                    Stroke = Brushes.GreenYellow,
                    StrokeThickness = 2,
                    Opacity = 0.9
                };
                Overlay.Children.Add(dirLine);
            }

            foreach (var drone in _sensorSimulator.Drones)
            {
                double half = drone.FieldOfView / 2.0;
                double start = drone.ViewAngle - half;
                double end = drone.ViewAngle + half;

                var sectorFill = new SolidColorBrush(Color.FromArgb(70, 15, 56, 240));
                var sectorStroke = new SolidColorBrush(Color.FromArgb(160, 15, 56, 240));

                var fovSector = MakeSector(
                    drone.Position,
                    drone.ScanRadius,
                    start, end,
                    sectorFill,
                    sectorStroke,
                    strokeThickness: 1.0,
                    opacity: 0.55);

                Overlay.Children.Add(fovSector);
            }
            string radarMarkerPath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "Icons", "radar.png");
            foreach (var radar in _sensorSimulator.Radars)
            {
                var radarMarker = new Image
                {
                    Width = 20,
                    Height = 20,
                    Source = new BitmapImage(new Uri(radarMarkerPath))
                };
                Canvas.SetLeft(radarMarker, radar.Position.X - 8);
                Canvas.SetTop(radarMarker, radar.Position.Y - 8);
                Overlay.Children.Add(radarMarker);
            }

            foreach (var camera in _sensorSimulator.Cameras)
            {
                var cameraMarker = new Ellipse
                {
                    Width = 8,
                    Height = 8,
                    Fill = Brushes.Magenta,
                    Stroke = Brushes.White,
                    StrokeThickness = 2
                };
                Canvas.SetLeft(cameraMarker, camera.Position.X - 4);
                Canvas.SetTop(cameraMarker, camera.Position.Y - 4);
                Overlay.Children.Add(cameraMarker);
            }
            string droneMarkerPath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "Icons", "drone.png");
            foreach (var drone in _sensorSimulator.Drones)
            {
                var droneMarker = new Image
                {
                    Width = 20,
                    Height = 20,
                    Source = new BitmapImage(new Uri(droneMarkerPath))
                };
                Canvas.SetLeft(droneMarker, drone.Position.X - 4);
                Canvas.SetTop(droneMarker, drone.Position.Y - 10);
                Overlay.Children.Add(droneMarker);
            }
        }
        private void DrawSensorPositions()
        {
            if (_sensorSimulator == null) return;

            foreach (var radar in _sensorSimulator.Radars)
            {
                Ellipse coverage = new Ellipse
                {
                    Width = radar.DetectionRange * 2,
                    Height = radar.DetectionRange * 2,
                    Stroke = Brushes.Cyan,
                    StrokeThickness = 1,
                    Fill = new SolidColorBrush(Color.FromArgb(50, 0, 255, 255)),
                    Opacity = 0.3,
                    StrokeDashArray = new DoubleCollection { 5, 5 }
                };

                Canvas.SetLeft(coverage, radar.Position.X - radar.DetectionRange);
                Canvas.SetTop(coverage, radar.Position.Y - radar.DetectionRange);
                Overlay.Children.Add(coverage);
            }

            foreach (var camera in _sensorSimulator.Cameras)
            {
                double halfFov = camera.FieldOfView / 2.0;
                double start = camera.ViewAngle - halfFov;
                double end = camera.ViewAngle + halfFov;
                var fovFill = new SolidColorBrush(Color.FromArgb(90, 20, 120, 240));
                var fovStroke = new SolidColorBrush(Color.FromArgb(140, 135, 206, 235));

                var fovSector = MakeSector(
                    camera.Position,
                    camera.DetectionRange,
                    start, end,
                    fovFill,
                    fovStroke,
                    strokeThickness: 1.0,
                    opacity: 0.55);

                Overlay.Children.Add(fovSector);

                var cameraMarker = new Rectangle
                {
                    Width = 12,
                    Height = 12,
                    Fill = Brushes.Magenta,
                    Stroke = Brushes.White,
                    StrokeThickness = 2
                };

                Canvas.SetLeft(cameraMarker, camera.Position.X - 6);
                Canvas.SetTop(cameraMarker, camera.Position.Y - 6);
                Overlay.Children.Add(cameraMarker);
            }
            string droneMarkerPath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "Icons", "drone.png");
            foreach (var drone in _sensorSimulator.Drones)
            {
                var nextWaypoint = drone.PatrolRoute[(drone.CurrentWaypoint + 1) % drone.PatrolRoute.Count];
                double dx = nextWaypoint.X - drone.Position.X;
                double dy = nextWaypoint.Y - drone.Position.Y;
                double angle = Math.Atan2(dy, dx);
            }
        }
        private void UpdateStatistics()
        {
            var detections = _sensorSimulator?.GetActiveDetections() ?? new List<SensorDetection>();
            ActiveRadarsText.Text = _sensorSimulator?.Radars.Count.ToString() ?? "0";
            DetectionCountText.Text = detections.Count.ToString();
            IntruderStatusText.Text = _trajectoryGenerator.hasReachedPerimeter ? "⚠️ ДОСТИГ ПЕРИМЕТРА" : "Активен";
        }

        private Polygon MakePolygon(List<double[]> pts, string fillHex, string strokeHex, double strokeThickness)
        {
            Polygon poly = new Polygon();
            poly.Fill = ToBrushSafe(fillHex, "#400000FF");
            poly.Stroke = ToBrushSafe(strokeHex, "#FF0000FF");
            poly.StrokeThickness = strokeThickness;

            PointCollection pc = new PointCollection();
            foreach (var a in pts)
            {
                if (a != null && a.Length >= 2)
                    pc.Add(new Point(a[0], a[1]));
            }

            poly.Points = pc;
            return poly;
        }

        private static bool PointInPolygon(System.Windows.Point p, System.Collections.Generic.List<double[]> poly)
        {
            int n = poly?.Count ?? 0;
            bool inside = false;
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                double xi = poly[i][0], yi = poly[i][1];
                double xj = poly[j][0], yj = poly[j][1];

                bool intersect = ((yi > p.Y) != (yj > p.Y)) &&
                                 (p.X < (xj - xi) * (p.Y - yi) / (yj - yi + 1e-12) + xi);
                if (intersect) inside = !inside;
            }
            return inside;
        }

        private void AddLabel(Polygon poly, string text, Brush color)
        {
            if (poly.Points == null || poly.Points.Count == 0) return;

            double sx = 0, sy = 0;
            foreach (Point p in poly.Points) { sx += p.X; sy += p.Y; }
            double cx = sx / poly.Points.Count;
            double cy = sy / poly.Points.Count;

            TextBlock tb = new TextBlock();
            tb.Text = text;
            tb.Foreground = color;
            tb.FontSize = 12;
            tb.Background = ToBrushSafe("#66000000", "#66000000");
            tb.Padding = new Thickness(4, 1, 4, 1);

            Canvas.SetLeft(tb, cx + 6);
            Canvas.SetTop(tb, cy + 6);
            Overlay.Children.Add(tb);
        }

        private static Brush ToBrushSafe(string hexOrName, string fallbackHex)
        {
            if (string.IsNullOrWhiteSpace(hexOrName))
                return new SolidColorBrush((Color)ColorConverter.ConvertFromString(fallbackHex));

            string s = hexOrName.Trim();
            try
            {
                object obj = ColorConverter.ConvertFromString(s);
                if (obj is Color c1) return new SolidColorBrush(c1);
            }
            catch { }

            try
            {
                if (s[0] == '#') s = s.Substring(1);
                byte a = 255, r, g, b;

                if (s.Length == 3)
                {
                    r = (byte)Convert.ToInt32(new string(s[0], 2), 16);
                    g = (byte)Convert.ToInt32(new string(s[1], 2), 16);
                    b = (byte)Convert.ToInt32(new string(s[2], 2), 16);
                }
                else if (s.Length == 4)
                {
                    a = (byte)Convert.ToInt32(new string(s[0], 2), 16);
                    r = (byte)Convert.ToInt32(new string(s[1], 2), 16);
                    g = (byte)Convert.ToInt32(new string(s[2], 2), 16);
                    b = (byte)Convert.ToInt32(new string(s[3], 2), 16);
                }
                else if (s.Length == 6)
                {
                    r = byte.Parse(s.Substring(0, 2), System.Globalization.NumberStyles.HexNumber);
                    g = byte.Parse(s.Substring(2, 2), System.Globalization.NumberStyles.HexNumber);
                    b = byte.Parse(s.Substring(4, 2), System.Globalization.NumberStyles.HexNumber);
                }
                else if (s.Length == 8)
                {
                    a = byte.Parse(s.Substring(0, 2), System.Globalization.NumberStyles.HexNumber);
                    r = byte.Parse(s.Substring(2, 2), System.Globalization.NumberStyles.HexNumber);
                    g = byte.Parse(s.Substring(4, 2), System.Globalization.NumberStyles.HexNumber);
                    b = byte.Parse(s.Substring(6, 2), System.Globalization.NumberStyles.HexNumber);
                }
                else
                {
                    return new SolidColorBrush((Color)ColorConverter.ConvertFromString(fallbackHex));
                }

                return new SolidColorBrush(Color.FromArgb(a, r, g, b));
            }
            catch
            {
                return new SolidColorBrush((Color)ColorConverter.ConvertFromString(fallbackHex));
            }
        }

        private async void BtnStart_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                _sensorSimulator?.ResetAllStatuses();
                _sensorSimulator?.ClearLogs();

                if (_trajectoryGenerator != null)
                {
                    if (_trajectoryGenerator.Trajectory != null)
                        _trajectoryGenerator.Trajectory.Clear();

                    _trajectoryGenerator.hasReachedPerimeter = false;
                    _trajectoryGenerator.SetRandomPosition(0, Scene.Width, 0, Scene.Height);
                }

                _simulationStartTime = DateTime.Now;
                UpdateTimerDisplay();

                BtnStart.IsEnabled = false;
                BtnStop.IsEnabled = true;

                _clockGenerator.Start();
            }
            catch (Exception ex)
            {
                MessageBox.Show("Ошибка запуска обычной симуляции: " + ex.Message);
                BtnStart.IsEnabled = true;
                BtnStop.IsEnabled = false;
            }
        }

        double ZoneWeightFunc(Point p)
        {
            if (_layout != null && _layout.Zones != null)
            {
                foreach (var z in _layout.Zones)
                {
                    if (PointInPolygon(p, z.Points)) return Math.Max(1.0, z.Weight);
                }
            }
            return 1.0;
        }

        private void BtnEditMode_Click(object sender, RoutedEventArgs e)
        {
            _isEditMode = (BtnEditMode.IsChecked == true);
            if (_isEditMode)
            {
                _clockGenerator.Stop();
                BtnStart.IsEnabled = false;
                BtnStop.IsEnabled = false;
                BtnRunMultiSimulation.IsEnabled = false;
                EnterEditMode();
            }
            else
            {
                ExitEditMode();
                BtnStart.IsEnabled = true;
                BtnStop.IsEnabled = false;
                BtnRunMultiSimulation.IsEnabled = true;
                Redraw();
            }
        }

        private void BtnSaveConfig_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                string cfgPath = System.IO.Path.Combine(
                    AppDomain.CurrentDomain.BaseDirectory,
                    "Assets", "Config", "sensors_config.json");

                int intruders = (_lastResults != null) ? _lastResults.TotalIntruders : 0;
                SensorsConfigIO.Save(cfgPath, _sensorSimulator, intruders);
                MessageBox.Show("Конфигурация сохранена:\n" + cfgPath, "Сохранено",
                    MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Ошибка сохранения конфигурации:\n" + ex.Message,
                    "Ошибка", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private async void BtnRunMultiSimulation_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                BtnRunMultiSimulation.IsEnabled = false;
                BtnStart.IsEnabled = false;
                BtnStop.IsEnabled = false;

                IDetectionProvider detector = null;
                if (_sensorSimulator != null)
                    detector = new DeterministicDetectionAdapter(_sensorSimulator)
                    {
                        RadarSweepDegPerSec = 60.0,
                        RadarPd = 0.95,
                        CameraPd = 0.90,
                        DronePd = 0.92
                    };

                IAvoidanceField avoidField = null;
                if (_sensorSimulator != null)
                {
                    var field = new SensorAvoidanceField(_sensorSimulator);
                    field.RadarWeight = 1.0;
                    field.CameraWeight = 1.2;
                    avoidField = field;
                }

                var perimeterPts = ToPoints(_layout.Perimeter.Points);

                var sim = new MultiIntruderSimulator(perimeterPts, ZoneWeightFunc, detector, Scene.Width, Scene.Height, avoidField);
                sim.Spawn = SpawnMode.PerimeterRing;
                sim.Approach = ApproachMode.SmartAvoidSensors;
                sim.DtSeconds = 0.5;
                sim.MinSpeed = 5;
                sim.MaxSpeed = 10;
                sim.MaxStepsPerIntruder = 5000;
                sim.GoalWeight = 1.0;
                sim.AvoidWeight = 1.3;
                sim.NoiseWeight = 0.12;
                sim.SmartTargetSamples = 48;

                int intruders_conf = 0;
                string configPath = System.IO.Path.Combine(
                AppDomain.CurrentDomain.BaseDirectory,
                "Assets",
                "Config",
                "sensors_config.json");
                try
                {
                    if (!System.IO.File.Exists(configPath))
                    {
                        MessageBox.Show($"⚠️ Конфиг файл не найден: {configPath}\n\n" +
                                      "Используются параметры по умолчанию.",
                                      "Внимание", MessageBoxButton.OK, MessageBoxImage.Warning);
                        LoadDefaultSensors();
                        return;
                    }

                    string jsonContent = System.IO.File.ReadAllText(configPath);
                    dynamic config = JsonConvert.DeserializeObject(jsonContent);

                    if (config["Intruders"] != null)
                    {
                        intruders_conf = (int)config["Intruders"]?.Value;
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"❌ Ошибка при загрузке конфига:\n{ex.Message}\n\n" +
                                  "Используются параметры по умолчанию.",
                                  "Ошибка", MessageBoxButton.OK, MessageBoxImage.Error);
                    LoadDefaultSensors();
                }
                int intruders = 0;
                if (intruders_conf == 0)
                {
                    intruders = 2000;
                }
                else
                {
                    intruders = intruders_conf;
                }
                    var results = await System.Threading.Tasks.Task.Run<SimulationResults>(() => sim.Run(intruders));
                _lastResults = results;

                DrawVulnerableSegments(results);
                DrawSegmentCounts(results);

                string exportDir = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "ExportedData");
                var bundle = DetectionDataExporter.ExportAll(exportDir, results, "simulation");

                var most = results.MostVulnerableSegment;
                string msg =
                    "Всего нарушителей: " + results.TotalIntruders + "\n" +
                    "Перехвачено сенсорами: " + results.Intercepted + "\n" +
                    "Проникло через периметр: " + results.Breached + "\n" +
                    "Самый уязвимый участок: " + (most == null ? "—" :
                        ("#" + most.Index + " (" + most.Start.X.ToString("0") + "," + most.Start.Y.ToString("0") + ")–(" +
                         most.End.X.ToString("0") + "," + most.End.Y.ToString("0") + "), прорывов: " + most.Breaches +
                         " (" + (most.BreachProbability * 100).ToString("0.00") + "%)")) + "\n\n" +
                    "JSON: " + bundle.JsonPath + "\nCSV: " + bundle.CsvPath;

                MessageBox.Show(msg, "Результаты симуляции", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Ошибка симуляции: " + ex.Message);
            }
            finally
            {
                BtnRunMultiSimulation.IsEnabled = true;
                BtnStart.IsEnabled = true;
                BtnStop.IsEnabled = true;
            }
        }

        private void DrawSegmentCounts(SimulationResults res)
        {
            if (res == null || res.Segments == null) return;

            foreach (var seg in res.Segments)
            {
                if (seg.Breaches <= 0) continue;

                double mx = (seg.Start.X + seg.End.X) * 0.5;
                double my = (seg.Start.Y + seg.End.Y) * 0.5;
                string probabilityText = (seg.BreachProbability * 100).ToString("0.00") + "%";

                var tb = new TextBlock
                {
                    Text = probabilityText,
                    Foreground = Brushes.White,
                    FontSize = 12,
                    FontWeight = FontWeights.SemiBold,
                    Background = new SolidColorBrush(Color.FromArgb(160, 220, 38, 38)),
                    Padding = new Thickness(4, 1, 4, 1)
                };

                Canvas.SetLeft(tb, mx + 6);
                Canvas.SetTop(tb, my + 6);
                Overlay.Children.Add(tb);
            }
        }

        private void BtnStop_Click(object sender, RoutedEventArgs e)
        {
            _clockGenerator.Stop();
            BtnStart.IsEnabled = true;
            BtnStop.IsEnabled = false;
        }

        private void DrawVulnerableSegments(SimulationResults res)
        {
            Dispatcher.Invoke(Redraw);

            if (res == null || res.Segments == null || res.Segments.Count == 0) return;

            int max = res.MaxSegmentBreaches;
            if (max <= 0) return;

            foreach (var seg in res.Segments)
            {
                if (seg.Breaches <= 0) continue;

                double ratio = (double)seg.Breaches / (double)max;

                byte g = (byte)(255 - (int)(255 * ratio));
                Color core = Color.FromArgb(255, 255, g, 0);
                Color halo = Color.FromArgb(120, 255, g, 0);

                var under = new Line();
                under.X1 = seg.Start.X; under.Y1 = seg.Start.Y;
                under.X2 = seg.End.X; under.Y2 = seg.End.Y;
                under.Stroke = new SolidColorBrush(halo);
                under.StrokeThickness = 8.0 + 10.0 * ratio;
                under.StrokeStartLineCap = PenLineCap.Round;
                under.StrokeEndLineCap = PenLineCap.Round;
                under.StrokeLineJoin = PenLineJoin.Round;
                under.SnapsToDevicePixels = true;
                Overlay.Children.Add(under);

                var top = new Line();
                top.X1 = seg.Start.X; top.Y1 = seg.Start.Y;
                top.X2 = seg.End.X; top.Y2 = seg.End.Y;
                top.Stroke = new SolidColorBrush(core);
                top.StrokeThickness = 2.0 + 5.0 * ratio;
                top.StrokeStartLineCap = PenLineCap.Round;
                top.StrokeEndLineCap = PenLineCap.Round;
                top.StrokeLineJoin = PenLineJoin.Round;
                top.SnapsToDevicePixels = true;
                Overlay.Children.Add(top);
            }

            if (res.BreachPoints != null && res.BreachPoints.Count > 0)
            {
                for (int i = 0; i < res.BreachPoints.Count; i++)
                {
                    var bp = res.BreachPoints[i];
                    double r = 3.5;

                    var dot = new Ellipse();
                    dot.Width = r * 2; dot.Height = r * 2;
                    dot.Fill = new SolidColorBrush(Color.FromArgb(240, 255, 64, 64));
                    dot.Stroke = Brushes.White;
                    dot.StrokeThickness = 1.2;

                    Canvas.SetLeft(dot, bp.X - r);
                    Canvas.SetTop(dot, bp.Y - r);
                    Overlay.Children.Add(dot);
                }
            }
        }

        private ContextMenu BuildSensorContextMenu(SensorBase s)
        {
            var cm = new ContextMenu();

            var miDelete = new MenuItem { Header = "Удалить" };
            miDelete.Click += delegate
            {
                if (s is RadarSensor) _sensorSimulator.RemoveRadar(s.SensorId);
                else if (s is VideoCameraSensor) _sensorSimulator.RemoveCamera(s.SensorId);
                else if (s is DroneSensor) _sensorSimulator.RemoveDrone(s.SensorId);

                BuildEditorOverlay();
            };
            cm.Items.Add(miDelete);

            return cm;
        }

        private void AddRadarAt(Point p)
        {
            string id = "RADAR_ED_" + (_radarSeq++);
            string name = "Радар (ред.) " + _radarSeq;
            _sensorSimulator.AddRadar(id, name, p, 220, 2.0);
        }

        private void AddCameraAt(Point p)
        {
            string id = "CAM_ED_" + (_camSeq++);
            string name = "Камера (ред.) " + _camSeq;
            _sensorSimulator.AddCamera(id, name, p, 100, 0.0, 3.0);
        }

        private void AddDroneNear(Point center)
        {
            string id = "DRN_ED_" + (_droneSeq++);
            string name = "БПЛА (ред.) " + _droneSeq;

            double d = 120;
            var route = new List<Point>();
            route.Add(new Point(center.X - d, center.Y - d));
            route.Add(new Point(center.X + d, center.Y - d));
            route.Add(new Point(center.X + d, center.Y + d));
            route.Add(new Point(center.X - d, center.Y + d));

            _sensorSimulator.AddDrone(id, name, route, 250, 2.5);
        }

        private void Overlay_PreviewMouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (!_isEditMode) return;

            var src = e.OriginalSource as DependencyObject;
            for (FrameworkElement fe = src as FrameworkElement; fe != null; fe = VisualTreeHelper.GetParent(fe) as FrameworkElement)
            {
                if (fe == Overlay) break;

                if (fe is Ellipse && fe.Tag is DroneWpTag)
                    return;

                if (_markerToSensor.ContainsKey(fe))
                    return;

                var pl = fe as Polyline;
                if (pl != null && pl.Tag is DroneSensor)
                    return;
            }

            var pos = e.GetPosition(Overlay);
            var cm = new ContextMenu();

            var miAddRadar = new MenuItem { Header = "Добавить радар здесь" };
            miAddRadar.Click += delegate { AddRadarAt(pos); BuildEditorOverlay(); };
            cm.Items.Add(miAddRadar);

            var miAddCamera = new MenuItem { Header = "Добавить камеру здесь" };
            miAddCamera.Click += delegate { AddCameraAt(pos); BuildEditorOverlay(); };
            cm.Items.Add(miAddCamera);

            var miAddDrone = new MenuItem { Header = "Добавить БПЛА рядом" };
            miAddDrone.Click += delegate { AddDroneNear(pos); BuildEditorOverlay(); };
            cm.Items.Add(miAddDrone);

            cm.IsOpen = true;
            e.Handled = true;
        }

        private void HookMarkerEvents(FrameworkElement el)
        {
            el.MouseLeftButtonDown += Marker_MouseLeftButtonDown;
            el.MouseMove += Marker_MouseMove;
            el.MouseLeftButtonUp += Marker_MouseLeftButtonUp;
            el.MouseWheel += Marker_MouseWheel;
        }

        private sealed class DroneWpTag
        {
            public DroneSensor Drone;
            public int Index;
        }

        private void HookWaypointEvents(FrameworkElement el, DroneSensor drone, int index)
        {
            el.Tag = new DroneWpTag { Drone = drone, Index = index };
            el.MouseLeftButtonDown += Waypoint_MouseLeftButtonDown;
            el.MouseMove += Waypoint_MouseMove;
            el.MouseLeftButtonUp += Waypoint_MouseLeftButtonUp;
        }

        private void Marker_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (!_isEditMode) return;

            var fe = sender as FrameworkElement;
            if (fe == null) return;

            if (!_markerToSensor.ContainsKey(fe)) return;
            var sensor = _markerToSensor[fe];

            _selectedSensor = sensor;

            if (e.ClickCount == 2)
            {
                if (sensor is RadarSensor) _sensorSimulator.RemoveRadar(sensor.SensorId);
                else if (sensor is VideoCameraSensor) _sensorSimulator.RemoveCamera(sensor.SensorId);
                else if (sensor is DroneSensor) _sensorSimulator.RemoveDrone(sensor.SensorId);

                _selectedSensor = null;
                BuildEditorOverlay();
                e.Handled = true;
                return;
            }

            _draggedElement = fe;
            _draggedSensor = sensor;
            _dragStartMouse = e.GetPosition(Overlay);
            _dragStartSensorPos = _draggedSensor.Position;

            if (_draggedSensor is DroneSensor)
            {
                var d = (DroneSensor)_draggedSensor;
                _dragStartDroneRoute = new List<Point>();
                if (d.PatrolRoute != null)
                    for (int i = 0; i < d.PatrolRoute.Count; i++) _dragStartDroneRoute.Add(d.PatrolRoute[i]);
            }

            fe.CaptureMouse();
            e.Handled = true;
        }

        private void Marker_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (!_isEditMode) return;

            var fe = sender as FrameworkElement;
            if (fe == null) return;
            if (!_markerToSensor.ContainsKey(fe)) return;

            var sensor = _markerToSensor[fe];
            int ticks = e.Delta / 120;
            int sign = Math.Sign(ticks);
            if (sign == 0) return;

            bool fast = Keyboard.IsKeyDown(Key.LeftShift) || Keyboard.IsKeyDown(Key.RightShift);
            bool ctrl = Keyboard.IsKeyDown(Key.LeftCtrl) || Keyboard.IsKeyDown(Key.RightCtrl);

            if (sensor is VideoCameraSensor)
            {
                var c = (VideoCameraSensor)sensor;
                if (ctrl)
                {
                    double step = fast ? 10.0 : 3.0;
                    c.FieldOfView = Clamp(c.FieldOfView + sign * step, 10.0, 170.0);
                }
                else
                {
                    double step = fast ? 10.0 : 3.0;
                    c.ViewAngle = NormalizeAngle(c.ViewAngle + sign * step);
                }
                BuildEditorOverlay();
                e.Handled = true;
                return;
            }

            if (sensor is RadarSensor)
            {
                var r = (RadarSensor)sensor;
                double step = fast ? 20.0 : 8.0;
                r.DetectionRange = Clamp(r.DetectionRange + sign * step, 20.0, 2000.0);
                BuildEditorOverlay();
                e.Handled = true;
                return;
            }
        }

        private static double Clamp(double v, double lo, double hi)
        {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
        }

        private static double NormalizeAngle(double a)
        {
            while (a < 0) a += 360.0;
            while (a >= 360.0) a -= 360.0;
            return a;
        }


        private void Marker_MouseMove(object sender, MouseEventArgs e)
        {
            if (_draggedElement == null || _draggedSensor == null) return;
            if (e.LeftButton != MouseButtonState.Pressed) return;

            var cur = e.GetPosition(Overlay);
            var dx = cur.X - _dragStartMouse.X;
            var dy = cur.Y - _dragStartMouse.Y;

            if (_draggedSensor is DroneSensor)
            {
                var d = (DroneSensor)_draggedSensor;
                _draggedSensor.Position = new Point(_dragStartSensorPos.X + dx, _dragStartSensorPos.Y + dy);

                if (_dragStartDroneRoute != null && d.PatrolRoute != null && d.PatrolRoute.Count == _dragStartDroneRoute.Count)
                {
                    for (int i = 0; i < d.PatrolRoute.Count; i++)
                    {
                        var p0 = _dragStartDroneRoute[i];
                        d.PatrolRoute[i] = new Point(p0.X + dx, p0.Y + dy);
                    }
                }
            }
            else
            {
                _draggedSensor.Position = new Point(_dragStartSensorPos.X + dx, _dragStartSensorPos.Y + dy);
            }

            if (sender is FrameworkElement fe)
            {
                Canvas.SetLeft(fe, _draggedSensor.Position.X - fe.Width / 2.0);
                Canvas.SetTop(fe, _draggedSensor.Position.Y - fe.Height / 2.0);
            }
        }

        private void Marker_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (sender is UIElement fe) fe.ReleaseMouseCapture();
            _draggedElement = null;
            _draggedSensor = null;
            _dragStartDroneRoute = null;
            BuildEditorOverlay();
        }

        private void Waypoint_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (!_isEditMode) return;
            var fe = sender as FrameworkElement;
            if (fe == null) return;
            var tag = fe.Tag as DroneWpTag;
            if (tag == null || tag.Drone == null) return;
            _selectedWpDrone = tag.Drone;
            _selectedWpIndex = tag.Index;
            _dragWpEl = fe;
            _dragWpDrone = tag.Drone;
            _dragWpIndex = tag.Index;
            _dragWpStartMouse = e.GetPosition(Overlay);
            _dragWpStartPoint = _dragWpDrone.PatrolRoute[_dragWpIndex];
            fe.CaptureMouse();
            e.Handled = true;
        }


        private void Waypoint_MouseMove(object sender, MouseEventArgs e)
        {
            if (_dragWpEl == null || _dragWpDrone == null) return;
            if (e.LeftButton != MouseButtonState.Pressed) return;

            var cur = e.GetPosition(Overlay);
            var dx = cur.X - _dragWpStartMouse.X;
            var dy = cur.Y - _dragWpStartMouse.Y;

            var np = new Point(_dragWpStartPoint.X + dx, _dragWpStartPoint.Y + dy);
            _dragWpDrone.PatrolRoute[_dragWpIndex] = np;

            Canvas.SetLeft(_dragWpEl, np.X - 4);
            Canvas.SetTop(_dragWpEl, np.Y - 4);
        }

        private void Waypoint_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (sender is UIElement uie) uie.ReleaseMouseCapture();
            _dragWpEl = null;
            _dragWpDrone = null;
            _dragWpIndex = -1;
            BuildEditorOverlay();
        }


        private void BuildEditorOverlay()
        {
            Overlay.Children.Clear();

            if (_layout != null && _layout.Perimeter != null && _layout.Perimeter.Points != null)
            {
                var perPoly = MakePolygon(_layout.Perimeter.Points, _layout.Perimeter.Fill, _layout.Perimeter.Stroke, _layout.Perimeter.StrokeThickness);
                Overlay.Children.Add(perPoly);
                AddLabel(perPoly, "Периметр", Brushes.Khaki);
            }
            if (_layout != null && _layout.Zones != null)
            {
                foreach (var z in _layout.Zones)
                {
                    if (z.Points == null || z.Points.Count < 3) continue;
                    var zp = MakePolygon(z.Points, z.Fill, z.Stroke, z.StrokeThickness);
                    Overlay.Children.Add(zp);
                }
            }

            _markerToSensor.Clear();

            // РАДАРЫ
            for (int i = 0; i < _sensorSimulator.Radars.Count; i++)
            {
                var r = _sensorSimulator.Radars[i];

                var range = new Ellipse
                {
                    Width = r.DetectionRange * 2,
                    Height = r.DetectionRange * 2,
                    Stroke = Brushes.SteelBlue,
                    StrokeThickness = 1,
                    StrokeDashArray = new DoubleCollection { 5, 5 },
                    Opacity = 0.5
                };
                Canvas.SetLeft(range, r.Position.X - r.DetectionRange);
                Canvas.SetTop(range, r.Position.Y - r.DetectionRange);
                Overlay.Children.Add(range);

                var marker = new Ellipse
                {
                    Width = MarkerSize,
                    Height = MarkerSize,
                    Fill = Brushes.DeepSkyBlue,
                    Stroke = Brushes.White,
                    StrokeThickness = 2,
                    Cursor = Cursors.SizeAll,
                    ToolTip = r.SensorName + " (" + r.SensorId + ")"
                };
                Canvas.SetLeft(marker, r.Position.X - MarkerSize / 2);
                Canvas.SetTop(marker, r.Position.Y - MarkerSize / 2);
                HookMarkerEvents(marker);
                _markerToSensor[marker] = r;

                marker.ContextMenu = BuildSensorContextMenu(r);
                if (object.ReferenceEquals(_selectedSensor, r))
                {
                    var halo = new Ellipse
                    {
                        Width = MarkerSize + 10,
                        Height = MarkerSize + 10,
                        Stroke = Brushes.White,
                        StrokeThickness = 2,
                        Opacity = 0.9
                    };
                    Canvas.SetLeft(halo, r.Position.X - (MarkerSize + 10) / 2);
                    Canvas.SetTop(halo, r.Position.Y - (MarkerSize + 10) / 2);
                    Panel.SetZIndex(halo, 999);
                    Overlay.Children.Add(halo);
                }
                Overlay.Children.Add(marker);
            }

            // КАМЕРЫ
            for (int i = 0; i < _sensorSimulator.Cameras.Count; i++)
            {
                var c = _sensorSimulator.Cameras[i];

                var fov = MakeSector(c.Position, c.DetectionRange, c.ViewAngle - c.FieldOfView / 2.0, c.ViewAngle + c.FieldOfView / 2.0,
                                     new SolidColorBrush(Color.FromArgb(70, 20, 120, 240)),
                                     new SolidColorBrush(Color.FromArgb(160, 135, 206, 235)), 1.0, 0.55);
                Overlay.Children.Add(fov);

                var marker = new Rectangle
                {
                    Width = MarkerSize,
                    Height = MarkerSize,
                    Fill = Brushes.Magenta,
                    Stroke = Brushes.White,
                    StrokeThickness = 2,
                    Cursor = Cursors.SizeAll,
                    ToolTip = c.SensorName + " (" + c.SensorId + ")"
                };
                Canvas.SetLeft(marker, c.Position.X - MarkerSize / 2);
                Canvas.SetTop(marker, c.Position.Y - MarkerSize / 2);
                HookMarkerEvents(marker);
                _markerToSensor[marker] = c;
                marker.ContextMenu = BuildSensorContextMenu(c);
                if (object.ReferenceEquals(_selectedSensor, c))
                {
                    var halo = new Ellipse
                    {
                        Width = MarkerSize + 10,
                        Height = MarkerSize + 10,
                        Stroke = Brushes.White,
                        StrokeThickness = 2,
                        Opacity = 0.9
                    };
                    Canvas.SetLeft(halo, c.Position.X - (MarkerSize + 10) / 2);
                    Canvas.SetTop(halo, c.Position.Y - (MarkerSize + 10) / 2);
                    Panel.SetZIndex(halo, 999);
                    Overlay.Children.Add(halo);
                }

                Overlay.Children.Add(marker);
            }

            // ДРОНЫ
            for (int i = 0; i < _sensorSimulator.Drones.Count; i++)
            {
                var d = _sensorSimulator.Drones[i];

                if (d.PatrolRoute != null && d.PatrolRoute.Count > 1)
                {
                    var pl = new Polyline
                    {
                        Stroke = Brushes.Lime,
                        StrokeThickness = 1.5,
                        Opacity = 0.9
                    };
                    foreach (var p in d.PatrolRoute) pl.Points.Add(p);
                    pl.Points.Add(d.PatrolRoute[0]);
                    Panel.SetZIndex(pl, 400);
                    Overlay.Children.Add(pl);

                    var plHit = new Polyline
                    {
                        Stroke = Brushes.Transparent,
                        StrokeThickness = 12,
                        FillRule = FillRule.Nonzero,
                        IsHitTestVisible = true,
                        Tag = d
                    };
                    foreach (var p in d.PatrolRoute) plHit.Points.Add(p);
                    plHit.Points.Add(d.PatrolRoute[0]);

                    plHit.MouseRightButtonDown += Polyline_MouseRightButtonDown;
                    Panel.SetZIndex(plHit, 800);
                    Overlay.Children.Add(plHit);

                    for (int k = 0; k < d.PatrolRoute.Count; k++)
                    {
                        var wp = d.PatrolRoute[k];
                        var h = new Ellipse
                        {
                            Width = 8,
                            Height = 8,
                            Fill = Brushes.OrangeRed,
                            Stroke = Brushes.White,
                            StrokeThickness = 1,
                            Cursor = Cursors.Hand,
                            ToolTip = "Маршрут БПЛА " + d.SensorName + " — точка #" + (k + 1)
                        };
                        Canvas.SetLeft(h, wp.X - 4);
                        Canvas.SetTop(h, wp.Y - 4);
                        HookWaypointEvents(h, d, k);
                        h.ContextMenu = BuildWaypointContextMenu(d, k);
                        Panel.SetZIndex(h, 2000);
                        if (object.ReferenceEquals(_selectedWpDrone, d) && _selectedWpIndex == k)
                        {
                            var halo = new Ellipse
                            {
                                Width = 14,
                                Height = 14,
                                Stroke = Brushes.White,
                                StrokeThickness = 2,
                                Opacity = 0.95
                            };
                            Canvas.SetLeft(halo, wp.X - 7);
                            Canvas.SetTop(halo, wp.Y - 7);
                            Panel.SetZIndex(halo, 1999);
                            Overlay.Children.Add(halo);
                        }
                        Overlay.Children.Add(h);
                    }
                }

                var dm = new Ellipse
                {
                    Width = MarkerSize,
                    Height = MarkerSize,
                    Fill = Brushes.Gold,
                    Stroke = Brushes.White,
                    StrokeThickness = 2,
                    Cursor = Cursors.SizeAll,
                    ToolTip = d.SensorName + " (" + d.SensorId + ")\nПеретащи — сдвиг маршрута целиком"
                };
                Canvas.SetLeft(dm, d.Position.X - MarkerSize / 2);
                Canvas.SetTop(dm, d.Position.Y - MarkerSize / 2);
                HookMarkerEvents(dm);
                _markerToSensor[dm] = d;
                dm.ContextMenu = BuildSensorContextMenu(d);
                if (object.ReferenceEquals(_selectedSensor, d))
                {
                    var halo = new Ellipse
                    {
                        Width = MarkerSize + 10,
                        Height = MarkerSize + 10,
                        Stroke = Brushes.White,
                        StrokeThickness = 2,
                        Opacity = 0.9
                    };
                    Canvas.SetLeft(halo, d.Position.X - (MarkerSize + 10) / 2);
                    Canvas.SetTop(halo, d.Position.Y - (MarkerSize + 10) / 2);
                    Panel.SetZIndex(halo, 999);
                    Overlay.Children.Add(halo);
                }

                Overlay.Children.Add(dm);
            }
        }

        private static void FindNearestSegment(IList<Point> route, Point p, out int insertAfterIndex, out Point projected)
        {
            insertAfterIndex = 0;
            projected = route[0];
            double best = double.MaxValue;
            int n = route.Count;

            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                var a = route[i];
                var b = route[j];

                double vx = b.X - a.X, vy = b.Y - a.Y;
                double wx = p.X - a.X, wy = p.Y - a.Y;
                double vv = vx * vx + vy * vy;
                double t = vv > 1e-9 ? (vx * wx + vy * wy) / vv : 0.0;
                if (t < 0) t = 0; else if (t > 1) t = 1;

                var proj = new Point(a.X + t * vx, a.Y + t * vy);
                double dx = p.X - proj.X, dy = p.Y - proj.Y;
                double d2 = dx * dx + dy * dy;

                if (d2 < best)
                {
                    best = d2;
                    projected = proj;
                    insertAfterIndex = i;
                }
            }
        }


        private void Polyline_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (!_isEditMode) return;

            var pl = sender as Polyline;
            if (pl == null) return;
            var drone = pl.Tag as DroneSensor;
            if (drone == null || drone.PatrolRoute == null || drone.PatrolRoute.Count < 2) return;

            var click = e.GetPosition(Overlay);

            int afterIndex;
            Point projected;
            FindNearestSegment(drone.PatrolRoute, click, out afterIndex, out projected);

            InsertWaypoint(drone, afterIndex, projected, insertAfter: true);
            BuildEditorOverlay();
            e.Handled = true;
        }


        private void InsertWaypoint(DroneSensor drone, int index, Point newPoint, bool insertAfter)
        {
            if (drone.PatrolRoute == null) drone.PatrolRoute = new List<Point>();
            int i = insertAfter ? (index + 1) : index;
            if (i < 0) i = 0;
            if (i > drone.PatrolRoute.Count) i = drone.PatrolRoute.Count;
            drone.PatrolRoute.Insert(i, newPoint);
        }

        private void RemoveWaypoint(DroneSensor drone, int index)
        {
            if (drone.PatrolRoute == null) return;
            if (index < 0 || index >= drone.PatrolRoute.Count) return;
            if (drone.PatrolRoute.Count <= 2) return;
            drone.PatrolRoute.RemoveAt(index);
        }


        private ContextMenu BuildWaypointContextMenu(DroneSensor drone, int index)
        {
            var cm = new ContextMenu();

            var miBefore = new MenuItem { Header = "Вставить точку до" };
            miBefore.Click += delegate
            {
                var p = drone.PatrolRoute[index];
                var np = new Point(p.X + 12, p.Y + 12);
                InsertWaypoint(drone, index, np, insertAfter: false);
                BuildEditorOverlay();
            };
            cm.Items.Add(miBefore);

            var miAfter = new MenuItem { Header = "Вставить точку после" };
            miAfter.Click += delegate
            {
                var p = drone.PatrolRoute[index];
                var np = new Point(p.X + 12, p.Y + 12);
                InsertWaypoint(drone, index, np, insertAfter: true);
                BuildEditorOverlay();
            };
            cm.Items.Add(miAfter);

            var miDel = new MenuItem { Header = "Удалить точку" };
            miDel.Click += delegate
            {
                RemoveWaypoint(drone, index);
                if (object.ReferenceEquals(_selectedWpDrone, drone) && _selectedWpIndex == index)
                {
                    _selectedWpDrone = null;
                    _selectedWpIndex = -1;
                }
                BuildEditorOverlay();
            };
            cm.Items.Add(miDel);

            return cm;
        }


        private void MainWindow_PreviewKeyDown(object sender, KeyEventArgs e)
        {
            if (_isEditMode && e.Key == Key.Delete)
            {
                if (_selectedWpDrone != null && _selectedWpIndex >= 0)
                {
                    RemoveWaypoint(_selectedWpDrone, _selectedWpIndex);
                    _selectedWpDrone = null;
                    _selectedWpIndex = -1;
                    BuildEditorOverlay();
                    e.Handled = true;
                    return;
                }
            }
            if (!_isEditMode) return;
            if (e.Key != Key.Delete || _selectedSensor == null) return;

            if (_selectedSensor is RadarSensor) _sensorSimulator.RemoveRadar(_selectedSensor.SensorId);
            else if (_selectedSensor is VideoCameraSensor) _sensorSimulator.RemoveCamera(_selectedSensor.SensorId);
            else if (_selectedSensor is DroneSensor) _sensorSimulator.RemoveDrone(_selectedSensor.SensorId);

            _selectedSensor = null;
            BuildEditorOverlay();
            e.Handled = true;
        }


        private void EnterEditMode()
        {
            BuildEditorOverlay();
            Overlay.PreviewMouseRightButtonDown += Overlay_PreviewMouseRightButtonDown;
            this.Focusable = true;
            this.Focus();
        }

        private void ExitEditMode()
        {
            Overlay.PreviewMouseRightButtonDown -= Overlay_PreviewMouseRightButtonDown;
            _markerToSensor.Clear();
            _draggedElement = null;
            _draggedSensor = null;
            _dragStartDroneRoute = null;
        }


        private void OnClockTick()
        {
            if (_isEditMode) return;

            Dispatcher.Invoke(() =>
            {
                if (!_trajectoryGenerator.hasReachedPerimeter)
                {
                    _trajectoryGenerator.UpdatePosition(0.1); // 100ms = 0.1s

                    _sensorSimulator?.UpdateAllSensors(_trajectoryGenerator.CurrentPosition, 0.1);

                    var activeDetections = _sensorSimulator?.GetActiveDetections() ?? new List<SensorDetection>();
                    if (activeDetections.Count > 0)
                    {
                        _detectionLogCount += activeDetections.Count;
                    }

                    Dispatcher.Invoke(() =>
                    {
                        Redraw();
                        UpdateTimerDisplay();
                    });
                }
                else
                {
                    _clockGenerator.Stop();
                    Dispatcher.Invoke(() =>
                    {
                        BtnStart.IsEnabled = true;
                        BtnStop.IsEnabled = false;
                    });
                }
            });
        }

        private void UpdateTimerDisplay()
        {
            TimeSpan elapsed = DateTime.Now - _simulationStartTime;
            TimerText.Text = elapsed.ToString(@"hh\:mm\:ss");
        }

        private void OnPropertyChanged(string propertyName)
        => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        private System.Windows.Shapes.Path MakeSector(Point center, double radius, double startDeg, double endDeg, Brush fill, Brush stroke, double strokeThickness, double opacity = 0.45)
        {
            double s = startDeg;
            double e = endDeg;
            if (e < s) e += 360.0;

            double sRad = s * Math.PI / 180.0;
            double eRad = e * Math.PI / 180.0;

            Point startPt = new Point(center.X + radius * Math.Cos(sRad),
                                      center.Y + radius * Math.Sin(sRad));
            Point endPt = new Point(center.X + radius * Math.Cos(eRad),
                                      center.Y + radius * Math.Sin(eRad));

            bool isLargeArc = (e - s) > 180.0;

            var fig = new PathFigure { StartPoint = center, IsClosed = true, IsFilled = true };
            fig.Segments.Add(new LineSegment(startPt, true));
            fig.Segments.Add(new ArcSegment(
                endPt, new Size(radius, radius), 0,
                isLargeArc, SweepDirection.Clockwise, true));
            fig.Segments.Add(new LineSegment(center, true));

            var geo = new PathGeometry();
            geo.Figures.Add(fig);

            return new System.Windows.Shapes.Path
            {
                Data = geo,
                Fill = fill,
                Stroke = stroke,
                StrokeThickness = strokeThickness,
                Opacity = opacity
            };
        }

        private INotifyCollectionChanged _detectionsCollectionTop;

        private void DetectionsListBox_Loaded(object sender, RoutedEventArgs e)
        {
            var view = CollectionViewSource.GetDefaultView(DetectionsListBox.ItemsSource);
            if (view != null)
                view.Filter = o => (o as SensorDetection)?.IsDetected == true;

            if (_detectionsCollectionTop != null)
                _detectionsCollectionTop.CollectionChanged -= OnDetectionsTopChanged;

            _detectionsCollectionTop = SensorSimulator?.AllDetections as INotifyCollectionChanged;
            if (_detectionsCollectionTop != null)
                _detectionsCollectionTop.CollectionChanged += OnDetectionsTopChanged;

            DetectionsListBox.Dispatcher.BeginInvoke(new Action(() =>
            {
                if (DetectionsListBox.Items.Count > 0)
                    DetectionsListBox.ScrollIntoView(
                        DetectionsListBox.Items[DetectionsListBox.Items.Count - 1]);
            }));
        }

        private void OnDetectionsTopChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            if (e.Action == NotifyCollectionChangedAction.Add || e.Action == NotifyCollectionChangedAction.Reset)
            {
                DetectionsListBox.Dispatcher.BeginInvoke(new Action(() =>
                {
                    if (DetectionsListBox.Items.Count > 0)
                    {
                        var last = DetectionsListBox.Items[DetectionsListBox.Items.Count - 1];
                        DetectionsListBox.ScrollIntoView(last);
                    }
                }), System.Windows.Threading.DispatcherPriority.Background);
            }
        }
        private void SensorPanel_DroneViewRequested(object sender, SensorPanelControl.DroneViewEventArgs e)
        {
            var drone = _sensorSimulator.Drones.FirstOrDefault(d => d.SensorId == e.DroneId);
            if (drone == null) return;

            if (_droneWindows.TryGetValue(e.DroneId, out var opened))
            {
                opened.Close();
                _droneWindows.Remove(e.DroneId);
                return;
            }

            var wnd = new DroneViewWindow(drone) { Owner = this };
            wnd.Closed += (_, __) => _droneWindows.Remove(e.DroneId);
            _droneWindows[e.DroneId] = wnd;
            wnd.Show();
        }

        private void BtnExport_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_lastResults == null)
                {
                    MessageBox.Show("Нет данных симуляции для экспорта. Сначала запустите симуляцию.", "Экспорт", MessageBoxButton.OK, MessageBoxImage.Information);
                    return;
                }

                string exportDir = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "ExportedData");
                var bundle = DetectionDataExporter.ExportAll(exportDir, _lastResults, "simulation");

                string message = $"Данные экспортированы:\n\nJSON: {bundle.JsonPath}\nCSV:  {bundle.CsvPath}";
                MessageBox.Show(message, "Экспорт данных", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Ошибка при экспорте данных: {ex.Message}", "Ошибка", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }
    }
}