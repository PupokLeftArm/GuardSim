using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Media;

namespace GuardSim
{
    public abstract class SensorBase : INotifyPropertyChanged
    {
        public string SensorId { get; set; }
        public string SensorName { get; set; }
        public Point Position { get; set; }
        public double DetectionRange { get; set; }
        public double NoiseLevel { get; set; }
        public Brush VisualizationColor { get; set; }
        public bool IsActive { get; set; } = true;
        public DateTime LastDetectionTime { get; set; }

        private bool _hasDetection = false;
        public bool HasDetection
        {
            get => _hasDetection;
            set
            {
                if (_hasDetection != value)
                {
                    _hasDetection = value;
                    OnPropertyChanged();
                    OnPropertyChanged(nameof(DetectionStatus));
                }
            }
        }

        public Point LastDetectedPosition { get; set; }

        private double _detectionConfidence = 0.0;
        public double DetectionConfidence
        {
            get => _detectionConfidence;
            set
            {
                if (_detectionConfidence != value)
                {
                    _detectionConfidence = value;
                    OnPropertyChanged();
                }
            }
        }

        protected Random _random;
        public virtual string DetectionStatus => HasDetection ? "Обнаружено" : "Не обнаружено";

        private bool _isActivated = false;
        public virtual bool IsActivated
        {
            get => _isActivated;
            set
            {
                if (_isActivated != value)
                {
                    _isActivated = value;
                    OnPropertyChanged();
                }
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        public SensorBase(string sensorId, string sensorName, Point position, double detectionRange)
        {
            SensorId = sensorId;
            SensorName = sensorName;
            Position = position;
            DetectionRange = detectionRange;
            _random = new Random();
        }

        public abstract SensorDetection CheckDetection(Point intruderPosition, double intruderRadius = 5.0);

        protected double AddGaussianNoise(double value, double stdDeviation)
        {
            double u1 = _random.NextDouble();
            double u2 = _random.NextDouble();
            double z = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
            return value + z * stdDeviation;
        }

        protected double GetDistance(Point p1, Point p2)
        {
            double dx = p1.X - p2.X;
            double dy = p1.Y - p2.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }
    }

    public class SensorDetection
    {
        public string SensorId { get; set; }
        public string SensorName { get; set; }
        public string SensorType { get; set; }
        public DateTime Timestamp { get; set; }
        public bool IsDetected { get; set; }
        public Point MeasuredPosition { get; set; }
        public Point ActualPosition { get; set; }
        public double PositionError { get; set; }
        public double Confidence { get; set; }
        public double Azimuth { get; set; }
        public double Range { get; set; }
        public double RawVelocity { get; set; }
        public double SignalStrength { get; set; }

        public SensorDetection()
        {
            Timestamp = DateTime.Now;
            MeasuredPosition = new Point(0, 0);
            ActualPosition = new Point(0, 0);
            Confidence = 0.0;
            IsDetected = false;
        }
    }

    public class RadarSensor : SensorBase
    {
        public double BeamWidth { get; set; } = 10.0;   // град
        private double _currentScanAngle = 0.0;
        private readonly Random _velocityRandom = new Random();

        public double CurrentScanAngle => _currentScanAngle;
        public override string DetectionStatus => HasDetection ? "Обнаружено" : "Не обнаружено";

        public RadarSensor(string sensorId, string sensorName, Point position,
            double detectionRange, double noiseLevel = 2.0)
            : base(sensorId, sensorName, position, detectionRange)
        {
            NoiseLevel = noiseLevel;
            VisualizationColor = new SolidColorBrush(Color.FromRgb(0, 37, 255));
        }

        public override SensorDetection CheckDetection(Point intruderPosition, double intruderRadius = 5.0)
        {
            var detection = new SensorDetection
            {
                SensorId = SensorId,
                SensorName = SensorName,
                SensorType = "Radar",
                Timestamp = DateTime.Now,
                ActualPosition = intruderPosition
            };

            double distance = GetDistance(Position, intruderPosition);
            if (distance > DetectionRange)
            {
                detection.IsDetected = false;
                detection.Confidence = 0.0;
                return detection;
            }

            double dx = intruderPosition.X - Position.X;
            double dy = intruderPosition.Y - Position.Y;
            double azimuth = Math.Atan2(dy, dx) * 180.0 / Math.PI;
            if (azimuth < 0) azimuth += 360.0;

            double angleDiff = Math.Abs(azimuth - _currentScanAngle);
            if (angleDiff > 180) angleDiff = 360 - angleDiff;
            if (angleDiff > BeamWidth / 2.0)
            {
                detection.IsDetected = false;
                detection.Confidence = 0.0;
                return detection;
            }

            double snr = 30.0 - NoiseLevel;
            double baseProbability = 1.0 / (1.0 + Math.Exp(-snr / 10.0));
            double confidenceMultiplier = Math.Max(0.0, 1.0 - (distance / DetectionRange) * 0.3);
            double confidence = baseProbability * confidenceMultiplier;

            if (_random.NextDouble() > confidence)
            {
                detection.IsDetected = false;
                detection.Confidence = confidence * 0.5;
                return detection;
            }

            double noisyRange = AddGaussianNoise(distance, NoiseLevel / 10.0);
            double noisyAzimuth = AddGaussianNoise(azimuth, NoiseLevel / 5.0);

            double measuredX = Position.X + noisyRange * Math.Cos(noisyAzimuth * Math.PI / 180.0);
            double measuredY = Position.Y + noisyRange * Math.Sin(noisyAzimuth * Math.PI / 180.0);

            detection.IsDetected = true;
            detection.MeasuredPosition = new Point(measuredX, measuredY);
            detection.Range = noisyRange;
            detection.Azimuth = noisyAzimuth;
            detection.PositionError = GetDistance(intruderPosition, detection.MeasuredPosition);
            detection.Confidence = confidence;
            detection.SignalStrength = 30.0 - NoiseLevel - (distance / DetectionRange) * 15.0;

            detection.RawVelocity = AddGaussianNoise(0.0, NoiseLevel);

            LastDetectionTime = detection.Timestamp;
            LastDetectedPosition = detection.MeasuredPosition;
            HasDetection = true;
            DetectionConfidence = confidence;

            return detection;
        }

        public void UpdateScanAngle(double deltaTime)
        {
            _currentScanAngle += 60.0 * deltaTime + _velocityRandom.NextDouble() * 10;
            if (_currentScanAngle >= 360.0)
                _currentScanAngle -= 360.0;
        }
    }

    public class VideoCameraSensor : SensorBase
    {
        public double FieldOfView { get; set; } = 90.0; // град
        public double ViewAngle { get; set; } = 0.0;    // град

        private bool _isActivated = false;
        public override bool IsActivated
        {
            get => _isActivated;
            set
            {
                if (_isActivated != value)
                {
                    _isActivated = value;
                    OnPropertyChanged();
                    if (!value)
                    {
                        HasDetection = false;
                        DetectionConfidence = 0.0;
                    }
                }
            }
        }

        public VideoCameraSensor(string sensorId, string sensorName, Point position,
            double detectionRange, double viewAngle = 0.0, double noiseLevel = 3.0)
            : base(sensorId, sensorName, position, detectionRange)
        {
            NoiseLevel = noiseLevel;
            ViewAngle = viewAngle;
            VisualizationColor = new SolidColorBrush(Color.FromRgb(0, 0, 255));
        }

        public override SensorDetection CheckDetection(Point intruderPosition, double intruderRadius = 5.0)
        {
            var detection = new SensorDetection
            {
                SensorId = SensorId,
                SensorName = SensorName,
                SensorType = "Camera",
                Timestamp = DateTime.Now,
                ActualPosition = intruderPosition
            };

            if (!IsActivated)
            {
                detection.IsDetected = false;
                detection.Confidence = 0.0;
                HasDetection = false;
                return detection;
            }

            double distance = GetDistance(Position, intruderPosition);
            if (distance > DetectionRange)
            {
                detection.IsDetected = false;
                detection.Confidence = 0.0;
                return detection;
            }

            double dx = intruderPosition.X - Position.X;
            double dy = intruderPosition.Y - Position.Y;
            double targetAngle = Math.Atan2(dy, dx) * 180.0 / Math.PI;
            if (targetAngle < 0) targetAngle += 360.0;

            double angleDiff = Math.Abs(targetAngle - ViewAngle);
            if (angleDiff > 180) angleDiff = 360 - angleDiff;
            if (angleDiff > FieldOfView / 2.0)
            {
                detection.IsDetected = false;
                detection.Confidence = 0.0;
                return detection;
            }

            double confidence = Math.Max(0.0, 1.0 - (distance / DetectionRange) * 0.4);

            double videoNoiseStd = NoiseLevel / 5.0;
            double noisyX = AddGaussianNoise(intruderPosition.X, videoNoiseStd);
            double noisyY = AddGaussianNoise(intruderPosition.Y, videoNoiseStd);

            detection.IsDetected = _random.NextDouble() < confidence;
            detection.MeasuredPosition = new Point(noisyX, noisyY);
            detection.PositionError = GetDistance(intruderPosition, detection.MeasuredPosition);
            detection.Confidence = confidence;
            detection.SignalStrength = 25.0 - NoiseLevel - (distance / DetectionRange) * 10.0;

            if (detection.IsDetected)
            {
                LastDetectionTime = detection.Timestamp;
                LastDetectedPosition = detection.MeasuredPosition;
                HasDetection = true;
                DetectionConfidence = confidence;
            }

            return detection;
        }

        public void Activate() => IsActivated = true;
        public void Deactivate() => IsActivated = false;
    }

    public class DroneSensor : SensorBase
    {
        public List<Point> PatrolRoute { get; set; }
        public double Speed { get; set; } = 50.0;
        public int CurrentWaypoint { get; set; } = 0;
        public double ScanRadius { get; set; } = 100.0;
        public double FieldOfView { get; set; } = 60.0;
        public double ViewAngle { get; private set; } = 0.0;

        private double _routeProgress = 0.0;

        public DroneSensor(string sensorId, string sensorName, List<Point> patrolRoute,
            double scanRadius, double noiseLevel = 2.5)
            : base(sensorId, sensorName, patrolRoute != null && patrolRoute.Count > 0 ? patrolRoute[0] : new Point(0, 0), scanRadius)
        {
            PatrolRoute = patrolRoute ?? new List<Point>();
            ScanRadius = scanRadius;
            NoiseLevel = noiseLevel;
            VisualizationColor = new SolidColorBrush(Color.FromRgb(34, 57, 41));
            DetectionRange = scanRadius;

            if (PatrolRoute.Count >= 2)
            {
                var p1 = PatrolRoute[0];
                var p2 = PatrolRoute[1];
                var angleDeg = Math.Atan2(p2.Y - p1.Y, p2.X - p1.X) * 180.0 / Math.PI;
                if (angleDeg < 0) angleDeg += 360.0;
                ViewAngle = angleDeg;
            }
        }

        private bool IsPointInSector(Point point, Point center, double radius, double startDeg, double endDeg)
        {
            while (endDeg < startDeg) endDeg += 360;
            double dx = point.X - center.X;
            double dy = point.Y - center.Y;
            double angle = Math.Atan2(dy, dx) * 180.0 / Math.PI;
            if (angle < 0) angle += 360;

            bool angleInRange = (angle >= startDeg && angle <= endDeg);
            double distance = Math.Sqrt(dx * dx + dy * dy);
            bool distanceInRange = (distance <= radius);

            return angleInRange && distanceInRange;
        }

        public override SensorDetection CheckDetection(Point intruderPosition, double intruderRadius = 5.0)
        {
            var detection = new SensorDetection
            {
                SensorId = SensorId,
                SensorName = SensorName,
                SensorType = "Drone",
                Timestamp = DateTime.Now,
                ActualPosition = intruderPosition
            };

            double distance = GetDistance(Position, intruderPosition);
            if (distance > ScanRadius)
            {
                detection.IsDetected = false;
                detection.Confidence = 0.0;
                return detection;
            }

            bool isInSector = IsPointInSector(intruderPosition, Position, ScanRadius, ViewAngle - FieldOfView / 2, ViewAngle + FieldOfView / 2);
            if (!isInSector)
            {
                detection.IsDetected = false;
                detection.Confidence = 0.0;
                return detection;
            }

            double confidence = Math.Max(0.0, 1.0 - (distance / ScanRadius) * 0.2);
            detection.Confidence = confidence;
            detection.IsDetected = true;

            double noisyX = AddGaussianNoise(intruderPosition.X, NoiseLevel / 8.0);
            double noisyY = AddGaussianNoise(intruderPosition.Y, NoiseLevel / 8.0);

            detection.MeasuredPosition = new Point(noisyX, noisyY);
            detection.PositionError = GetDistance(intruderPosition, detection.MeasuredPosition);

            LastDetectionTime = detection.Timestamp;
            LastDetectedPosition = detection.MeasuredPosition;
            HasDetection = true;
            DetectionConfidence = confidence;

            return detection;
        }

        public void UpdatePosition(double deltaTime)
        {
            if (PatrolRoute == null || PatrolRoute.Count < 2) return;

            double total = 0.0;
            for (int i = 0; i < PatrolRoute.Count; i++)
            {
                int j = (i + 1) % PatrolRoute.Count;
                total += Distance(PatrolRoute[i], PatrolRoute[j]);
            }

            _routeProgress += Speed * deltaTime;
            while (_routeProgress > total) _routeProgress -= total;

            double acc = 0.0;
            for (int i = 0; i < PatrolRoute.Count; i++)
            {
                int j = (i + 1) % PatrolRoute.Count;
                var a = PatrolRoute[i];
                var b = PatrolRoute[j];
                double seg = Distance(a, b);
                if (acc + seg >= _routeProgress)
                {
                    double t = seg > 1e-9 ? (_routeProgress - acc) / seg : 0.0;
                    Position = new Point(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t);
                    CurrentWaypoint = i;

                    double ang = Math.Atan2(b.Y - a.Y, b.X - a.X) * 180.0 / Math.PI;
                    if (ang < 0) ang += 360.0;
                    ViewAngle = ang;
                    return;
                }
                acc += seg;
            }
        }

        private static double Distance(Point p1, Point p2)
        {
            double dx = p2.X - p1.X;
            double dy = p2.Y - p1.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }
    }

    public class SensorDataSimulator
    {
        private readonly List<SensorBase> _sensors = new List<SensorBase>();
        public ObservableCollection<SensorDetection> AllDetections { get; }
        private readonly Dictionary<string, List<SensorDetection>> _detectionHistory;

        public List<RadarSensor> Radars { get; }
        public List<VideoCameraSensor> Cameras { get; }
        public List<DroneSensor> Drones { get; }
        public ObservableCollection<SensorStatus> SensorStatuses { get; }

        public SensorDataSimulator()
        {
            AllDetections = new ObservableCollection<SensorDetection>();
            _detectionHistory = new Dictionary<string, List<SensorDetection>>();
            Radars = new List<RadarSensor>();
            Cameras = new List<VideoCameraSensor>();
            Drones = new List<DroneSensor>();
            SensorStatuses = new ObservableCollection<SensorStatus>();
        }

        public RadarSensor AddRadar(string sensorId, string sensorName, Point position, double range, double noiseLevel = 2.0)
        {
            var radar = new RadarSensor(sensorId, sensorName, position, range, noiseLevel);
            _sensors.Add(radar);
            Radars.Add(radar);
            _detectionHistory[sensorId] = new List<SensorDetection>();
            return radar;
        }

        public VideoCameraSensor AddCamera(string sensorId, string sensorName, Point position, double range,
            double viewAngle = 0.0, double noiseLevel = 3.0)
        {
            var camera = new VideoCameraSensor(sensorId, sensorName, position, range, viewAngle, noiseLevel);
            _sensors.Add(camera);
            Cameras.Add(camera);
            _detectionHistory[sensorId] = new List<SensorDetection>();
            return camera;
        }

        public VideoCameraSensor AddCamera(string id, string name, Point pos,
                                   double range, double fov, double viewAngle, double noise)
        {
            var cam = new VideoCameraSensor(id, name, pos, range, fov, noise);
            cam.ViewAngle = viewAngle;
            Cameras.Add(cam);
            _sensors.Add(cam);
            return cam;
        }


        public DroneSensor AddDrone(string sensorId, string sensorName, List<Point> patrolRoute,
            double scanRadius, double noiseLevel = 2.5)
        {
            var drone = new DroneSensor(sensorId, sensorName, patrolRoute, scanRadius, noiseLevel);
            _sensors.Add(drone);
            Drones.Add(drone);
            _detectionHistory[sensorId] = new List<SensorDetection>();
            return drone;
        }

        public bool RemoveRadar(string sensorId)
        {
            var r = Radars.Find(x => x.SensorId == sensorId);
            if (r == null) return false;
            Radars.Remove(r);
            var all = _sensors.Find(x => x.SensorId == sensorId);
            if (all != null) _sensors.Remove(all);
            if (_detectionHistory.ContainsKey(sensorId)) _detectionHistory.Remove(sensorId);
            return true;
        }

        public bool RemoveCamera(string sensorId)
        {
            var c = Cameras.Find(x => x.SensorId == sensorId);
            if (c == null) return false;
            Cameras.Remove(c);
            var all = _sensors.Find(x => x.SensorId == sensorId);
            if (all != null) _sensors.Remove(all);
            if (_detectionHistory.ContainsKey(sensorId)) _detectionHistory.Remove(sensorId);
            return true;
        }

        public bool RemoveDrone(string sensorId)
        {
            var d = Drones.Find(x => x.SensorId == sensorId);
            if (d == null) return false;
            Drones.Remove(d);
            var all = _sensors.Find(x => x.SensorId == sensorId);
            if (all != null) _sensors.Remove(all);
            if (_detectionHistory.ContainsKey(sensorId)) _detectionHistory.Remove(sensorId);
            return true;
        }

        public void UpdateAllSensors(Point intruderPosition, double deltaTime)
        {
            foreach (var radar in Radars)
            {
                radar.UpdateScanAngle(deltaTime);
                RecordDetection(radar.CheckDetection(intruderPosition));
            }

            foreach (var drone in Drones)
            {
                drone.UpdatePosition(deltaTime);
                RecordDetection(drone.CheckDetection(intruderPosition));
            }

            foreach (var camera in Cameras)
            {
                if (camera.IsActivated)
                    RecordDetection(camera.CheckDetection(intruderPosition));
                else
                    UpdateSensorStatus(camera, false);
            }
        }

        public bool IsDetected(Point pos)
        {
            foreach (var r in Radars) if (r.CheckDetection(pos).IsDetected) return true;
            foreach (var d in Drones) if (d.CheckDetection(pos).IsDetected) return true;
            foreach (var c in Cameras)
            {
                if (c.CheckDetection(pos).IsDetected) return true;
            }
            return false;
        }

        private void RecordDetection(SensorDetection detection)
        {
            if (detection?.IsDetected != true) return;

            if (!_detectionHistory.TryGetValue(detection.SensorId, out var list))
            {
                list = new List<SensorDetection>();
                _detectionHistory[detection.SensorId] = list;
            }

            Application.Current?.Dispatcher?.Invoke(() =>
            {
                list.Add(detection);
                AllDetections.Add(detection);

                var sensor = _sensors.FirstOrDefault(s => s.SensorId == detection.SensorId);
                if (sensor != null) UpdateSensorStatus(sensor, true);
            });
        }

        public List<SensorDetection> GetActiveDetections()
            => AllDetections.Where(d => d.IsDetected).ToList();

        public void ClearLogs()
        {
            Application.Current?.Dispatcher?.Invoke(() =>
            {
                AllDetections.Clear();
                foreach (var kv in _detectionHistory) kv.Value.Clear();
            });
        }

        public void ResetAllStatuses()
        {
            foreach (var sensor in _sensors)
            {
                sensor.HasDetection = false;
                sensor.DetectionConfidence = 0.0;
                UpdateSensorStatus(sensor, false);
            }
            Application.Current?.Dispatcher?.Invoke(() => AllDetections.Clear());
        }

        public List<SensorBase> GetAllSensors() => _sensors;

        public void UpdateSensorStatus(SensorBase sensor, bool hasDetection)
        {
            var status = SensorStatuses.FirstOrDefault(s => s.SensorId == sensor.SensorId);
            if (status == null)
            {
                status = new SensorStatus
                {
                    SensorId = sensor.SensorId,
                    SensorName = sensor.SensorName,
                    SensorType = sensor.GetType().Name.Replace("Sensor", "")
                };
                Application.Current?.Dispatcher?.Invoke(() => SensorStatuses.Add(status));
            }

            string statusText = hasDetection ? "⚫ ОБНАРУЖЕНО" : "⚫ НЕТ ОБНАРУЖЕНИЯ";
            Brush statusColor = hasDetection ? Brushes.Red : Brushes.White;

            if (sensor is VideoCameraSensor camera)
            {
                statusText = camera.IsActivated
                    ? (hasDetection ? "⚫ ОБНАРУЖЕНО" : "⚫ АКТИВНА")
                    : "⚫ ВЫКЛЮЧЕНА";
                statusColor = hasDetection ? Brushes.Red : (camera.IsActivated ? Brushes.White : Brushes.Gray);
            }

            Application.Current?.Dispatcher?.Invoke(() =>
            {
                status.Status = statusText;
                status.StatusColor = statusColor;
                status.IsActive = hasDetection;
            });
        }
    }

    public class SensorStatus : INotifyPropertyChanged
    {
        public string SensorId { get; set; }
        public string SensorName { get; set; }
        public string SensorType { get; set; }

        private string _status;
        public string Status { get => _status; set { _status = value; OnPropertyChanged(); } }

        private Brush _statusColor = Brushes.White;
        public Brush StatusColor { get => _statusColor; set { _statusColor = value; OnPropertyChanged(); } }

        private bool _isActive;
        public bool IsActive { get => _isActive; set { _isActive = value; OnPropertyChanged(); } }

        public event PropertyChangedEventHandler PropertyChanged;
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }
}