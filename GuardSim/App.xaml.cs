using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Threading;
using System.Timers;

namespace GuardSim
{

    public partial class App : Application { }

    public class ClockGenerator
    {
        private readonly Timer _timer;
        public event Action Tick;

        public ClockGenerator(double intervalMs = 500)
        {
            _timer = new Timer(intervalMs);
            _timer.Elapsed += OnTimerElapsed;
        }

        public void Start() => _timer.Start();
        public void Stop() => _timer.Stop();

        private void OnTimerElapsed(object sender, ElapsedEventArgs e)
        {
            Tick?.Invoke();
        }
    }


    public class TrajectoryGenerator
    {
        public Point CurrentPosition { get; private set; }
        public double CurrentSpeed { get; private set; }
        public ObservableCollection<Point> Trajectory { get; private set; }
        public bool hasReachedPerimeter = false;

        private Random _random;
        private List<Point> _perimeterPoints;
        private List<Zone> _zones;
        private Window _mainWindow;
        private double _baseSpeed;
        private Point _targetPerimeterPoint;
        private Dispatcher _dispatcher;

        public TrajectoryGenerator(List<Point> perimeterPoints, List<Zone> zones, double minSpeed = 5, double maxSpeed = 10, Window mainWindow = null)
        {
            _random = new Random();
            _perimeterPoints = perimeterPoints;
            _zones = zones;
            _mainWindow = mainWindow;
            _dispatcher = mainWindow?.Dispatcher ?? Dispatcher.CurrentDispatcher;
            _baseSpeed = _random.NextDouble() * (maxSpeed - minSpeed) + minSpeed;
            CurrentSpeed = _baseSpeed;
            Trajectory = new ObservableCollection<Point>();
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

        private bool IsPointInPolygon(Point point, List<Point> polygonPoints)
        {
            int count = polygonPoints.Count;
            bool inside = false;

            for (int i = 0, j = count - 1; i < count; j = i++)
            {
                if (((polygonPoints[i].Y > point.Y) != (polygonPoints[j].Y > point.Y)) &&
                    (point.X < (polygonPoints[j].X - polygonPoints[i].X) *
                     (point.Y - polygonPoints[i].Y) /
                     (polygonPoints[j].Y - polygonPoints[i].Y) + polygonPoints[i].X))
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        private double GetCurrentZoneWeight()
        {
            double maxWeight = 1.0;
            try
            {
                foreach (var zone in _zones)
                {
                    var zonePoints = ToPoints(zone.Points);
                    if (IsPointInPolygon(CurrentPosition, zonePoints))
                    {
                        if (zone.Weight > maxWeight)
                        {
                            maxWeight = zone.Weight;
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Ошибка при определении веса зоны: {ex.Message}");
                return 1.0;
            }

            return maxWeight;
        }

        public void UpdatePosition(double deltaTime)
        {
            if (hasReachedPerimeter) return;

            double zoneWeight = GetCurrentZoneWeight();
            double effectiveSpeed = _baseSpeed / zoneWeight;
            CurrentSpeed = effectiveSpeed;

            double dx = _targetPerimeterPoint.X - CurrentPosition.X;
            double dy = _targetPerimeterPoint.Y - CurrentPosition.Y;
            double distance = Math.Sqrt(dx * dx + dy * dy);

            if (distance <= 1)
            {
                hasReachedPerimeter = true;
                ShowExitNotification();
                return;
            }

            if (distance > 0)
            {
                dx /= distance;
                dy /= distance;
            }

            double moveDistance = effectiveSpeed * deltaTime;
            if (moveDistance > distance)
            {
                moveDistance = distance;
            }

            double newX = CurrentPosition.X + dx * moveDistance;
            double newY = CurrentPosition.Y + dy * moveDistance;
            Point newPosition = new Point(newX, newY);

            CurrentPosition = newPosition;

            _dispatcher.Invoke(() => Trajectory.Add(CurrentPosition));
        }

        private Point FindClosestPointOnPerimeter()
        {
            double minDistance = double.MaxValue;
            Point closestPoint = CurrentPosition;

            for (int i = 0; i < _perimeterPoints.Count; i++)
            {
                Point a = _perimeterPoints[i];
                Point b = _perimeterPoints[(i + 1) % _perimeterPoints.Count];

                Point pointOnEdge = GetClosestPointOnLineSegment(a, b, CurrentPosition);
                double distance = Math.Sqrt(Math.Pow(pointOnEdge.X - CurrentPosition.X, 2) +
                                           Math.Pow(pointOnEdge.Y - CurrentPosition.Y, 2));

                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestPoint = pointOnEdge;
                }
            }

            return closestPoint;
        }

        private Point GetClosestPointOnLineSegment(Point a, Point b, Point p)
        {
            double abX = b.X - a.X;
            double abY = b.Y - a.Y;

            double apX = p.X - a.X;
            double apY = p.Y - a.Y;

            double dotProduct = apX * abX + apY * abY;
            double abLengthSquared = abX * abX + abY * abY;

            double t = dotProduct / abLengthSquared;
            t = Math.Max(0, Math.Min(1, t));

            return new Point(a.X + t * abX, a.Y + t * abY);
        }

        private void ShowExitNotification()
        {
            if (_mainWindow != null)
            {
                _mainWindow.Dispatcher.Invoke(() =>
                {
                    MessageBox.Show("Нарушитель достиг периметра!");
                });
            }
        }

        public void SetRandomPosition(double minX, double maxX, double minY, double maxY)
        {
                int pointIndex = _random.Next(_perimeterPoints.Count);
                Point perimeterPoint = _perimeterPoints[pointIndex];

                Point center = CalculateCentroid(_perimeterPoints);
                Point direction = new Point(perimeterPoint.X - center.X, perimeterPoint.Y - center.Y);

                double length = Math.Sqrt(direction.X * direction.X + direction.Y * direction.Y);
                if (length > 0)
                {
                    direction = new Point(direction.X / length, direction.Y / length);
                }

                double offset = 200;
                CurrentPosition = new Point(
                    perimeterPoint.X + direction.X * offset,
                    perimeterPoint.Y + direction.Y * offset
                );
            
            _targetPerimeterPoint = FindClosestPointOnPerimeter();

            _dispatcher.Invoke(() =>
            {
                Trajectory.Clear();
                Trajectory.Add(CurrentPosition);
            });
        }

        private Point CalculateCentroid(List<Point> points)
        {
            double sumX = 0, sumY = 0;
            foreach (var point in points)
            {
                sumX += point.X;
                sumY += point.Y;
            }
            return new Point(sumX / points.Count, sumY / points.Count);
        }      

        public void Reset()
        {
            hasReachedPerimeter = false;
            _dispatcher.Invoke(() => Trajectory.Clear());
        }
    }
}