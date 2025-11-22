using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Windows;

namespace GuardSim
{
    public interface IDetectionProvider
    {
        bool IsDetected(Point pos, double tSeconds);
    }

    public sealed class SensorDetectionAdapter : IDetectionProvider
    {
        private readonly SensorDataSimulator _sensors;
        public bool IsDetected(Point pos, double tSeconds) { return _sensors != null && _sensors.IsDetected(pos); }
    }

    public interface IAvoidanceField
    {
        Vector GetRepulsion(Point p);
    }

    public sealed class SensorAvoidanceField : IAvoidanceField
    {
        private readonly SensorDataSimulator _sensors;
        public double RadarWeight = 1.0;
        public double CameraWeight = 1.0;

        public SensorAvoidanceField(SensorDataSimulator sensors) { _sensors = sensors; }

        public Vector GetRepulsion(Point p)
        {
            Vector total = new Vector(0, 0);

            if (_sensors == null) return total;

            foreach (var r in _sensors.Radars)
            {
                Vector v = new Vector(p.X - r.Position.X, p.Y - r.Position.Y);
                double d = v.Length;
                if (d <= 1e-6 || d > r.DetectionRange) continue;
                v.Normalize();
                double risk = 1.0 - (d / r.DetectionRange);
                double m = RadarWeight * risk * risk;
                total += v * m;
            }


            foreach (var c in _sensors.Cameras)
            {
                Vector v = new Vector(p.X - c.Position.X, p.Y - c.Position.Y);
                double d = v.Length;
                if (d <= 1e-6 || d > c.DetectionRange) continue;

                double angToP = Math.Atan2(v.Y, v.X) * 180.0 / Math.PI;
                double diff = NormalizeAngleDiff(angToP, c.ViewAngle);
                if (Math.Abs(diff) > c.FieldOfView * 0.5) continue;

                v.Normalize();
                double dirFactor = 1.0 - (Math.Abs(diff) / (c.FieldOfView * 0.5));
                double distFactor = 1.0 - (d / c.DetectionRange);
                double m = CameraWeight * dirFactor * distFactor;
                total += v * m;
            }

            return total;
        }

        private static double NormalizeAngleDiff(double a, double b)
        {
            double d = a - b;
            while (d > 180) d -= 360;
            while (d < -180) d += 360;
            return d;
        }
    }

    public sealed class BreachPoint
    {
        public double X { get; set; }
        public double Y { get; set; }
        public int SegmentIndex { get; set; }
    }

    public sealed class SegmentStat
    {
        public int Index { get; set; }
        public Point Start { get; set; }
        public Point End { get; set; }
        public int Breaches { get; set; }
        public double BreachProbability { get; set; }
    }

    public sealed class SimulationResults
    {
        public int TotalIntruders { get; set; }
        public int Intercepted { get; set; }
        public int Breached { get; set; }
        public List<BreachPoint> BreachPoints { get; set; } = new List<BreachPoint>();
        public List<SegmentStat> Segments { get; set; } = new List<SegmentStat>();
        public int MaxSegmentBreaches { get; set; }
        public SegmentStat MostVulnerableSegment { get; set; }

        public void CalculateProbabilities(int totalIntruders)
        {
            foreach (var segment in Segments)
            {
                segment.BreachProbability = totalIntruders > 0
                    ? (double)segment.Breaches / totalIntruders
                    : 0;
            }
        }
    }

    public enum SpawnMode { MapEdges, PerimeterRing }
    public enum ApproachMode { Nearest, RandomPerimeter, SmartAvoidSensors }

    public sealed class MultiIntruderSimulator
    {
        private readonly List<Point> _perimeter;
        private readonly Func<Point, double> _zoneWeight;
        private readonly IDetectionProvider _detector;
        private readonly IAvoidanceField _avoidField;
        private readonly double _mapW, _mapH;
        private readonly Random _rnd = new Random();

        public double DtSeconds { get; set; } = 0.5;
        public double MinSpeed { get; set; } = 5.0;
        public double MaxSpeed { get; set; } = 10.0;
        public int MaxStepsPerIntruder { get; set; } = 5000;

        public SpawnMode Spawn = SpawnMode.PerimeterRing;
        public ApproachMode Approach = ApproachMode.SmartAvoidSensors;

        public double GoalWeight = 1.0;
        public double AvoidWeight = 1.3;
        public double NoiseWeight = 0.12;

        public int SmartTargetSamples = 48;
        public double SpawnOffsetMin = 80;
        public double SpawnOffsetMax = 220;

        private readonly double _perimeterLength;
        private readonly List<double> _cumLen;
        private readonly Point _centroid;

        public MultiIntruderSimulator(
            List<Point> perimeter,
            Func<Point, double> zoneWeight,
            IDetectionProvider detector,
            double mapWidth,
            double mapHeight,
            IAvoidanceField avoidField = null)
        {
            if (perimeter == null) throw new ArgumentNullException("perimeter");
            _perimeter = perimeter;
            _zoneWeight = zoneWeight ?? (p => 1.0);
            _detector = detector;
            _avoidField = avoidField;
            _mapW = mapWidth;
            _mapH = mapHeight;

            _cumLen = BuildCumulativeLengths(perimeter, out _perimeterLength);
            _centroid = ComputeCentroid(perimeter);
        }

        public SimulationResults Run(int intrudersCount, CancellationToken ct = default(CancellationToken))
        {
            var results = new SimulationResults
            {
                TotalIntruders = intrudersCount,
                Segments = BuildEmptySegmentStats()
            };

            for (int i = 0; i < intrudersCount; i++)
            {
                if (ct.IsCancellationRequested) break;

                Point pos = (Spawn == SpawnMode.PerimeterRing)
                            ? RandomPerimeterRingPosition()
                            : RandomEdgePosition();

                double speed = Lerp(MinSpeed, MaxSpeed, _rnd.NextDouble());

                bool intercepted = false;
                bool breached = false;
                double t = 0;

                Point fixedTarget = (Approach == ApproachMode.RandomPerimeter) ? SamplePerimeterPoint(_rnd.NextDouble() * _perimeterLength)
                                  : (Approach == ApproachMode.SmartAvoidSensors) ? ChooseSmartTarget()
                                  : new Point();

                for (int step = 0; step < MaxStepsPerIntruder; step++)
                {
                    Point target = (Approach == ApproachMode.Nearest)
                                   ? ClosestPointOnPerimeter(pos)
                                   : fixedTarget;

                    Vector vGoal = new Vector(target.X - pos.X, target.Y - pos.Y);
                    double distToGoal = vGoal.Length;
                    if (distToGoal > 1e-12) vGoal.Normalize();

                    Vector vAvoid = (_avoidField != null) ? _avoidField.GetRepulsion(pos) : new Vector(0, 0);

                    double ang = _rnd.NextDouble() * Math.PI * 2.0;
                    Vector vNoise = new Vector(Math.Cos(ang), Math.Sin(ang));

                    Vector dir = vGoal * GoalWeight - vAvoid * AvoidWeight + vNoise * NoiseWeight;
                    if (dir.Length < 1e-9)
                        dir = vGoal;
                    else
                        dir.Normalize();

                    double weight = Math.Max(1.0, _zoneWeight(pos));
                    double stepLen = (speed / weight) * DtSeconds;

                    Point prevPos = pos;
                    Point next = new Point(pos.X + dir.X * stepLen, pos.Y + dir.Y * stepLen);

                    if (IsDetectedAlongStep(prevPos, next, t))
                    {
                        intercepted = true;
                        break;
                    }

                    Point hit; int segIndex;
                    if (TryFirstIntersectionWithPerimeter(prevPos, next, out hit, out segIndex))
                    {
                        breached = true;
                        RegisterBreach(results, hit, segIndex);
                        pos = hit;
                        break;
                    }

                    if (Approach != ApproachMode.Nearest)
                    {
                        if (distToGoal <= stepLen + 1.0)
                        {
                            int segIdx0 = FindClosestSegmentIndex(target);
                            breached = true;
                            RegisterBreach(results, target, segIdx0);
                            pos = target;
                            break;
                        }
                    }

                    pos = next;
                    t += DtSeconds;

                    pos = new Point(Clamp(pos.X, 0, _mapW), Clamp(pos.Y, 0, _mapH));
                }

                if (intercepted) results.Intercepted++;
                else if (breached) results.Breached++;
                else results.Intercepted++;
            }
            results.CalculateProbabilities(intrudersCount);
            results.MaxSegmentBreaches = results.Segments.Count == 0 ? 0 : results.Segments.Max(s => s.Breaches);
            results.MostVulnerableSegment = results.Segments.OrderByDescending(s => s.Breaches).FirstOrDefault();
            return results;
        }

        private Point RandomEdgePosition()
        {
            int side = _rnd.Next(4);
            double margin = _rnd.Next(0, 100);
            if (side == 0) return new Point(_rnd.NextDouble() * _mapW, 0 + margin);
            if (side == 1) return new Point(_rnd.NextDouble() * _mapW, _mapH - margin);
            if (side == 2) return new Point(0 + margin, _rnd.NextDouble() * _mapH);
            return new Point(_mapW - margin, _rnd.NextDouble() * _mapH);
        }

        private Point RandomPerimeterRingPosition()
        {
            double s = _rnd.NextDouble() * _perimeterLength;
            Point p = SamplePerimeterPoint(s);

            int segIndex;
            Point a, b; GetSegmentAtLength(s, out segIndex, out a, out b);

            Vector edge = new Vector(b.X - a.X, b.Y - a.Y);
            if (edge.Length < 1e-9) edge = new Vector(1, 0);
            edge.Normalize();

            Vector normal = new Vector(-edge.Y, edge.X);
            Vector toOut = new Vector(p.X - _centroid.X, p.Y - _centroid.Y);
            if (Vector.Multiply(normal, toOut) < 0) normal = -normal;

            double offset = Lerp(SpawnOffsetMin, SpawnOffsetMax, _rnd.NextDouble());
            Point spawn = new Point(p.X + normal.X * offset, p.Y + normal.Y * offset);

            spawn = new Point(Clamp(spawn.X, 0, _mapW), Clamp(spawn.Y, 0, _mapH));
            return spawn;
        }

        private Point ChooseSmartTarget()
        {
            Point best = _perimeter[0];
            double bestScore = double.PositiveInfinity;

            for (int k = 0; k < SmartTargetSamples; k++)
            {
                double s = _rnd.NextDouble() * _perimeterLength;
                Point p = SamplePerimeterPoint(s);

                double score = 0.0;
                if (_avoidField != null)
                {
                    Vector rep = _avoidField.GetRepulsion(p);
                    score += rep.Length;
                }
                score += _rnd.NextDouble() * 0.05;

                if (score < bestScore) { bestScore = score; best = p; }
            }
            return best;
        }

        private List<SegmentStat> BuildEmptySegmentStats()
        {
            var list = new List<SegmentStat>();
            int n = _perimeter.Count;
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                var st = new SegmentStat();
                st.Index = i;
                st.Start = _perimeter[i];
                st.End = _perimeter[j];
                st.Breaches = 0;
                list.Add(st);
            }
            return list;
        }

        private void RegisterBreach(SimulationResults res, Point hit, int segIndex)
        {
            var bp = new BreachPoint();
            bp.X = hit.X; bp.Y = hit.Y; bp.SegmentIndex = segIndex;
            res.BreachPoints.Add(bp);

            if (segIndex >= 0 && segIndex < res.Segments.Count)
                res.Segments[segIndex].Breaches++;
        }

        private Point ClosestPointOnPerimeter(Point p)
        {
            int n = _perimeter.Count;
            double best2 = double.PositiveInfinity;
            Point best = p;

            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                Point q = ClosestPointOnSegment(p, _perimeter[i], _perimeter[j]);
                double d2 = (q.X - p.X) * (q.X - p.X) + (q.Y - p.Y) * (q.Y - p.Y);
                if (d2 < best2) { best2 = d2; best = q; }
            }
            return best;
        }

        private int FindClosestSegmentIndex(Point p)
        {
            int n = _perimeter.Count;
            double best2 = double.PositiveInfinity;
            int bestIdx = 0;
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                Point q = ClosestPointOnSegment(p, _perimeter[i], _perimeter[j]);
                double d2 = (q.X - p.X) * (q.X - p.X) + (q.Y - p.Y) * (q.Y - p.Y);
                if (d2 < best2) { best2 = d2; bestIdx = i; }
            }
            return bestIdx;
        }

        private static Point ClosestPointOnSegment(Point p, Point a, Point b)
        {
            Vector ap = new Vector(p.X - a.X, p.Y - a.Y);
            Vector ab = new Vector(b.X - a.X, b.Y - a.Y);
            double ab2 = ab.X * ab.X + ab.Y * ab.Y;
            if (ab2 <= 1e-12) return a;
            double t = (ap.X * ab.X + ap.Y * ab.Y) / ab2;
            if (t < 0) t = 0;
            else if (t > 1) t = 1;
            return new Point(a.X + ab.X * t, a.Y + ab.Y * t);
        }

        private bool TryFirstIntersectionWithPerimeter(Point p1, Point p2, out Point hit, out int segIndex)
        {
            int n = _perimeter.Count;
            hit = new Point();
            segIndex = -1;

            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                Point h;
                if (SegmentsIntersect(p1, p2, _perimeter[i], _perimeter[j], out h))
                {
                    hit = h;
                    segIndex = i;
                    return true;
                }
            }
            return false;
        }

        private bool IsDetectedAlongStep(Point start, Point end, double tStart)
        {
            if (_detector == null) return false;

            const int N = 5;
            for (int k = 0; k < N; k++)
            {
                double a = (double)k / (double)(N - 1);
                var p = new Point(start.X + (end.X - start.X) * a,
                                  start.Y + (end.Y - start.Y) * a);
                double tk = tStart + DtSeconds * a;
                if (_detector.IsDetected(p, tk)) return true;
            }
            return false;
        }

        private static bool SegmentsIntersect(Point p, Point p2, Point q, Point q2, out Point intersection)
        {
            intersection = new Point();

            double s1_x = p2.X - p.X;
            double s1_y = p2.Y - p.Y;
            double s2_x = q2.X - q.X;
            double s2_y = q2.Y - q.Y;

            double denom = (-s2_x * s1_y + s1_x * s2_y);
            if (Math.Abs(denom) < 1e-12) return false;

            double s = (-s1_y * (p.X - q.X) + s1_x * (p.Y - q.Y)) / denom;
            double t = (s2_x * (p.Y - q.Y) - s2_y * (p.X - q.X)) / denom;

            if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
            {
                intersection = new Point(p.X + (t * s1_x), p.Y + (t * s1_y));
                return true;
            }
            return false;
        }

        private static List<double> BuildCumulativeLengths(List<Point> poly, out double total)
        {
            var cum = new List<double>();
            total = 0.0;
            int n = poly.Count;
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                double d = Distance(poly[i], poly[j]);
                total += d;
                cum.Add(total);
            }
            return cum;
        }

        private Point SamplePerimeterPoint(double s)
        {
            int seg;
            Point a, b;
            GetSegmentAtLength(s, out seg, out a, out b);

            double prevCum = (seg == 0) ? 0.0 : _cumLen[seg - 1];
            double segLen = Distance(a, b);
            double t = (segLen <= 1e-12) ? 0.0 : (s - prevCum) / segLen;
            return new Point(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t);
        }

        private void GetSegmentAtLength(double s, out int segIndex, out Point a, out Point b)
        {
            double target = s;
            int n = _perimeter.Count;
            int idx = 0;
            for (; idx < n; idx++)
            {
                if (_cumLen[idx] >= target) break;
            }
            if (idx >= n) idx = n - 1;
            segIndex = idx;
            a = _perimeter[idx];
            b = _perimeter[(idx + 1) % n];
        }

        private static double Distance(Point p1, Point p2)
        {
            double dx = p2.X - p1.X;
            double dy = p2.Y - p1.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        private static Point ComputeCentroid(List<Point> poly)
        {
            double sx = 0, sy = 0;
            int n = poly.Count;
            for (int i = 0; i < n; i++) { sx += poly[i].X; sy += poly[i].Y; }
            return new Point(sx / n, sy / n);
        }

        private static double Clamp(double v, double lo, double hi) { if (v < lo) return lo; if (v > hi) return hi; return v; }
        private static double Lerp(double a, double b, double t) { return a + (b - a) * t; }
    }
}