using System;
using System.Windows;

namespace GuardSim
{
    public sealed class DeterministicDetectionAdapter : IDetectionProvider
    {
        private readonly SensorDataSimulator _s;
        private readonly Random _rnd = new Random(12345);

        public double RadarSweepDegPerSec { get; set; } = 60.0; // скорость разворота луча радара
        public double RadarPd { get; set; } = 0.95; // вероятность обнаружения внутри луча
        public double CameraPd { get; set; } = 0.90;
        public double DronePd { get; set; } = 0.92;

        public DeterministicDetectionAdapter(SensorDataSimulator sensors)
        {
            _s = sensors;
        }

        public bool IsDetected(Point pos, double tSeconds)
        {
            if (_s == null) return false;

            foreach (var r in _s.Radars)
            {
                double dx = pos.X - r.Position.X;
                double dy = pos.Y - r.Position.Y;
                double dist = Math.Sqrt(dx * dx + dy * dy);
                if (dist > r.DetectionRange) continue;

                double az = Math.Atan2(dy, dx) * 180.0 / Math.PI; if (az < 0) az += 360.0;

                double beamCenter = (RadarSweepDegPerSec * tSeconds) % 360.0;
                double half = Math.Max(5.0, r.BeamWidth * 0.5);
                double d = AngleDiff(az, beamCenter);
                if (Math.Abs(d) <= half)
                {
                    double conf = Math.Max(0.05, 1.0 - dist / r.DetectionRange);
                    if (_rnd.NextDouble() < RadarPd * conf) return true;
                }
            }

            foreach (var c in _s.Cameras)
            {
                double dx = pos.X - c.Position.X;
                double dy = pos.Y - c.Position.Y;
                double dist = Math.Sqrt(dx * dx + dy * dy);
                if (dist > c.DetectionRange) continue;

                double az = Math.Atan2(dy, dx) * 180.0 / Math.PI; if (az < 0) az += 360.0;
                double d = AngleDiff(az, c.ViewAngle);
                if (Math.Abs(d) <= c.FieldOfView * 0.5)
                {
                    double conf = Math.Max(0.05, 1.0 - dist / c.DetectionRange);
                    if (_rnd.NextDouble() < CameraPd * conf) return true;
                }
            }

            foreach (var d in _s.Drones)
            {
                if (d.PatrolRoute == null || d.PatrolRoute.Count < 2) continue;

                var (dp, vAngleDeg) = DronePoseAtTime(d, tSeconds);

                double dx = pos.X - dp.X;
                double dy = pos.Y - dp.Y;
                double dist = Math.Sqrt(dx * dx + dy * dy);
                if (dist > d.ScanRadius) continue;

                double az = Math.Atan2(dy, dx) * 180.0 / Math.PI; if (az < 0) az += 360.0;
                double diff = AngleDiff(az, vAngleDeg);
                if (Math.Abs(diff) <= d.FieldOfView * 0.5)
                {
                    double conf = Math.Max(0.05, 1.0 - dist / d.ScanRadius);
                    if (_rnd.NextDouble() < DronePd * conf) return true;
                }
            }

            return false;
        }

        private static (Point pos, double viewAngleDeg) DronePoseAtTime(DroneSensor d, double t)
        {
            double speed = (d.Speed <= 0 ? 50.0 : d.Speed);
            double path = speed * t;

            double total = 0.0;
            var rt = d.PatrolRoute;
            for (int i = 0; i < rt.Count; i++)
            {
                int j = (i + 1) % rt.Count;
                total += Dist(rt[i], rt[j]);
            }
            if (total <= 1e-9) return (rt[0], 0);

            path %= total;

            double acc = 0.0;
            for (int i = 0; i < rt.Count; i++)
            {
                int j = (i + 1) % rt.Count;
                double seg = Dist(rt[i], rt[j]);
                if (acc + seg >= path)
                {
                    double tSeg = (path - acc) / (seg <= 1e-9 ? 1.0 : seg);
                    double x = rt[i].X + (rt[j].X - rt[i].X) * tSeg;
                    double y = rt[i].Y + (rt[j].Y - rt[i].Y) * tSeg;
                    double ang = Math.Atan2(rt[j].Y - rt[i].Y, rt[j].X - rt[i].X) * 180.0 / Math.PI;
                    if (ang < 0) ang += 360.0;
                    return (new Point(x, y), ang);
                }
                acc += seg;
            }

            return (rt[rt.Count - 1], 0);
        }

        private static double Dist(Point a, Point b)
        {
            double dx = b.X - a.X, dy = b.Y - a.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        private static double AngleDiff(double a, double b)
        {
            double d = a - b;
            while (d > 180) d -= 360;
            while (d < -180) d += 360;
            return d;
        }
    }
}