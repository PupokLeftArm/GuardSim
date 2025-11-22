using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using System.Windows;

namespace GuardSim
{
    public class SensorsConfig
    {
        public int Intruders { get; set; }
        public List<RadarCfg> Radars { get; set; }
        public List<CameraCfg> Cameras { get; set; }
        public List<DroneCfg> Drones { get; set; }
    }

    public class RadarCfg
    {
        public string SensorId { get; set; }
        public string SensorName { get; set; }
        public double PositionX { get; set; }
        public double PositionY { get; set; }
        public double DetectionRange { get; set; }
        public double NoiseLevel { get; set; }
        public double BeamWidth { get; set; }
    }

    public class CameraCfg
    {
        public string SensorId { get; set; }
        public string SensorName { get; set; }
        public double PositionX { get; set; }
        public double PositionY { get; set; }
        public double DetectionRange { get; set; }
        public double FieldOfView { get; set; }
        public double ViewAngle { get; set; }
        public double NoiseLevel { get; set; }
    }

    public class DroneCfg
    {
        public string SensorId { get; set; }
        public string SensorName { get; set; }
        public double ScanRadius { get; set; }
        public double NoiseLevel { get; set; }
        public List<PointCfg> PatrolRoute { get; set; }
    }

    public class PointCfg
    {
        public double X { get; set; }
        public double Y { get; set; }
    }

    public static class SensorsConfigIO
    {
        public static SensorsConfig Load(string path)
        {
            if (!File.Exists(path)) return null;
            var json = File.ReadAllText(path);
            return JsonConvert.DeserializeObject<SensorsConfig>(json);
        }

        public static void Save(string path, SensorDataSimulator sim, int intrudersIfAny)
        {
            var cfg = new SensorsConfig();
            cfg.Intruders = intrudersIfAny;

            cfg.Radars = new List<RadarCfg>();
            foreach (var r in sim.Radars)
            {
                cfg.Radars.Add(new RadarCfg
                {
                    SensorId = r.SensorId,
                    SensorName = r.SensorName,
                    PositionX = r.Position.X,
                    PositionY = r.Position.Y,
                    DetectionRange = r.DetectionRange,
                    NoiseLevel = r.NoiseLevel,
                    BeamWidth = r.BeamWidth
                });
            }

            cfg.Cameras = new List<CameraCfg>();
            foreach (var c in sim.Cameras)
            {
                cfg.Cameras.Add(new CameraCfg
                {
                    SensorId = c.SensorId,
                    SensorName = c.SensorName,
                    PositionX = c.Position.X,
                    PositionY = c.Position.Y,
                    DetectionRange = c.DetectionRange,
                    FieldOfView = c.FieldOfView,
                    ViewAngle = c.ViewAngle,
                    NoiseLevel = c.NoiseLevel
                });
            }

            cfg.Drones = new List<DroneCfg>();
            foreach (var d in sim.Drones)
            {
                var route = new List<PointCfg>();
                if (d.PatrolRoute != null)
                {
                    for (int i = 0; i < d.PatrolRoute.Count; i++)
                        route.Add(new PointCfg { X = d.PatrolRoute[i].X, Y = d.PatrolRoute[i].Y });
                }
                cfg.Drones.Add(new DroneCfg
                {
                    SensorId = d.SensorId,
                    SensorName = d.SensorName,
                    ScanRadius = d.ScanRadius,
                    NoiseLevel = d.NoiseLevel,
                    PatrolRoute = route
                });
            }

            Directory.CreateDirectory(Path.GetDirectoryName(path));
            var json = JsonConvert.SerializeObject(cfg, Formatting.Indented);
            File.WriteAllText(path, json);
        }
    }
}
