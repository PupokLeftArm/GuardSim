using System;
using System.Globalization;
using System.IO;
using System.Text;
using Newtonsoft.Json;

namespace GuardSim
{
    public static class DetectionDataExporter
    {
        public sealed class ExportBundle
        {
            public SimulationResults Results { get; set; }
            public string JsonPath { get; set; }
            public string CsvPath { get; set; }
        }

        public static ExportBundle ExportAll(string exportDir, SimulationResults res, string baseName = "experiment")
        {
            if (res == null) throw new ArgumentNullException(nameof(res));
            Directory.CreateDirectory(exportDir);

            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string jsonPath = Path.Combine(exportDir, $"{baseName}_{stamp}.json");
            string csvPath = Path.Combine(exportDir, $"{baseName}_{stamp}.csv");

            var json = JsonConvert.SerializeObject(new
            {
                totalIntruders = res.TotalIntruders,
                intercepted = res.Intercepted,
                breached = res.Breached,
                mostVulnerable = new
                {
                    index = res.MostVulnerableSegment?.Index ?? -1,
                    start = new { x = res.MostVulnerableSegment?.Start.X ?? 0, y = res.MostVulnerableSegment?.Start.Y ?? 0 },
                    end = new { x = res.MostVulnerableSegment?.End.X ?? 0, y = res.MostVulnerableSegment?.End.Y ?? 0 },
                    breaches = res.MostVulnerableSegment?.Breaches ?? 0
                },
                breachPoints = res.BreachPoints,
                segments = res.Segments
            }, Formatting.Indented);
            File.WriteAllText(jsonPath, json);

            var sb = new StringBuilder();
            var ci = CultureInfo.InvariantCulture;

            sb.AppendLine("TotalIntruders,Intercepted,Breached");
            sb.AppendLine($"{res.TotalIntruders},{res.Intercepted},{res.Breached}");
            sb.AppendLine();

            sb.AppendLine("MostVulnerable_Index,StartX,StartY,EndX,EndY,Breaches");
            if (res.MostVulnerableSegment != null)
            {
                var s = res.MostVulnerableSegment;
                sb.AppendLine($"{s.Index},{s.Start.X.ToString(ci)},{s.Start.Y.ToString(ci)},{s.End.X.ToString(ci)},{s.End.Y.ToString(ci)},{s.Breaches}");
            }
            else sb.AppendLine("-1,0,0,0,0,0");
            sb.AppendLine();

            sb.AppendLine("BreachPointX,BreachPointY,SegmentIndex");
            foreach (var bp in res.BreachPoints)
                sb.AppendLine($"{bp.X.ToString(ci)},{bp.Y.ToString(ci)},{bp.SegmentIndex}");
            sb.AppendLine();

            sb.AppendLine("SegmentIndex,StartX,StartY,EndX,EndY,Breaches");
            foreach (var seg in res.Segments)
                sb.AppendLine($"{seg.Index},{seg.Start.X.ToString(ci)},{seg.Start.Y.ToString(ci)},{seg.End.X.ToString(ci)},{seg.End.Y.ToString(ci)},{seg.Breaches}");

            File.WriteAllText(csvPath, sb.ToString());

            return new ExportBundle
            {
                Results = res,
                JsonPath = jsonPath,
                CsvPath = csvPath
            };
        }
    }
}