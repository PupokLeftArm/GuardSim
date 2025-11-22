using System.Collections.Generic;

namespace GuardSim
{
    public class Layout
    {
        public string Image { get; set; }
        public Perimeter Perimeter { get; set; }
        public List<Zone> Zones { get; set; }
    }

    public class Perimeter
    {
        public List<double[]> Points { get; set; }
        public string Fill { get; set; }
        public string Stroke { get; set; }
        public double StrokeThickness { get; set; }
    }

    public class Zone
    {
        public string Name { get; set; }
        public double Weight { get; set; }
        public List<double[]> Points { get; set; }
        public string Fill { get; set; }
        public string Stroke { get; set; }
        public double StrokeThickness { get; set; }
    }
}
