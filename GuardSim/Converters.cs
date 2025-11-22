using System;
using System.Windows.Data;

namespace GuardSim
{
    public class BoolToStatusConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            bool hasDetection = (bool)value;
            return hasDetection ? "⚫ Обнаружено" : "⚫ Нет";
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    public class BoolToActiveConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            bool isActive = (bool)value;
            return isActive ? "🔴 ПРЯМОЙ ЭФИР" : "⚫ Автономный";
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}