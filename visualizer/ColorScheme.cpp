// Corresponding header
#include "visualizer/ColorScheme.h"

namespace viz
{

QColor ColorScheme::colorFor(const QString& typeName) const
{
   const auto it = byType.find(typeName);
   if (it != byType.end())
   {
      return it->second;
   }
   return defaultColor;
}

void ColorScheme::setColor(const QString& typeName, const QColor& color)
{
   byType[typeName] = color;
}

const std::vector<ColorScheme>& presets()
{
   static const std::vector<ColorScheme> schemes = []
   {
      std::vector<ColorScheme> result;

      // "Classic" — the readable default (must be presets().front()).
      {
         ColorScheme classic;
         classic.name = QStringLiteral("Classic");
         classic.byType[QStringLiteral("NoseCone")]     = QColor(0xC0, 0x39, 0x2B);
         classic.byType[QStringLiteral("BodyTube")]     = QColor(0xEC, 0xF0, 0xF1);
         classic.byType[QStringLiteral("FinSet")]       = QColor(0x29, 0x80, 0xB9);
         classic.byType[QStringLiteral("HollowSphere")] = QColor(0x95, 0xA5, 0xA6);
         classic.defaultColor = QColor(0xA0, 0xA0, 0xA0);
         classic.background   = QColor(0x2B, 0x30, 0x3B);
         result.push_back(classic);
      }

      // "Hi-Vis" — high-contrast, saturated colors on a lighter background.
      {
         ColorScheme hiVis;
         hiVis.name = QStringLiteral("Hi-Vis");
         hiVis.byType[QStringLiteral("NoseCone")]     = QColor(0xE6, 0x7E, 0x22); // bright orange
         hiVis.byType[QStringLiteral("BodyTube")]     = QColor(0xFF, 0xFF, 0xFF); // pure white
         hiVis.byType[QStringLiteral("FinSet")]       = QColor(0x14, 0x14, 0x14); // near-black
         hiVis.byType[QStringLiteral("HollowSphere")] = QColor(0xF1, 0xC4, 0x0F); // vivid yellow
         hiVis.defaultColor = QColor(0xE7, 0x4C, 0x3C);                           // strong red
         hiVis.background   = QColor(0x55, 0x5B, 0x66);                           // lighter slate
         result.push_back(hiVis);
      }

      // "Carbon" — dark greys with a single warm accent on a near-black background.
      {
         ColorScheme carbon;
         carbon.name = QStringLiteral("Carbon");
         carbon.byType[QStringLiteral("NoseCone")]     = QColor(0xD3, 0x54, 0x00); // burnt-orange accent
         carbon.byType[QStringLiteral("BodyTube")]     = QColor(0x4D, 0x52, 0x59); // mid graphite
         carbon.byType[QStringLiteral("FinSet")]       = QColor(0x2C, 0x30, 0x36); // dark graphite
         carbon.byType[QStringLiteral("HollowSphere")] = QColor(0x60, 0x66, 0x6E); // light graphite
         carbon.defaultColor = QColor(0x3A, 0x3F, 0x45);                           // neutral dark grey
         carbon.background   = QColor(0x12, 0x14, 0x17);                           // near-black
         result.push_back(carbon);
      }

      // "Pastel" — soft, low-saturation tints on a muted background.
      {
         ColorScheme pastel;
         pastel.name = QStringLiteral("Pastel");
         pastel.byType[QStringLiteral("NoseCone")]     = QColor(0xFF, 0xAD, 0xAD); // soft rose
         pastel.byType[QStringLiteral("BodyTube")]     = QColor(0xFD, 0xFF, 0xB6); // soft cream
         pastel.byType[QStringLiteral("FinSet")]       = QColor(0xA0, 0xC4, 0xFF); // soft blue
         pastel.byType[QStringLiteral("HollowSphere")] = QColor(0xCA, 0xFF, 0xBF); // soft green
         pastel.defaultColor = QColor(0xBD, 0xB2, 0xFF);                           // soft lavender
         pastel.background   = QColor(0x40, 0x44, 0x4D);                           // muted slate
         result.push_back(pastel);
      }

      return result;
   }();

   return schemes;
}

ColorScheme defaultScheme()
{
   return presets().front();
}

} // namespace viz
