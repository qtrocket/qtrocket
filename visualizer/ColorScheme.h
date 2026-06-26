#ifndef VISUALIZER_COLORSCHEME_H
#define VISUALIZER_COLORSCHEME_H

// Qt headers
#include <QColor>
#include <QString>

// C++ headers
#include <map>
#include <vector>

namespace viz
{

/// @brief Named lookup from a Part's typeName() to its render color, plus a viewport background.
///        Components are colored purely by type; unmapped types fall back to @ref defaultColor.
///        Switch @ref presets or override one type at runtime via @ref setColor.
struct ColorScheme
{
   QString                   name;                       ///< human-facing scheme name
   std::map<QString, QColor> byType;                     ///< typeName() -> render color
   QColor                    defaultColor{160, 160, 160};///< fallback for an unmapped part type
   QColor                    background{0x2b, 0x30, 0x3b};///< viewport clear color

   /// @brief Color for part type @p typeName, or @ref defaultColor when the type is not mapped.
   QColor colorFor(const QString& typeName) const;

   /// @brief Set (or override) the render color for part type @p typeName.
   void setColor(const QString& typeName, const QColor& color);
};

/**
 * @brief The built-in color schemes. Guaranteed non-empty; @c presets().front() is the sane
 *        default ("Classic"). The others are user-selectable alternatives (e.g. high-visibility,
 *        dark/carbon, pastel).
 */
const std::vector<ColorScheme>& presets();

/// @brief The default scheme used at startup (== @c presets().front()).
ColorScheme defaultScheme();

} // namespace viz

#endif // VISUALIZER_COLORSCHEME_H
