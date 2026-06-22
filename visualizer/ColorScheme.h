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

/**
 * @brief A named mapping from a Part's typeName() ("NoseCone", "BodyTube", "FinSet",
 *        "HollowSphere", ...) to the color used to render it, plus a viewport background.
 *
 * The visualizer colors each rendered component purely by its part type, so a scheme is just a
 * small lookup table with a fallback. Unknown / unmapped types render in @ref defaultColor. The
 * user can switch between the built-in @ref presets or override any individual type color at
 * runtime (the per-type color pickers in the UI call @ref setColor).
 */
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
