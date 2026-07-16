#ifndef MODEL_AERO_H
#define MODEL_AERO_H

/// \cond
// C headers
// C++ headers

// 3rd party headers
/// \endcond

namespace model
{

/**
 * @brief One part's Barrowman contribution, referenced to a shared rocket reference area so the
 *        pieces are directly additive. Stores the CNalpha-weighted moment (cnAlphaXcp) rather than a
 *        raw x_cp, so composing CP is field addition and a zero-lift body (CNalpha == 0) drops out of
 *        the weighted average with no special case.
 *
 * Datum: x_cp is measured from the part's own CM along +z. The composite walk
 * (Part::getCompositeAero) re-expresses every part onto the root part's CM -- the same datum as
 * Part::getCompositeCm -- so composite cp() and cg() are comparable (static margin = cp() - cg()).
 * The per-part Barrowman x_cp formulas (cone CP 2/3 L from the tip, fin CP from the root LE) are
 * converted to this CM datum inside each getAero override.
 *
 * Roll/pitch/yaw moment coefficients (Cl/Cm/Cn) are not modeled here; they derive from CNalpha and
 * the CP-CG lever rather than being stored per part.
 */
struct AeroComponent
{
    double cnAlpha{0.0};      ///< normal-force-coeff slope (per rad), referenced to the shared refArea
    double cnAlphaXcp{0.0};   ///< cnAlpha * x_cp (axial CP station from the part's CM, m) -- the weighted moment
    double cd{0.0};           ///< this part's drag-coeff contribution, already normalized to refArea
};

/**
 * @brief The assembled whole-rocket aero profile. cnAlpha and cnAlphaXcp add over parts; cd adds.
 *        cp() is the CNalpha-weighted CP; cpValid guards the body-only case where cnAlpha == 0.
 */
struct AeroProfile
{
    double cnAlpha{0.0};
    double cnAlphaXcp{0.0};
    double cd{0.0};
    double refArea{0.0};
    bool   cpValid{false};      ///< false when cnAlpha == 0 (CP undefined; e.g. body tube only)

    double cp() const { return cpValid ? cnAlphaXcp / cnAlpha : 0.0; } ///< axial CP from the root CM (m)

    AeroProfile& operator+=(const AeroComponent& c)
    {
        cnAlpha += c.cnAlpha;
        cnAlphaXcp += c.cnAlphaXcp;
        cd += c.cd;
        cpValid = (cnAlpha != 0.0);
        return *this;
    }
};

/// @brief Back-compat alias: Propagatable still holds a default-constructed, unread `Aero aeroData`
///        member. Keeping the name complete and default-constructible lets that member stay untouched
///        while the seam itself is the AeroComponent/AeroProfile value types.
using Aero = AeroProfile;

} // namespace model

#endif // MODEL_AERO_H
