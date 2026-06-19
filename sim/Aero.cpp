#include "sim/Aero.h"

// The Barrowman seam (sim::AeroComponent / sim::AeroProfile) is header-only: the value types are
// trivially default-constructible and every operation is inline. This translation unit is kept in
// the build graph deliberately -- model::Propagatable holds a `sim::Aero aeroData` member, so the
// type must stay complete -- and is the home for any future non-inline aero helper.

namespace sim
{
}
