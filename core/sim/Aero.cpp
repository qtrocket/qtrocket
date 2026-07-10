#include "sim/Aero.h"

// The Barrowman seam (sim::AeroComponent / sim::AeroProfile) is header-only. This TU stays in the
// build (model::Propagatable holds a sim::Aero member, so the type must stay complete) and is the
// home for any future non-inline aero helper.

namespace sim
{
}
