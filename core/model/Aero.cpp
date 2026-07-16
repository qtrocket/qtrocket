#include "model/Aero.h"

// The Barrowman seam (model::AeroComponent / model::AeroProfile) is header-only. This TU stays in the
// build (Propagatable holds a model::Aero member, so the type must stay complete) and is the home for
// any future non-inline aero helper.

namespace model
{
}
