#ifndef STATEDATA_H
#define STATEDATA_H

/// \cond
// C headers
// C++ headers
// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/math/MathTypes.h"

/// Physical state at one instant: CM position/velocity, orientation and its rate, plus the composite
/// mass properties recorded each step.
class StateData
{
public:
    StateData() {}
    ~StateData() {}

    StateData(const StateData&) = default;
    StateData(StateData&&) = default;

    // Defaulted memberwise: every member is an Eigen value type, so memberwise copy is correct and
    // a new member can't be silently dropped (as an explicit field-list assignment could).
    StateData& operator=(const StateData&) = default;
    StateData& operator=(StateData&&) = default;

/// TODO: Put these behind an interface
    //Vector3 getPosition() const
    //{
    //   return position;
    //}

    //Vector3 getVelocity() const
    //{
    //   return velocity;
    //}
// private:

    // Intended to be used as world state data
    Vector3 position{0.0, 0.0, 0.0};
    Vector3 velocity{0.0, 0.0, 0.0};

    // Orientation of body coordinates w.r.t. world coordinates
    Quaternion orientation{0.0, 0.0, 0.0, 0.0}; /// (vector, scalar)
    Quaternion orientationRate{0.0, 0.0, 0.0, 0.0}; /// (vector, scalar)

    Matrix3 dcm{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    /// Euler angles are yaw-pitch-roll, and (3-2-1) order
    /// yaw   - psi
    /// pitch - theta
    /// roll  - phi
    Vector3 eulerAngles{0.0, 0.0, 0.0};

    // Composite mass properties at this sample's time, recorded each step via
    // Propagatable::writeMassProperties. Not integrated in 3-DOF; they make CG(t)/I(t) observable and
    // are the seam the 6-DOF rotational ODE will read.
    double  mass{0.0};                 ///< composite mass at this time (kg)
    Vector3 cg{0.0, 0.0, 0.0};         ///< composite center of mass (== CG), body frame
    Matrix3 inertia{Matrix3::Zero()};  ///< full composite inertia tensor (kg*m^2) about cg

};

#endif // STATEDATA_H
