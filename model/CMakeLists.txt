add_library(model
   MotorModel.cpp
   MotorModel.h
   Part.cpp
   Part.h
   parts/HollowSphere.cpp
   parts/HollowSphere.h
   parts/Parts.h
   Propagatable.cpp
   Propagatable.h
   RocketModel.cpp
   RocketModel.h
   ThrustCurve.cpp
   ThrustCurve.h
   InertiaTensors.h)

# sim is a real dependency: Propagatable/RocketModel use sim::Aero and
# sim/StateData.h. Together with utils -> model this forms a declared
# static-library cycle, which CMake resolves by repeating the archives.
target_link_libraries(model PUBLIC
   utils
   sim)

# Unit tests
add_subdirectory(tests)
