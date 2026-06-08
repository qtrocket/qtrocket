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

target_link_libraries(model PUBLIC
   utils)

# Unit tests
add_subdirectory(tests)
