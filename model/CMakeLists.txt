add_library(model
   MotorModel.cpp
   MotorModel.h
   MotorModelDatabase.cpp
   MotorModelDatabase.h
   Part.cpp
   Part.h
   Propagatable.cpp
   Propagatable.h
   Rocket.cpp
   Rocket.h
   ThrustCurve.cpp
   ThrustCurve.h
   InertiaTensors.h)

target_link_libraries(model PRIVATE
   utils)

# Unit tests
add_subdirectory(tests)
