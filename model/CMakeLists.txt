add_library(model
   MotorModel.cpp
   MotorModel.h
   MotorModelDatabase.cpp
   MotorModelDatabase.h
   Part.cpp
   Part.h
   Rocket.cpp
   Rocket.h
   ThrustCurve.cpp
   ThrustCurve.h)

target_link_libraries(model PRIVATE
   utils)
