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
   Stage.cpp
   Stage.h
   ThrustCurve.cpp
   ThrustCurve.h)

target_link_libraries(model PRIVATE
   utils)
