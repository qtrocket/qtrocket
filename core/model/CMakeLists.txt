add_library(model
   MotorModel.cpp
   MotorModel.h
   MotorModelDatabase.cpp
   MotorModelDatabase.h
   parts/Part.cpp
   parts/Part.h
   parts/Placement.cpp
   parts/Placement.h
   parts/HollowSphere.cpp
   parts/HollowSphere.h
   parts/ConicalNoseCone.cpp
   parts/ConicalNoseCone.h
   parts/BodyTube.cpp
   parts/BodyTube.h
   parts/FinSet.cpp
   parts/FinSet.h
   parts/Motor.cpp
   parts/Motor.h
   parts/PartFactory.cpp
   parts/PartFactory.h
   parts/Parts.h
   Propagatable.cpp
   Propagatable.h
   RocketModel.cpp
   RocketModel.h
   DesignSerializer.cpp
   DesignSerializer.h
   RASPLoader.cpp
   RASPLoader.h
   RSEDatabaseLoader.cpp
   RSEDatabaseLoader.h
   ThrustCurve.cpp
   ThrustCurve.h
   ThrustCurveClient.cpp
   ThrustCurveClient.h
   InertiaTensors.cpp
   InertiaTensors.h)
qtrocket_enable_coverage(model)

# headers are included as "model/...", rooted at core/
target_include_directories(model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)

# sim is a real dependency: Propagatable/RocketModel use sim::Aero and
# sim/StateData.h. Boost/jsoncpp serve the motor-database ingest paths
# (RockSim .rse XML and the thrustcurve.org REST API).
target_link_libraries(model PUBLIC
   utils
   sim
   Boost::property_tree
   jsoncpp_static)

# Unit tests
add_subdirectory(tests)
