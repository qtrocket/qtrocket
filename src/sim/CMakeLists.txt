add_library(sim
   AtmosphericModel.cpp
   GravityModel.cpp
   WindModel.cpp
   USStandardAtmosphere.cpp
   SphericalGravityModel.cpp
   SphericalGeoidModel.cpp)

target_link_libraries(sim PRIVATE math utils)
