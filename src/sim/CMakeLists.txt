add_library(sim
   AtmosphericModel.cpp
   GravityModel.cpp
   WindModel.cpp
   USStandardAtmosphere.cpp)

target_link_libraries(sim PRIVATE math)
