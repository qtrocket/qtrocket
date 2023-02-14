add_library(sim
   AtmosphericModel.cpp
   GravityModel.cpp
   WindModel.cpp
   USStandardAtmosphere.cpp)

target_include_directories(sim PRIVATE ${PROJECT_SOURCE_DIR}/src)
