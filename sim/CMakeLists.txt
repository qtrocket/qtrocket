add_library(sim
   Aero.cpp
   Aero.h
   AtmosphericModel.h
   ConstantAtmosphere.h
   ConstantGravityModel.h
   DESolver.h
   Environment.h
   GeoidModel.h
   GravityModel.h
   Propagator.cpp
   Propagator.h
   RK4Solver.h
   SphericalGeoidModel.cpp
   SphericalGeoidModel.h
   SphericalGravityModel.cpp
   SphericalGravityModel.h
   StateData.h
   USStandardAtmosphere.cpp
   USStandardAtmosphere.h
   WindModel.cpp
   WindModel.h)

target_link_libraries(sim PRIVATE
   utils)

# Unit tests
add_subdirectory(tests)
