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
   Integrator.h
   Propagator.cpp
   Propagator.h
   RK4Solver.h
   RK45Solver.h
   SphericalGeoidModel.cpp
   SphericalGeoidModel.h
   SphericalGravityModel.cpp
   SphericalGravityModel.h
   StateData.h
   USStandardAtmosphere.cpp
   USStandardAtmosphere.h
   VacuumAtmosphere.h
   WindModel.cpp
   WindModel.h)
qtrocket_enable_coverage(sim)

target_link_libraries(sim PUBLIC
   utils)

# Unit tests
add_subdirectory(tests)
