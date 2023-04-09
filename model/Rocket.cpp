#include "Rocket.h"

Rocket::Rocket() : propagator(this)
{
   propagator.setTimeStep(0.01);
   //propagator.set

}

void Rocket::launch()
{
    propagator.runUntilTerminate();
}
