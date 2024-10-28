#include "Copter.h"
#include <iostream>

#if MODE_LLC_ENABLED

/*
 * Init and run calls for llc flight mode
 */

// initialise llc controller
bool ModeLLC::init(bool ignore_checks)
{
    // start in angle control mode
    ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeLLC::run()
{
    // run angle controller
    ModeGuided::angle_control_run();
    std::cout << "Running LLC mode" << std::endl;
}

#endif
