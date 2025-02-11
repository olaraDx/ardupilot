#include "Copter.h"
// #include <iostream>

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
}

bool ModeLLC::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_FORCE_VECTOR_TARGET: {
            // Verificar que el mensaje es para este sistema
            mavlink_force_vector_target_t packet;
            mavlink_msg_force_vector_target_decode(&msg, &packet);
            
            if (packet.target_system != g.sysid_this_mav) {
                break;
            }
            
            // Actualizar el vector de fuerza
            _force_target.x = packet.force_x;
            _force_target.y = packet.force_y;
            _force_target.z = packet.force_z;
            _force_target_derivative.x = packet.force_derivative_x;
            _force_target_derivative.y = packet.force_derivative_y;
            _force_target_derivative.z = packet.force_derivative_z;

            attitude_control->llc_set_virtual_ctrl(_force_target, _force_target_derivative);
            // std::cout << "Force vector target: " << _force_target.x << ", " << _force_target.y << ", " << _force_target.z << std::endl; 
            // std::cout << "Force vector target derivative: " << _force_target_derivative.x << ", " << _force_target_derivative.y << ", " << _force_target_derivative.z << std::endl;

            _have_new_force_target = true;
            _last_force_target_ms = AP_HAL::millis();
            
            return true;
        }
    }
    return false;
}
#endif
