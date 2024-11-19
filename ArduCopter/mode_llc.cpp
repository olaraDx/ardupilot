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

    mavlink_message_t msg;
    mavlink_status_t status;

    while(hal.serial(0)->available())
    {
        uint8_t c = hal.serial(0)->read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
            // Handle message
            // handle_mavlink_message(&msg);
            std::cout << "Handling mavlink message" << std::endl;
            std::cout << "Message ID: " << msg.msgid << std::endl;
            std::cout << "Status: " << status.parse_state << std::endl;

            // Decode message
            switch (msg.msgid)
            {
                case MAV_CMD_NAV_VTOL_TAKEOFF:
                {
                    // decode
                    mavlink_command_long_t command_long;
                    mavlink_msg_command_long_decode(&msg, &command_long);

                    // print
                    std::cout << "Command long message received" << std::endl;
                    std::cout << "Command: " << command_long.command << std::endl;
                    std::cout << "Confirmation: " << command_long.confirmation << std::endl;
                    std::cout << "Param1: " << command_long.param1 << std::endl;
                    std::cout << "Param2: " << command_long.param2 << std::endl;
                    std::cout << "Param3: " << command_long.param3 << std::endl;
                    std::cout << "Param4: " << command_long.param4 << std::endl;
                    std::cout << "Param5: " << command_long.param5 << std::endl;
                    std::cout << "Param6: " << command_long.param6 << std::endl;
                    std::cout << "Param7: " << command_long.param7 << std::endl;

                    Vector3f u_d = {command_long.param3, command_long.param2, command_long.param4};
                    Vector3f u_d_dot = {command_long.param6, command_long.param5, command_long.param7};

                    attitude_control->llc_set_virtual_ctrl(u_d, u_d_dot);

                    break;
                }
                default:
                    std::cout << "Unknown message ID" << std::endl;
                    break;
            }

        }
    }
}

#endif
