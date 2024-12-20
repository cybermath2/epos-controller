#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include "constants.h"
#include "main.h"

static int8_t mode_last = 0xFF;

double deg_to_inc(uint16_t node_id, double deg) {
    switch (node_id)
    {
    case NODE_ID_YAW:
        return deg * 4648L;
    case NODE_ID_ROLL:
    case NODE_ID_PITCH:
        return deg * DEGTOINC;
    
    default:
        return 0;
    }
}

void motion_halt(void)
{
	uint32_t err;
	switch (mode_last) {
		case OMD_PROFILE_POSITION_MODE:
			if (!VCS_HaltPositionMovement(port, NODE_ID_YAW, &err) |
			    !VCS_HaltPositionMovement(port, NODE_ID_PITCH, &err) |
			    !VCS_HaltPositionMovement(port, NODE_ID_ROLL, &err)) {
				die(err, "failed to halt profile position movement for one or more axes");
			}
			break;

		case OMD_PROFILE_VELOCITY_MODE:
			if (!VCS_HaltVelocityMovement(port, NODE_ID_YAW, &err) |
			    !VCS_HaltVelocityMovement(port, NODE_ID_PITCH, &err) |
			    !VCS_HaltVelocityMovement(port, NODE_ID_ROLL, &err)) {
				die(err, "failed to halt profile velocity movement for one or more axes");
			}
			break;
	}
}

void motion_position(uint16_t node_id, bool absolute, double deg)
{
	// TODO better safety (also use VCS_SetMaxAcceleration()...)
    brakes(false);
	// TODO move this up into caller

	printf("moving to new position %lf\n", deg);

        uint32_t err;
	mode_last = OMD_PROFILE_POSITION_MODE;

        // TODO only activate if necessary so we don't have to do this every time
	if (!VCS_ActivateProfilePositionMode(port, node_id, &err)) {
		die(err, "failed to activate profile position mode");
	}

        // TODO move into node_configure() so we don't have to do this every time
        if (!VCS_SetPositionProfile(port, node_id, PPM_MAX_VELOCITY, 1000, 1000, &err)) {
                die(err, "failed to set position profile");
        }

        if (!VCS_MoveToPosition(port, node_id, deg_to_inc(node_id, deg), absolute, true, &err)) {
                die(err, "failed to move to position");
        }
}

void motion_velocity(uint16_t node_id, double rpm)
{
    brakes(false);

    uint32_t err;
	mode_last = OMD_PROFILE_VELOCITY_MODE;

        // TODO only activate if necessary so we don't have to do this every time
	if (!VCS_ActivateProfileVelocityMode(port, node_id, &err)) {
		die(err, "failed to activate profile velocity mode");
	}

        // TODO move into node_configure() so we don't have to do this every time
        if (!VCS_SetVelocityProfile(port, node_id, 1000, 1000, &err)) {
                die(err, "failed to set velocity profile");
        }

        if (!VCS_MoveWithVelocity(port, node_id, rpm * RPMTOVEL, &err)) {
                die(err, "failed to move with velocity");
        }

        sleep(5); //uncomment to move for five seconds

        if (!VCS_HaltVelocityMovement(port, node_id, &err)) {
                die(err, "failed to halt velocity movement");
        }
}

void move_to_initial_position(void) {
        float diff;
        if (pos.yaw == 0.0) return;
        do {
                diff = 239.0 - pos.yaw;
                motion_position(NODE_ID_YAW, false, diff);
                sleep(1);
        } while (fabs(diff) > 0.1);
}