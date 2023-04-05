#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define MAX_SPEED 50

#define FRAME_LIGHTS_ID 0x80000123u
#define FRAME_MOVE_ID 0x80000321u
#define FRAME_MOTOR_SPEED_ID 0x80000C06u
#define FRAME_VEHICLE_SPEED_ID 0x80000C07u

#define FRAME_VISION_FULL_LEFT_ID 0x80000C00u
#define FRAME_VISION_LEFT_ID 0x80000C01u
#define FRAME_VISION_MIDDLE_LEFT_ID 0x80000C02u
#define FRAME_VISION_MIDDLE_RIGHT_ID 0x80000C03u
#define FRAME_VISION_RIGHT_ID 0x80000C04u
#define FRAME_VISION_FULL_RIGHT_ID 0x80000C05u

#define ACTION_RIGHT "->"
#define ACTION_FORWARD "^"
#define ACTION_LEFT "<-"

int can_socket;
struct car_state {
    int speed;
    int gear;
    int motor_speed;
    char* action_to_follow_road;
};

int ctrlc_handler(int signum) {
    int result = EXIT_SUCCESS;
    /* Closing the CAN socket */
	if (close(can_socket) < 0) {
		perror("close");
		result = EXIT_FAILURE;
	}

    return result;
}

/* Frames ID 2123 */
int frame_lights_reset(struct can_frame *frame) {
    int result = EXIT_SUCCESS;

    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x00, 0x00 };
    if (memset(frame->data, 8, 0x00) == 1) {
        result = EXIT_FAILURE;
    }
    if (!result && memcpy(frame->data, data, 2) == 1) {
        result = EXIT_FAILURE;
    }

    return result;
}
int frame_beams_short(struct can_frame *frame) {
    int result = EXIT_SUCCESS;

    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x01, 0x00 };
    if (memset(frame->data, 8, 0x00) == 1) {
        result = EXIT_FAILURE;
    }
    if (result || memcpy(frame->data, data, 2) == 1) {
        result = EXIT_FAILURE;
    }

    return result;
}
int frame_beams_long(struct can_frame *frame) {
    int result = EXIT_SUCCESS;

    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x02, 0x00 };
    if (memset(frame->data, 8, 0x00) == 1) {
        result = EXIT_FAILURE;
    }
    if (result || memcpy(frame->data, data, 2) == 1) {
        result = EXIT_FAILURE;
    }

    return result;
}
int frame_blinkers_right(struct can_frame *frame) {
    int result = EXIT_SUCCESS;

    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x00, 0x01 };
    if (memset(frame->data, 8, 0x00) == 1) {
        result = EXIT_FAILURE;
    }
    if (result || memcpy(frame->data, data, 2) == 1) {
        result = EXIT_FAILURE;
    }

    return result;
}
int frame_blinkers_left(struct can_frame *frame) {
    int result = EXIT_SUCCESS;

    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x00, 0x02 };
    if (memset(frame->data, 8, 0x00) == 1) {
        result = EXIT_FAILURE;
    }
    if (result || memcpy(frame->data, data, 2) == 1) {
        result = EXIT_FAILURE;
    }

    return result;
}

/* Frames ID 3321 */
// steering -100 => right
// steering 100 => left 
int move(struct can_frame *frame, int throttle, int brake, int steering) {
    int result = EXIT_SUCCESS;

    if ((throttle > 100) || (throttle < 0)) {
        perror("[error] move: throttle must be >=0 or <= 100");
        result = EXIT_FAILURE;
    }
    if ((brake > 100) || (brake < 0)) {
        perror("[error] move: brake must be >=0 or <= 100");
        result = EXIT_FAILURE;
    }
    if ((steering < -100) || (steering > 100)) {
        perror("[error] move: steering must be >=-100 or <= 100");
        result = EXIT_FAILURE;
    }

    if (!result) {
        frame->can_id = FRAME_MOVE_ID;
        frame->can_dlc = 3;
        char data[] = { (char)throttle, (char)brake, (signed char)steering };

        if (memset(frame->data, 8, 0x00) == 1) {
            result = EXIT_FAILURE;
        }
        if (result || memcpy(frame->data, data, 3) == 1) {
            result = EXIT_FAILURE;
        }
    }

    return result;
}
int steer_left(float percent) {
    return 100.0 * (percent / 100.0);
}
int steer_right(float percent) {
    return -100.0 * (percent / 100.0);
}

/* Decoding frames */
int16_t get_motor_speed(struct can_frame *frame) {
    return (frame->data[1] << 8) + frame->data[0];
}
int8_t get_vehicle_speed(struct can_frame *frame) {
    return frame->data[0];
}
int8_t get_gear_selection(struct can_frame *frame) {
    return frame->data[1];
}

/* Vision frames */
struct vision_frame {
    int position;
    int8_t road;
    int8_t stop;
    int8_t yield;
    int8_t crossing;
    int8_t car_park;
};
struct vision_frame get_vision(struct can_frame *frame) {
    struct vision_frame result;

    result.position = frame->can_id;
    result.road = frame->data[0];
    result.stop = frame->data[1];
    result.yield = frame->data[2];
    result.crossing = frame->data[3];
    result.car_park = frame->data[4];

    return result;
}
int print_vision(struct vision_frame *frame) {
    int result = EXIT_SUCCESS;

    if (printf("In the ") == 1) {
        result = EXIT_FAILURE;
    }
    switch (frame->position) {
        case FRAME_VISION_FULL_LEFT_ID:
            if (result || printf("full left") == 1) {
                result = EXIT_FAILURE;
            }
            break;
        case FRAME_VISION_LEFT_ID:
            if (result || printf("left") == 1) {
                result = EXIT_FAILURE;
            }
            break;
        case FRAME_VISION_MIDDLE_LEFT_ID:
            if (result || printf("middle left") == 1) {
                result = EXIT_FAILURE;
            }
            break;
        case FRAME_VISION_MIDDLE_RIGHT_ID:
            if (result || printf("middle right") == 1) {
                result = EXIT_FAILURE;
            }
            break;
        case FRAME_VISION_RIGHT_ID:
            if (result || printf("right") == 1) {
                result = EXIT_FAILURE;
            }
            break;
        case FRAME_VISION_FULL_RIGHT_ID:
            if (result || printf("full right") == 1) {
                result = EXIT_FAILURE;
            }
            break;
        default:
            break;
    }
    if (result || printf(" (ID 0x%03X), the vehicle sees:\r\n", frame->position) == 1) { 
        result = EXIT_FAILURE;
    }
    if (result || printf("- the road: 0x%02X (%d)\r\n", frame->road, frame->road) == 1) {
        result = EXIT_FAILURE;
    }
    if (result || printf("- yield marking: 0x%02X (%d)\r\n", frame->yield, frame->yield) == 1) {
        result = EXIT_FAILURE;
    }
    if (result || printf("- road crossing: 0x%02X (%d)\r\n", frame->crossing, frame->crossing) == 1) {
        result = EXIT_FAILURE;
    }
    if (result || printf("- stop sign: 0x%02X (%d)\r\n", frame->stop, frame->stop) == 1) {
        result = EXIT_FAILURE;
    }
    if (result || printf("- car park: 0x%02X (%d)\r\n", frame->car_park, frame->car_park) == 1) {
        result = EXIT_FAILURE;
    }

    return result;
}

/* CAN interactions */
int send_frame(struct can_frame *frame) {
    int result = EXIT_SUCCESS;
    /* Writing the frame to the CAN */
    if (write(can_socket, frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("write failed");
        result = EXIT_FAILURE;
    }

    return result;
}
int read_frame(struct can_frame *frame) {
    int result = EXIT_SUCCESS;

    /* Reading the frame to the CAN */
    int nbytes = read(can_socket, frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("read failed");
        result = EXIT_FAILURE;
    }

    return result;
}

/* Helpers */
int are_all_frames_checked(canid_t frames[], int len) {
    int result = 1;
    for (int i = 0; i < len; i++) {
        if (frames[i] != 0) {
            result = 0;
            break;
        }
    }

    return result;
}

int main(int argc, char **argv)
{
    int result = EXIT_SUCCESS;
    int nbytes;
	struct sockaddr_can addr;
    
	struct ifreq ifr;
	struct can_frame frame;
    canid_t must_see_frames[] = { 
        FRAME_VISION_FULL_LEFT_ID, 
        FRAME_VISION_LEFT_ID, 
        FRAME_VISION_MIDDLE_LEFT_ID,
        FRAME_VISION_MIDDLE_RIGHT_ID,
        FRAME_VISION_RIGHT_ID,
        FRAME_VISION_FULL_RIGHT_ID,
        FRAME_MOTOR_SPEED_ID,
        FRAME_VEHICLE_SPEED_ID,
    };
    int must_see_frames_len = sizeof(must_see_frames) / sizeof(canid_t);

    struct car_state car;
    car.motor_speed = 0;
    car.gear = 0;
    car.speed = 0;
    car.action_to_follow_road = NULL;

    struct vision_frame vision_state[6];

    /* Creating a CAN socket */
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (can_socket < 0) {
		perror("failed to create socket");
		result = EXIT_FAILURE;
	} else {
        /* Selecting the network device vcan0 */
        if (strcpy(ifr.ifr_name, "vcan0") == NULL) { result = EXIT_FAILURE; } 
        else if (ioctl(can_socket, SIOCGIFINDEX, &ifr) == 1) { result = EXIT_FAILURE; }
        else if (memset(&addr, 0, sizeof(addr)) == 1) { result = EXIT_FAILURE; }
        else {
            /* Setting the CAN Family of the address */
            addr.can_family = AF_CAN;
            /* Linking the CAN address with the network device */
            addr.can_ifindex = ifr.ifr_ifindex;

            /* Binding the socket to the vcan0 address */
            if (bind(can_socket, (struct sockaddr *) & addr, sizeof(addr)) < 0) {
                perror("failed to bind the socket");
                result = EXIT_FAILURE;
            } else {
                /* CTRL+C handler */
                signal(SIGINT, ctrlc_handler); 

                while (1) {
                    if (read_frame(&frame) == 1) {
                        result = EXIT_FAILURE;
                    } else {
                        struct vision_frame vframe = get_vision(&frame);
                        switch (frame.can_id) {
                            case FRAME_VISION_FULL_LEFT_ID:
                                vision_state[0] = get_vision(&frame);
                                break;
                            case FRAME_VISION_LEFT_ID:
                                vision_state[1] = get_vision(&frame);
                                break;
                            case FRAME_VISION_MIDDLE_LEFT_ID:
                                vision_state[2] = get_vision(&frame);
                                break;
                            case FRAME_VISION_MIDDLE_RIGHT_ID:
                                vision_state[3] = get_vision(&frame);
                                break;
                            case FRAME_VISION_RIGHT_ID:
                                vision_state[4] = get_vision(&frame);
                                break;
                            case FRAME_VISION_FULL_RIGHT_ID:
                                vision_state[5] = get_vision(&frame);
                                break;
                            case FRAME_MOTOR_SPEED_ID:
                                car.motor_speed = get_motor_speed(&frame);
                                break;
                            case FRAME_VEHICLE_SPEED_ID:
                                car.speed = get_vehicle_speed(&frame);
                                car.gear = get_gear_selection(&frame);
                                break;
                            default:
                                break;
                        }

                        if (are_all_frames_checked(must_see_frames, must_see_frames_len) == 1) {
                            int speed = MAX_SPEED;
                            int brake = 0;
                            if (car.speed > (MAX_SPEED*0.95)) {
                                speed = 0;
                                brake = 5;
                            }
                            // Calculate car action to follow road
                            // 300 + 200 => 500
                            float left_road = ((vision_state[0].road * 3) + (vision_state[1].road * 2)) / 5;
                            float right_road = ((vision_state[5].road * 3) + (vision_state[4].road * 2)) / 5;

                            // NOTE: We compare the two center visions states to see if we must follow the road
                            if (abs(vision_state[2].road - vision_state[3].road) < 10) {
                                car.action_to_follow_road = &ACTION_FORWARD;
                            } else if (left_road < right_road) {
                                car.action_to_follow_road = &ACTION_RIGHT;
                            } else {
                                car.action_to_follow_road = &ACTION_LEFT;
                            }
                            if (move(&frame, speed, brake, left_road - right_road) == 1) {
                                result = EXIT_FAILURE;
                            }
                            else if (send_frame(&frame) == 1) {
                                result = EXIT_FAILURE;
                            }
                            else if (fflush(0) == 1) {
                                result = EXIT_FAILURE;
                            } else {
                                // MISRA FUN
                            }
                        } else {
                            for (int i = 0; i < must_see_frames_len; i++) {
                                if (must_see_frames[i] == frame.can_id) {
                                    must_see_frames[i] = 0;
                                    continue;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

	return result;
}
