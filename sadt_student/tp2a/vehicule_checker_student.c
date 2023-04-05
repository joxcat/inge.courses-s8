#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define FRAME_LIGHTS_ID 0x80000123
#define FRAME_MOVE_ID 0x80000321
#define FRAME_MOTOR_SPEED_ID 0x80000C06
#define FRAME_VEHICLE_SPEED_ID 0x80000C07

#define FRAME_VISION_FULL_LEFT_ID 0x80000C00
#define FRAME_VISION_LEFT_ID 0x80000C01
#define FRAME_VISION_MIDDLE_LEFT_ID 0x80000C02
#define FRAME_VISION_MIDDLE_RIGHT_ID 0x80000C03
#define FRAME_VISION_RIGHT_ID 0x80000C04
#define FRAME_VISION_FULL_RIGHT_ID 0x80000C05

int can_socket;

void ctrlc_handler(int signum){
    printf("gracefull shutdown\r\n");

    /* Closing the CAN socket */
	if (close(can_socket) < 0) {
		perror("close");
		exit(EXIT_FAILURE);
	}

    exit(EXIT_SUCCESS);
}

/* Frames ID 2123 */
void frame_lights_reset(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x00, 0x00 };
    memset(frame->data, 8, 0x00);
    memcpy(frame->data, data, 2);
}
void frame_beams_short(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x01, 0x00 };
    memset(frame->data, 8, 0x00);
    memcpy(frame->data, data, 2);
}
void frame_beams_long(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x02, 0x00 };
    memset(frame->data, 8, 0x00);
    memcpy(frame->data, data, 2);
}
void frame_blinkers_right(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x00, 0x01 };
    memset(frame->data, 8, 0x00);
    memcpy(frame->data, data, 2);
}
void frame_blinkers_left(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->can_dlc = 2;
    char data[] = { 0x00, 0x02 };
    memset(frame->data, 8, 0x00);
    memcpy(frame->data, data, 2);
}

/* Frames ID 3321 */
// steering -100 => right
// steering 100 => left 
int move(struct can_frame *frame, int throttle, int brake, int steering) {
    if (throttle > 100 || throttle < 0) {
        perror("[error] move: throttle must be >=0 or <= 100");
        return EXIT_FAILURE;
    }
    if (brake > 100 || brake < 0) {
        perror("[error] move: brake must be >=0 or <= 100");
        return EXIT_FAILURE;
    }
    if (steering < -100 || steering > 100) {
        perror("[error] move: steering must be >=-100 or <= 100");
        return EXIT_FAILURE;
    }

    frame->can_id = FRAME_MOVE_ID;
    frame->can_dlc = 3;
    char data[] = { (char)throttle, (char)brake, (signed char)steering };
    memset(frame->data, 8, 0x00);
    memcpy(frame->data, data, 3);

    return EXIT_SUCCESS;
}
int steer_left(float percent) {
    return 100 * (percent / 100);
}
int steer_right(float percent) {
    return -100 * (percent / 100);
}

/* Decoding frames */
int16_t get_motor_speed(struct can_frame *frame) {
    return (frame->data[0] << 8) + frame->data[1];
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
void print_vision(struct vision_frame *frame) {
    printf("In the ");
    switch (frame->position)
    {
    case FRAME_VISION_FULL_LEFT_ID:
        printf("full left");
        break;
    case FRAME_VISION_LEFT_ID:
        printf("left");
        break;
    case FRAME_VISION_MIDDLE_LEFT_ID:
        printf("middle left");
        break;
    case FRAME_VISION_MIDDLE_RIGHT_ID:
        printf("middle right");
        break;
    case FRAME_VISION_RIGHT_ID:
        printf("right");
        break;
    case FRAME_VISION_FULL_RIGHT_ID:
        printf("full right");
        break;
    }
    printf(" (ID 0x%03X), the vehicle sees:\r\n", frame->position);
    printf("- the road: 0x%02X (%d)\r\n", frame->road, frame->road);
    printf("- yield marking: 0x%02X (%d)\r\n", frame->yield, frame->yield);
    printf("- road crossing: 0x%02X (%d)\r\n", frame->crossing, frame->crossing);
    printf("- stop sign: 0x%02X (%d)\r\n", frame->stop, frame->stop);
    printf("- car park: 0x%02X (%d)\r\n", frame->car_park, frame->car_park);
}

/* CAN interactions */
int send_frame(struct can_frame *frame) {
    /* Writing the frame to the CAN */
    if (write(can_socket, frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("write failed");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
int read_frame(struct can_frame *frame) {
    /* Reading the frame to the CAN */
    int nbytes = read(can_socket, frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("read failed");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/* Helpers */
int are_all_frames_checked(canid_t frames[], int len) {
    for (int i = 0; i < len; i++) {
        if (frames[i] != 0) return 0;
    }

    return 1;
}

/* Program */
int rough_simple_program(struct can_frame *frame) {
    frame_blinkers_right(frame);
    send_frame(frame);
    sleep(1);

    frame_blinkers_left(frame);
    send_frame(frame);
    sleep(1);

    frame_beams_short(frame);
    send_frame(frame);
    sleep(1);

    frame_beams_long(frame);
    send_frame(frame);
    sleep(1);

    frame_lights_reset(frame);
    send_frame(frame);
    sleep(1);

    move(frame, 100, 0, steer_left(9));
    send_frame(frame);
    sleep(8);

    move(frame, 100, 0, steer_left(3));
    send_frame(frame);
    sleep(1);

    move(frame, 100, 0, 0);
    send_frame(frame);
    sleep(3);
    
    move(frame, 0, 100, 0);
    send_frame(frame);
    sleep(3);

    move(frame, 0, 0, 0);
    send_frame(frame);
}

/* Frame processing */
int frame_processing(struct can_frame *frame) {
    switch (frame->can_id) {
        case FRAME_VISION_FULL_LEFT_ID:
        case FRAME_VISION_LEFT_ID:
        case FRAME_VISION_MIDDLE_LEFT_ID:
        case FRAME_VISION_MIDDLE_RIGHT_ID:
        case FRAME_VISION_RIGHT_ID:
        case FRAME_VISION_FULL_RIGHT_ID:
            struct vision_frame vframe = get_vision(frame);
            //print_vision(&vframe);
            break;
        default:
            printf("unhandled frame ID 0x%03X\r\n", frame->can_id);
            break;
    }
}

int main(int argc, char **argv)
{
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

    /* Creating a CAN socket */
	if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("failed to create socket");
		return EXIT_FAILURE;
	}

    /* Selecting the network device vcan0 */
	strcpy(ifr.ifr_name, "vcan0");
	ioctl(can_socket, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
    /* Setting the CAN Family of the address */
	addr.can_family = AF_CAN;
    /* Linking the CAN address with the network device */
	addr.can_ifindex = ifr.ifr_ifindex;

    /* Binding the socket to the vcan0 address */
	if (bind(can_socket, (struct sockaddr *) & addr, sizeof(addr)) < 0) {
		perror("failed to bind the socket");
		return EXIT_FAILURE;
	}

    /* CTRL+C handler */
    signal(SIGINT, ctrlc_handler); 

    /* Check must see frames */
    while (!are_all_frames_checked(must_see_frames, must_see_frames_len)) {
        /* Reading a frame from the socket */
        if (read_frame(&frame)) return 1;
        for (int i = 0; i < must_see_frames_len; i++) {
            if (must_see_frames[i] == frame.can_id) {
                must_see_frames[i] = 0;
                continue;
            }
        }
    }
    printf("init sequence received\r\n");

    if (rough_simple_program(&frame)) return EXIT_FAILURE;

	return EXIT_SUCCESS;
}
