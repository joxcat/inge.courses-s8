#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/poll.h>

#include <linux/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1
#define EXIT_TIMEOUT 2

#define REQUEST_TIMEOUT 250

#define MAX_SPEED 50

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

#define FRAME_OBD2_REQUEST 0x700000DF
#define FRAME_OBD2_RESPONSE 0x700000E8
#define OBD2_ENGINE_SPEED_PID 0x0C
#define OBD2_VEHICLE_SPEED_PID 0x0D
#define OBD2_THROTTLE_POSITION_PID 0x11
#define OBD2_SHOW_CURRENT_DATA_REQ_SERVICE 0x01
#define OBD2_REQ_TO_RES_SERVICE 0x40

#define ACTION_RIGHT "->"
#define ACTION_FORWARD "^"
#define ACTION_LEFT "<-"

#define CLEAR_STDOUT "\e[1;1H\e[2J"

int sensor_can_socket;
int request_can_socket;

struct car_state {
    int speed;
    int gear;
    int motor_speed;

    int throttle_percent;
    
    char* action_to_follow_road;
};

void ctrlc_handler(int signum){
    printf("gracefull shutdown %d\r\n", signum);

    /* Closing the CAN socket */
	if (close(sensor_can_socket) < 0) {
		perror("[error] failed to close sensor can");
		exit(EXIT_FAILURE);
	}

    if (close(request_can_socket) < 0) {
        perror("[error] failed to close request can");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}

/* Frames ID 2123 */
void frame_lights_reset(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->len = 2;
    char data[] = { 0x00, 0x00 };
    memset(frame->data, 0x00, 8);
    memcpy(frame->data, data, 2);
}
void frame_beams_short(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->len = 2;
    char data[] = { 0x01, 0x00 };
    memset(frame->data, 0x00, 8);
    memcpy(frame->data, data, 2);
}
void frame_beams_long(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->len = 2;
    char data[] = { 0x02, 0x00 };
    memset(frame->data, 0x00, 8);
    memcpy(frame->data, data, 2);
}
void frame_blinkers_right(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->len = 2;
    char data[] = { 0x00, 0x01 };
    memset(frame->data, 0x00, 8);
    memcpy(frame->data, data, 2);
}
void frame_blinkers_left(struct can_frame *frame) {
    frame->can_id = FRAME_LIGHTS_ID;
    frame->len = 2;
    char data[] = { 0x00, 0x02 };
    memset(frame->data, 0x00, 8);
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
    frame->len = 3;
    char data[] = { (char)throttle, (char)brake, (signed char)steering };
    memset(frame->data, 0x00, 8);
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
    return (frame->data[1] << 8) + frame->data[0];
}
int8_t get_vehicle_speed(struct can_frame *frame) {
    return frame->data[0];
}
int8_t get_gear_selection(struct can_frame *frame) {
    return frame->data[1];
}
int8_t get_char_throttle(struct can_frame *frame) {
    return frame->data[0];
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

/* OBD2 frames */
struct obd2_request {
    int8_t count_additional_data_bytes;
    int8_t service;
    int8_t pid;
};
struct obd2_response {
    int8_t count_additional_data_bytes;
    int8_t service;
    int8_t pid;
    int8_t values[4];
};
int decode_obd2_reqframe(struct can_frame *frame, struct obd2_request *req) {
    req->count_additional_data_bytes = frame->data[0];
    req->service = frame->data[1];
    req->pid = frame->data[2];

    return EXIT_SUCCESS;
}
int encode_obd2_respframe(struct can_frame *frame, struct obd2_response *resp) {
    memset(frame->data, 0x00, 8);

    frame->can_id = FRAME_OBD2_RESPONSE;
    frame->len = 7;
    frame->data[0] = resp->count_additional_data_bytes;
    frame->data[1] = resp->service;
    frame->data[2] = resp->pid;
    memcpy(&(frame->data[3]), resp->values, 4);

    return EXIT_SUCCESS;
}

/* CAN interactions */
int send_frame(int fd, struct can_frame *frame) {
    printf("Sending 0x%02X on %d\n", frame->can_id, fd);
    /* Writing the frame to the CAN */
    if (write(fd, frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("[error] write failed to the can_socket");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
int read_frame(int socket, struct can_frame *frame, int timeout) {
    frame->can_id = 0x00;
    memset(frame->data, 0x00, 8);

    if (timeout > 0) {
        struct pollfd pfds[1];
        pfds[0].fd = socket;
        pfds[0].events = POLLIN;
        int ret = poll(pfds, 1, timeout);

        if (ret == -1) {
            perror("[error] failed to poll can_socket");
            return EXIT_FAILURE;
        }

        // `pfds[0].revents & POLLIN` means that we can read something
        if (!ret && !(pfds[0].revents & POLLIN)) {
            perror("warning: timed out waiting for can_socket");
            return EXIT_TIMEOUT;
        }
    }

    /* Reading the frame to the CAN (blocking) */
    int nbytes = read(socket, frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        perror("[error] failed to read can_socket");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
int read_frame_blocking(int socket, struct can_frame *frame) {
    return read_frame(socket, frame, 0);
}

int main(void)
{
    int nbytes;
	struct sockaddr_can sensor_addr;
	struct sockaddr_can request_addr;
    
	struct ifreq sensor_ifr;
	struct ifreq request_ifr;
	struct can_frame frame;
    struct obd2_request obd2req;
    struct obd2_response obd2resp;

    struct car_state car;
    car.motor_speed = 0;
    car.gear = 0;
    car.speed = 0;
    car.action_to_follow_road = 0;
    car.throttle_percent = 0;

    struct vision_frame vision_state[6];

    /* Creating a CAN socket */
    sensor_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (sensor_can_socket < 0) {
		perror("[error] failed to create sensor socket");
		return EXIT_FAILURE;
	}
    request_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (request_can_socket < 0) {
        perror("[error] failed to create request socket");
        return EXIT_FAILURE;
    }

    /* Selecting the network device vcan0 & vcan1 */
	strcpy(sensor_ifr.ifr_name, "vcan0");
    strcpy(request_ifr.ifr_name, "vcan1");
	ioctl(sensor_can_socket, SIOCGIFINDEX, &sensor_ifr);
    ioctl(request_can_socket, SIOCGIFINDEX, &request_ifr);

	memset(&sensor_addr, 0, sizeof(sensor_addr));
    memset(&request_addr, 0, sizeof(request_addr));

    /* Setting the CAN Family of the address */
	sensor_addr.can_family = AF_CAN;
    request_addr.can_family = AF_CAN;
    /* Linking the CAN address with the network device */
	sensor_addr.can_ifindex = sensor_ifr.ifr_ifindex;
	request_addr.can_ifindex = request_ifr.ifr_ifindex;

    /* Binding the socket to the vcan0 & vcan1 address */
	if (bind(sensor_can_socket, (struct sockaddr *) & sensor_addr, sizeof(sensor_addr)) < 0) {
		perror("[error] failed to bind the sensor socket");
		return EXIT_FAILURE;
	}
    if (bind(request_can_socket, (struct sockaddr *) & request_addr, sizeof(request_addr)) < 0) {
		perror("[error] failed to bind the request socket");
		return EXIT_FAILURE;
	}

    /* CTRL+C handler */
    signal(SIGINT, ctrlc_handler); 

    while (1) {
        // HANDLE INFOS
        if (read_frame_blocking(sensor_can_socket, &frame)) return EXIT_FAILURE;

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
                printf("Updated state.motor_speed to %d\n", car.motor_speed);
                break;
            case FRAME_VEHICLE_SPEED_ID:
                car.speed = get_vehicle_speed(&frame);
                car.gear = get_gear_selection(&frame);
                printf("Updated state.car_speed to %d\n", car.speed);
                printf("Updated state.gear to %d\n", car.gear);
                break;
            case FRAME_MOVE_ID:
                car.throttle_percent = get_char_throttle(&frame);
                printf("Updated state.throttle to %d\n", car.throttle_percent);
                break;
            default:
                fprintf(stderr, "[warning] request handler: unhandled can_id 0x%03X\n", frame.can_id);
        }

        // HANDLE REQUESTS
        int res = read_frame_blocking(request_can_socket, &frame);
        if (res == EXIT_FAILURE) return EXIT_FAILURE;
        else if (res == EXIT_TIMEOUT) continue;
        else {
            if (decode_obd2_reqframe(&frame, &obd2req)) return EXIT_FAILURE;
            if (obd2req.service != OBD2_SHOW_CURRENT_DATA_REQ_SERVICE) {
                printf("\n%d %x\n",res, frame.data);
                fprintf(stderr, "[warning] obd2 handler: unhandled service 0x%02X\n", obd2req.service);
                continue;
            }

            // NOTE: Here it will always be a obd2 request
            int frame_to_send = 0;
            obd2resp.service = OBD2_SHOW_CURRENT_DATA_REQ_SERVICE + OBD2_REQ_TO_RES_SERVICE;
            switch (obd2req.pid) {
                case OBD2_ENGINE_SPEED_PID:
                    obd2resp.count_additional_data_bytes = 2;
                    obd2resp.pid = OBD2_ENGINE_SPEED_PID;
                    memset(obd2resp.values, 0x00, 4);
                    int motor_speed = car.motor_speed * 4;
                    obd2resp.values[0] = (motor_speed >> 8) & 0xFF;
                    obd2resp.values[1] = motor_speed & 0xFF;

                    frame_to_send = 1;
                    break;
                case OBD2_VEHICLE_SPEED_PID:
                    obd2resp.count_additional_data_bytes = 1;
                    obd2resp.pid = OBD2_VEHICLE_SPEED_PID;
                    memset(obd2resp.values, 0x00, 4);
                    obd2resp.values[0] = car.speed;

                    frame_to_send = 1;
                    break;
                case OBD2_THROTTLE_POSITION_PID:
                    obd2resp.count_additional_data_bytes = 1;
                    obd2resp.pid = OBD2_THROTTLE_POSITION_PID;
                    memset(obd2resp.values, 0x00, 4);
                    obd2resp.values[0] = car.throttle_percent * 255 / 100;

                    frame_to_send = 1;
                    break;
                default:
                    fprintf(stderr, "[warning] obd2 handler: unhandled pid 0x%02X\n", obd2req.pid);
                    break;
            }


            if (frame_to_send) {
                encode_obd2_respframe(&frame, &obd2resp);
                send_frame(request_can_socket, &frame);
            }
        }
    }

	return EXIT_SUCCESS;
}
