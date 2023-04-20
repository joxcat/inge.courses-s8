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

#define FRAME_OBD2_REQUEST 0x700000DF
#define FRAME_OBD2_RESPONSE 0x700000E8
#define OBD2_ENGINE_SPEED_PID 0x0C
#define OBD2_VEHICLE_SPEED_PID 0x0D
#define OBD2_THROTTLE_POSITION_PID 0x11
#define OBD2_SHOW_CURRENT_DATA_REQ_SERVICE 0x01
#define OBD2_REQ_TO_RES_SERVICE 0x40


#define CLEAR_STDOUT "\e[1;1H\e[2J"

int request_can_socket;

void ctrlc_handler(int signum){
    printf("gracefull shutdown %d\r\n", signum);

    /* Closing the CAN socket */
    if (close(request_can_socket) < 0) {
        perror("[error] failed to close request can");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}

/* Decoding frames */

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
int decode_obd2_respframe(struct can_frame *frame, struct obd2_response *resp) {
    resp->count_additional_data_bytes = frame->data[0];
    resp->service = frame->data[1];
    resp->pid = frame->data[2];
    memcpy(resp->values, &(frame->data[3]), 4);

    return EXIT_SUCCESS;
}
int encode_obd2_reqframe(struct can_frame *frame, struct obd2_request *req) {
    memset(frame->data, 0x00, 8);
    frame->can_id = FRAME_OBD2_REQUEST;
    frame->len = 3;
    frame->data[0] = req->count_additional_data_bytes;
    frame->data[1] = req->service;
    frame->data[2] = req->pid;

    return EXIT_SUCCESS;
}

/* CAN interactions */
int send_frame(int socket, struct can_frame *frame) {
    printf("Sending 0x%02X on %d\n", frame->can_id, socket);
    /* Writing the frame to the CAN */
    if (write(socket, frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
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
            // perror("warning: timed out waiting for can_socket");
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
	struct sockaddr_can request_addr;
    
	struct ifreq request_ifr;
	struct can_frame frame;
    struct obd2_request obd2req;
    struct obd2_response obd2resp;

    /* Creating a CAN socket */
    request_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (request_can_socket < 0) {
        perror("[error] failed to create request socket");
        return EXIT_FAILURE;
    }

    /* Selecting the network device vcan1 */
    strcpy(request_ifr.ifr_name, "vcan1");
    ioctl(request_can_socket, SIOCGIFINDEX, &request_ifr);

    memset(&request_addr, 0, sizeof(request_addr));

    /* Setting the CAN Family of the address */
    request_addr.can_family = AF_CAN;
    /* Linking the CAN address with the network device */
	request_addr.can_ifindex = request_ifr.ifr_ifindex;

    /* Binding the socket to the vcan1 address */
    if (bind(request_can_socket, (struct sockaddr *) & request_addr, sizeof(request_addr)) < 0) {
		perror("[error] failed to bind the request socket");
		return EXIT_FAILURE;
	}

    /* CTRL+C handler */
    signal(SIGINT, ctrlc_handler); 

    while (1) {
        // HANDLE OBD2 Responses
        /*int res = read_frame(request_can_socket, &frame, REQUEST_TIMEOUT);
        if (res == EXIT_FAILURE) return EXIT_FAILURE;
        else if (res == EXIT_SUCCESS) {
            switch (frame.can_id) {
                case FRAME_OBD2_RESPONSE:
                    if (decode_obd2_respframe(&frame, &obd2resp)) return EXIT_FAILURE;
                    if (obd2req.service != (OBD2_SHOW_CURRENT_DATA_REQ_SERVICE + OBD2_REQ_TO_RES_SERVICE)) {
                        fprintf(stderr, "[warning] obd2 handler: unhandled service 0x%02X\n", obd2resp.service);
                        break;
                    }

                    // NOTE: Here it will always be a obd2 response
                    switch (obd2req.pid) {
                        case OBD2_ENGINE_SPEED_PID:
                            printf("Car engine speed: %d\n", ((obd2resp.values[1] << 8) + obd2resp.values[0]) / 4);
                            break;
                        case OBD2_VEHICLE_SPEED_PID:
                            printf("Car speed: %d\n", obd2resp.values[0]);
                            break;
                        case OBD2_THROTTLE_POSITION_PID:
                            printf("Car throttle: %d\n", obd2resp.values[0] * 100 / 255);
                            break;
                        default:
                            fprintf(stderr, "[warning] obd2 handler: unhandled pid 0x%02X\n", obd2req.pid);
                            break;
                    }
                    break;
                default:
                    fprintf(stderr, "[warning] request handler: unhandled can_id 0x%03X\n", frame.can_id);
            }
        }*/

        // HANDLE OBD2 Requests
        int frame_to_send = 0;
        obd2req.service = OBD2_SHOW_CURRENT_DATA_REQ_SERVICE;
        switch (rand() % 3) {
            case 0:
                obd2req.count_additional_data_bytes = 2;
                obd2req.pid = OBD2_ENGINE_SPEED_PID;

                printf("asked engine speed\n");
                frame_to_send = 1;
                break;
            case 1:
                obd2req.count_additional_data_bytes = 1;
                obd2req.pid = OBD2_VEHICLE_SPEED_PID;

                printf("asked vehicle speed\n");
                frame_to_send = 1;
                break;
            case 2:
                obd2req.count_additional_data_bytes = 1;
                obd2req.pid = OBD2_THROTTLE_POSITION_PID;

                printf("asked throttle\n");
                frame_to_send = 1;
                break;
            default:
                break;
        }

        if (frame_to_send) {
            encode_obd2_reqframe(&frame, &obd2req);
            send_frame(request_can_socket, &frame);
        }
    }

	return EXIT_SUCCESS;
}
