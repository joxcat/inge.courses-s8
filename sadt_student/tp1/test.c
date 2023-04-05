
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

int can_socket;

void ctrlc_handler(int signum){
    printf("Gracefull shutdown\r\n");

    /* Closing the CAN socket */
	if (close(can_socket) < 0) {
		perror("Close");
		exit(EXIT_FAILURE);
	}

    exit(EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
    int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct can_frame frame;

	printf("CAN TP1 Test\r\n");

    /* Creating a CAN socket */
	if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

    /* Selecting the network device vcan0 */
	strcpy(ifr.ifr_name, "vcan0" );
    /* TODO: ??? */
	ioctl(can_socket, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
    /* Setting the CAN Family of the address */
	addr.can_family = AF_CAN;
    /* Linking the CAN address with the network device */
	addr.can_ifindex = ifr.ifr_ifindex;

    /* Binding the socket to the vcan0 address */
	if (bind(can_socket, (struct sockaddr *) & addr, sizeof(addr)) < 0) {
		perror("Bind");
		return 1;
	}

    /* Filtering frames */
    struct can_filter rfilter[1];
    /* From 0x100 */
    rfilter[0].can_id   = 0x100;
    /* To 0x1FF */
    // rfilter[0].can_mask = 0xF00;
    /* To 0x3FF */
	rfilter[0].can_mask = 0xD00;

    /* Set socket filtering */
    setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    /* CTRL+C handler */
    signal(SIGINT, ctrlc_handler); 

    while (1) {
        /* Creating the CAN frame */
        frame.can_id = 0x123;
        frame.can_dlc = 8;
        sprintf(frame.data, "Hello U!");

        /* Writing the frame to the CAN */
        if (write(can_socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write");
            return 1;
        }

        /* Reading a frame from the socket */
        nbytes = read(can_socket, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            perror("Read");
            return 1;
        }

        printf("0x%03X [%d] ", frame.can_id, frame.can_dlc);

        for (int i = 0; i < frame.can_dlc; i++)
            printf("%02X ", frame.data[i]);

        printf("\r\n");

        /* Flushing and waiting 1s */
        fflush(stdout);
    }

	return 0;
}
