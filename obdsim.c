#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <stdint.h>
#include <signal.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdio.h>
#include <arpa/inet.h>

static int quit = 0;

void handler(int s)
{
    quit = 1;
}

int main(int argc, const char *argv[])
{
    const char* interface = "vcan0";

    struct sigaction act;
    act.sa_handler = handler;
    sigaction(SIGINT, &act, NULL);


    int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0)
    {
        perror("Socket");
        return 1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface, IFNAMSIZ -1);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("Ioctl");
        return 1;
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        return 1;
    }

    struct can_filter filter[2] = {
        {0x7e0, CAN_SFF_MASK & ~0x0f},
        {0x7df, CAN_SFF_MASK}
    };
    setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter[0], sizeof(filter));


    uint8_t data[8];
    memset(data, 0, sizeof(data));

    while(!quit)
    {
        struct can_frame frame;
        memset(&frame, 0, sizeof(struct can_frame));
        if (recv(sockfd, &frame, sizeof(frame), 0) < 0)
        {
            perror("Receive");
            return 2;
        }

        const uint32_t id = frame.can_id;
        
        memset(data, 0x00, sizeof(data));
        memcpy(data, frame.data, frame.len);

        printf("%03x#%02x.%02x.%02x.%02x.%02x.%02x.%02x.%02x\n", 
            id,
            data[0], data[1], data[2], data[3], 
            data[4], data[5], data[6], data[7]
        );

        if (data[1] == 0x01)
        {
            if (id == 0x7df)
            {
                // ecu 0
                uint32_t bytes = 0xebeb81bb;
                bytes = htonl(bytes);
                memcpy(&data[3], &bytes, sizeof(uint32_t));
                data[0] = 0x07;
                data[1] = 0x41;
                memset(&frame, 0, sizeof(struct can_frame));
                frame.can_id = 0x7e8;
                frame.len = 8;
                memcpy(frame.data, data, sizeof(data));

                if (send(sockfd, &frame, sizeof(struct can_frame), 0) < 0)
                {
                    perror("Send");
                    return 2;
                }

                usleep(50000);

                // ecu 1
                bytes = 0x000000ff;
                bytes = htonl(bytes);
                memcpy(&data[3], &bytes, sizeof(uint32_t));
                data[0] = 0x07;
                data[1] = 0x41;
                memset(&frame, 0, sizeof(struct can_frame));
                frame.can_id = 0x7e9;
                frame.len = 8;
                memcpy(frame.data, data, sizeof(data));
                if (send(sockfd, &frame, sizeof(struct can_frame), 0) < 0)
                {
                    perror("Send");
                    return 2;
                }

                usleep(50000);
            }
            else
            {
                // ecu 0
                uint32_t bytes = (data[2] == 0x80) ? 0 : 0xebeb81bb;
                bytes = htonl(bytes);
                memcpy(&data[3], &bytes, sizeof(uint32_t));
                data[0] = 0x01;
                data[1] = 0x43;
                memset(&frame, 0, sizeof(struct can_frame));
                frame.can_id = id + 8;
                frame.len = 8;
                memcpy(frame.data, data, sizeof(data));

                if (send(sockfd, &frame, sizeof(struct can_frame), 0) < 0)
                {
                    perror("Send");
                    return 2;
                }

                usleep(50000);
            }



        }
        else if (data[1] == 0x09)
        {
            uint32_t id2 = 0x7e8;
            if (id != 0x7df)
            {
                id2 = id + 8;
            }

            if(data[2] == 0x00 && data[3] == 0x02)
            {
                data[0] = 7;
                data[1] = 0x49;
                memset(&data[2], 'A', sizeof(data) - 2);
            }
            else
            {
                // ecu 0
                uint32_t bytes = 0xebeb81bb;
                bytes = htonl(bytes);
                memcpy(&data[3], &bytes, sizeof(uint32_t));
                data[0] = 0x07;
                data[1] = 0x49;
            }

            memset(&frame, 0, sizeof(struct can_frame));
            frame.can_id = id2;
            frame.len = 8;
            memcpy(frame.data, data, sizeof(data));

            if (send(sockfd, &frame, sizeof(struct can_frame), 0) < 0)
            {
                perror("Send");
                return 2;
            }

            usleep(50000);
        }
        else if (data[1] == 0x03 || data[1] == 0x07 || data[1] == 0x0a)
        {
            uint8_t service = data[1];

            memset(&frame, 0, sizeof(struct can_frame));
            frame.can_id = 0x7e8;
            frame.len = 8;
            memset(data, 0x00, sizeof(data));
            data[0] = 0x10;
            data[1] = 0x0b; // first frame 11 bytes

            data[2] = 0x40 | service; // response to 0x03

            data[3] = 0xc1;
            data[4] = 0x58; // U0158

            data[5] = 0xc1;
            data[6] = 0x58; // U0158

            data[7] = 0xc1;

            memcpy(frame.data, data, sizeof(data));

            if (send(sockfd, &frame, sizeof(struct can_frame), 0) < 0)
            {
                perror("Send");
                return 2;
            }

            usleep(50000);
            
            memset(data, 0x00, sizeof(data));
            data[0] = 0x20; // consecutive

            data[1] = 0x58; // U0158
            data[2] = 0xc1; 
            data[3] = 0x58; // U0158
            data[4] = 0xc1; 
            data[5] = 0x58; // U0158
            
            memcpy(frame.data, data, sizeof(data));

            if (send(sockfd, &frame, sizeof(struct can_frame), 0) < 0)
            {
                perror("Send");
                return 2;
            }
        }

    }





    printf("done\n");

    close(sockfd);

    return 0;
}