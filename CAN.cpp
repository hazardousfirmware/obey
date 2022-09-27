#include "CAN.hpp"

#include <cerrno>
#include <cstring>
#include <system_error>
#include <unistd.h>
#include <poll.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>


CANDevice::CANDevice(std::string interface)
: sockfd{ socket(PF_CAN, SOCK_RAW, CAN_RAW) }
{
    if (sockfd < 0)
    {
        throw std::system_error(errno, std::system_category(), "Socket");
    }

    ifreq ifr{};
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ -1);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        throw std::system_error(errno, std::system_category(), "Ioctl");
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        throw std::system_error(errno, std::system_category(), "Bind");
    }
}

CANDevice::~CANDevice()
{
    close(sockfd);
}

void CANDevice::nofilter()
{
    setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
}

void CANDevice::filter(uint32_t id, uint32_t mask)
{
    setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);

    can_filter filter[] = {{id, CAN_SFF_MASK & mask}};
    setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter[0], sizeof(filter));
}

void CANDevice::raw_send(const uint8_t *data, size_t len)
{
    if (send(sockfd, data, len, 0) < 0)
    {
        throw std::system_error(errno, std::system_category(), "Send");
    }
}

void CANDevice::data_send(uint32_t id, const std::array<uint8_t, 8> &data)
{
    can_frame frame{};
    frame.can_id = id;
    frame.len = 8;

    memcpy(frame.data, data.data(), data.size());

    raw_send((const uint8_t*)(&frame), sizeof(frame));
}

bool CANDevice::data_receive(uint32_t &id, std::array<uint8_t, 8> &data)
{
    pollfd fds[1] = {{ sockfd, POLLIN }};
    int result = poll(fds, sizeof(fds) / sizeof(pollfd), timeout_ms);

    if (result < 0)
    {
        throw std::system_error(errno, std::system_category(), "Poll");
    }
    else if (result == 0)
    {
        // timed out without data
        return false;
    }

    can_frame frame{};
    if (recv(sockfd, &frame, sizeof(frame), 0) < 0)
    {
        throw std::system_error(errno, std::system_category(), "Receive");
    }

    id = frame.can_id;
    
    memset(data.data(), 0x00, data.size());
    memcpy(data.data(), frame.data, frame.len);

    return true;
}
