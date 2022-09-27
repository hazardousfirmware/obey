#ifndef __CAN_DEVICE_H
#define __CAN_DEVICE_H

#include <string>
#include <cstdint>
#include <array>

class CANDevice
{
    public:
        CANDevice(std::string interface);
        ~CANDevice();

        void data_send(uint32_t id, const std::array<uint8_t, 8> &data);
        bool data_receive(uint32_t &id, std::array<uint8_t, 8> &data);

        void filter(uint32_t id, uint32_t mask = 0x7FF);
        void nofilter();

    private:
        int sockfd;

        void raw_send(const uint8_t *data, size_t len);

        const int timeout_ms = 200;
};

#endif // __CAN_DEVICE_H
