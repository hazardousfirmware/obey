#include <cstdint>
#include <string>
#include <algorithm>
#include <array>
#include <functional>
#include <iostream>
#include <chrono>

#include "CAN.hpp"
#include "ISO15765.hpp"

using namespace std::chrono_literals;

const uint32_t OBD_BROADCAST = 0x7df;
const uint32_t OBD_ECU_SEND_BASE = 0x7e0;
const uint32_t OBD_ECU_RECV_BASE = OBD_ECU_SEND_BASE + 8;

const uint8_t OBD_PAD_BYTE = 0xCC;

const auto RESPONSE_WAIT = 1s;

const int MAX_ECUS = 8;
const int ANY_ECU = -1;

const int MIN_SERVICE = 0x00;
const int MAX_SERVICE = 0x3f;
const int MIN_PID = 0x00;
const int MAX_PID = 0xffff;
const int MAX_STANDARD_PID = 0xff;

const int VEHICLE_INFO_SERVICE = 0x09;
const int SHOW_DATA_SERVICE = 0x01;
const int SHOW_FREEZE_FRAME_SERVICE = 0x02;

enum fault_code_source:uint8_t {stored = 0x03 /* default */, pending = 0x07, permanent = 0x0a};

using can_data = std::array<uint8_t, 8>;

auto wait_override = RESPONSE_WAIT;


void foreach_pid(uint32_t features, std::function<void(int)> callback)
{
    for (int i = 0; i < 0x20; i++)
    {
        if ((features >> (31 - i)) & 0x01 == 0x01)
        {
            callback(i + 1);
        }
    }
}

void do_until_expire(std::function<bool()> what)
{
    const std::chrono::milliseconds wait = wait_override;
    const auto expire = std::chrono::system_clock::now() + wait;
    bool abort = false;
    while(std::chrono::system_clock::now() < expire && !abort)
    {
        abort = what();
    }
}


const std::string decode_dtc(uint16_t dtc)
{
    char code[6] = {'\x00'};
    switch(dtc >> 14)
    {
    case 0b00:
        code[0] = 'P';
        break;
    case 0b01:
        code[0] = 'C';
        break;
    case 0b10:
        code[0] = 'B';
        break;
    case 0b11:
        code[0] = 'U';
        break;
    }

    code[1] = ((dtc >> 12) & 0x03) + '0';

    code[2] = ((dtc >> 8) & 0x0f);
    code[3] = ((dtc >> 4) & 0x0f);
    code[4] = (dtc & 0x0f);

    code[2] = (code[2] > 0x09) ? (code[2] + '7') : (code[2] + '0');
    code[3] = (code[3] > 0x09) ? (code[3] + '7') : (code[3] + '0');
    code[4] = (code[4] > 0x09) ? (code[4] + '7') : (code[4] + '0');

    return std::string{code};
}

const std::vector<uint8_t> receive_multipart(CANDevice &can)
{
    ISO15765Decoder decoder;

    bool more = true;
    bool first = false;

    can_data buffer{};

    do_until_expire([&]() -> bool {
        uint32_t id = 0;

        if (!can.data_receive(id, buffer))
        {
            return false;
        }

        more = decoder.add_fragment(buffer);

        if (more)
        {
            if (!first)
            {
                std::fill(buffer.begin(), buffer.end(), 0x00);
                buffer[0] = 0x30; // flow control, enable all remaining parts
                can.data_send(id - 8, buffer);
                first = true;
            }

            return false;
        }

        return true; // expire the loop
    });

    return decoder.get_data();
}

void read_info(CANDevice &can, int ecu = ANY_ECU)
{
    can_data buffer{};
    std::fill(buffer.begin(), buffer.end(), OBD_PAD_BYTE);

    // Generate ISO-15765 headers + OBD-II command
    buffer[1] = 0x09; // Service 9
    buffer[2] = 0x00; // Get supported PIDs (1-20)
    buffer[0] = 0x02; // Single frame + length

    if (ecu < 0)
    {
        // Broadcast to all ECUs
        can.filter(OBD_ECU_RECV_BASE, ~0x07);
        can.data_send(OBD_BROADCAST, buffer);
    }
    else
    {
        can.filter(ecu + OBD_ECU_RECV_BASE);
        can.data_send(ecu + OBD_ECU_SEND_BASE, buffer);
    }

    do_until_expire([&]() -> bool {
        uint32_t id = 0;

        if (can.data_receive(id, buffer))
        {
            if (ecu < 0)
            {
                std::cout << "ECU: " << id - OBD_ECU_RECV_BASE
                    << " (0x" << std::hex << id + OBD_ECU_SEND_BASE - OBD_ECU_RECV_BASE
                    << "/0x" << id << std::dec
                    << ") :" << std::endl;
            }

            const uint32_t features = buffer[3] << 24 | buffer[4] << 16 | buffer[5] << 8 | buffer[6];

            printf("    Available vehicle information: 0x%08x\n", features);
            std::cout << "    ";
            foreach_pid(features, [&](int pid) { printf("%02X,", pid); });
            std::cout << std::endl << std::endl;

            return true;
        }
        return false;
    });
}


void enumerate(CANDevice &can)
{
    const int FEATURE_PAGE_SIZE = 0x20;
    const int MAX_DATAS = 7; // 7 pages of info of length 0x20; 0x01-0xe0

    // Accept all IDs between 0x7e8 - 0x7ef
    can.filter(OBD_ECU_RECV_BASE, ~0x07);

    can_data buffer{};
    std::fill(buffer.begin(), buffer.end(), OBD_PAD_BYTE);

    // Generate ISO-15765 headers + OBD-II command
    buffer[1] = 0x01; // Service 1
    buffer[2] = 0x00; // Get supported PIDs (1-20)
    buffer[0] = 0x02; // single frame, length

    can.data_send(OBD_BROADCAST, buffer);

    std::array<uint32_t, MAX_ECUS> ecu_features{};
    std::fill(ecu_features.begin(), ecu_features.end(), 0);

    bool found = false;

    std::cerr << "Waiting for ECUs to respond..." << std::endl;

    // Wait for ECU responses
    do_until_expire([&]() -> bool {
        uint32_t can_id = 0;
        can_data buffer{};

        if (can.data_receive(can_id, buffer))
        {
            const int ecu_index = can_id - OBD_ECU_RECV_BASE;
            found = true;

            const uint32_t features = buffer[3] << 24 | buffer[4] << 16 | buffer[5] << 8 | buffer[6];
            ecu_features[ecu_index] = features;
        }

        return false;
    });

    if (!found)
    {
        std::cout << "No ECUs found" << std::endl;
        return;
    }

    // Print the ECU responses and check for next page then collect the ECU available info
    for (int ecu = 0; ecu < MAX_ECUS; ecu++)
    {
        uint32_t features = ecu_features[ecu];

        if (features == 0)
        {
            // No ECU found for current slot
            continue;
        }

        const uint32_t send_id = ecu + OBD_ECU_SEND_BASE;
        const uint32_t recv_id = ecu + OBD_ECU_RECV_BASE;

        // Print the discovered ECU
        std::cout << "Found ECU: " << ecu
                    << " (0x" << std::hex << send_id
                    << "/0x" << recv_id << std::dec
                    << ") :" << std::endl;

        // Enumerate additional pages of read data service 0x01
        for (int page = 0; page < MAX_DATAS; page++)
        {
            const int offset = page * FEATURE_PAGE_SIZE;

            if (page > 0)
            {
                // Send request to ECU for next page of available info
                features = 0;

                buffer[2] = (uint8_t)offset; // Get supported PIDs (1-20) + 0x20 * page

                can.filter(recv_id);
                can.data_send(send_id, buffer);
                do_until_expire([&]() -> bool {
                    uint32_t can_id = 0;
                    can_data buffer{};

                    if (can.data_receive(can_id, buffer))
                    {
                        features = buffer[3] << 24 | buffer[4] << 16 | buffer[5] << 8 | buffer[6];

                        return true;
                    }

                    return false;
                });
            }

            if (features == 0)
            {
                // No more extra pages
                break;
            }


            printf("    Available data [%02X-%02X]: 0x%08x\n",
                    (offset + 1), // first feature
                    (offset + FEATURE_PAGE_SIZE), //last feature
                    features
            );
            std::cout << "    ";
            foreach_pid(features, [&](int pid) { printf("%02X,", pid + offset); });
            std::cout << std::endl << std::endl;

            if (features & 0x01 == 0)
            {
                // No more extra pages
                break;
            }
        }

        // Enumerate available vehicle info 0x09
        read_info(can, ecu);
    }
}

void clear_dtc(CANDevice &can, int ecu = ANY_ECU)
{
    can_data buffer{};
    std::fill(buffer.begin(), buffer.end(), OBD_PAD_BYTE);

    // Generate ISO-15765 headers + OBD-II command
    buffer[1] = 0x04; // Service 4
    buffer[0] = 0x01; // single frame, length

    if (ecu < 0)
    {
        // Broadcast to all ECUs
        can.data_send(OBD_BROADCAST, buffer);
    }
    else
    {
        can.data_send(ecu + OBD_ECU_SEND_BASE, buffer);
    }

    std::cerr << "Cleared DTC" << std::endl;
}

void read_dtc(CANDevice &can, int ecu = ANY_ECU, fault_code_source service = stored)
{
    can_data buffer{};
    std::fill(buffer.begin(), buffer.end(), OBD_PAD_BYTE);

    // Generate ISO-15765 headers + OBD-II command
    buffer[1] = service; // Service 3
    buffer[0] = 0x01; // single frame, length

    if (ecu < 0)
    {
        // Broadcast to all ECUs
        can.filter(OBD_ECU_RECV_BASE, ~0x07);
        can.data_send(OBD_BROADCAST, buffer);
    }
    else
    {
        can.filter(ecu + OBD_ECU_RECV_BASE);
        can.data_send(ecu + OBD_ECU_SEND_BASE, buffer);
    }

    const std::vector<uint8_t> defragmented = receive_multipart(can);

    if (defragmented.empty() || (defragmented[0] & 0x3f) != service)
    {
        // unknown service
        return;
    }

    std::cout << "Diagnostic trouble codes:" << std::endl;

    for (int i = 1; i < (int)defragmented.size() -1; i++)
    {
        const uint16_t dtc = defragmented[i] << 8 | defragmented[++i];
        std::cout << decode_dtc(dtc) << std::endl;
    }
}

void request(CANDevice &can, int service, int pid, int ecu = ANY_ECU)
{
    if (service < MIN_SERVICE || service > MAX_SERVICE)
    {
        std::cerr << "Impossible Service ID" << std::endl;
        return;
    }

    if (pid < MIN_PID || pid > MAX_PID)
    {
        std::cerr << "Impossible PID" << std::endl;
        return;
    }

    can_data buffer{};
    std::fill(buffer.begin(), buffer.end(), OBD_PAD_BYTE);

    // Generate ISO-15765 headers + OBD-II command
    buffer[1] = (uint8_t)service;
    if (pid > MAX_STANDARD_PID)
    {
        buffer[2] = (uint8_t)(pid >> 8);
        buffer[3] = (uint8_t)pid;
        buffer[0] = 3;
    }
    else
    {
        buffer[2] = (uint8_t)pid;
        buffer[0] = 2;
    }


    if (ecu < 0)
    {
        // Broadcast to all ECUs
        can.filter(OBD_ECU_RECV_BASE, ~0x07);
        can.data_send(OBD_BROADCAST, buffer);
    }
    else
    {
        can.filter(ecu + OBD_ECU_RECV_BASE);
        can.data_send(ecu + OBD_ECU_SEND_BASE, buffer);
    }

    const std::vector<uint8_t> defragmented = receive_multipart(can);
    if (defragmented.empty() || (defragmented[0] & 0x3f) != service)
    {
        // unknown service
        return;
    }

    std::cout << "Results (";
    printf("Service: %02x, PID: %02x, length: %i)\n", service, pid, defragmented.size() - ISO15765_DATA_OFFSET);

    //defragmented[0] = pid | 0x40;
    //defragmented[1] = service;

    for (int i = ISO15765_DATA_OFFSET; i < defragmented.size(); i++)
    {
        printf("%02x", defragmented[i]);
    }

    std::cout << std::endl;
}

int main(int argc, const char* argv[])
{
    if (argc == 1)
    {
        // print help
        std::cout << "Raw SocketCAN OBD-II tool" << std::endl << std::endl;
        std::cout << "USAGE: " << argv[0] << " COMMAND [OPTIONS]" << std::endl;
        std::cout << "\tCommands:" << std::endl;
        std::cout << "\t\tenum - enumerate ECUs" << std::endl;
        std::cout << "\t\tshow - show data for ECU (service=0x01)" << std::endl;
        std::cout << "\t\trequest - read custom service/pid" << std::endl;
        std::cout << "\t\tfaults - read fault codes (DTCs) (service=0x03)" << std::endl;
        std::cout << "\t\tclear - clear fault codes (DTCs) (service=0x04)" << std::endl;
        std::cout << "\t\tfrozen - show freeze frame data for ECU (service=0x02)" << std::endl;
        std::cout << "\t\tpending - read pending fault codes (DTCs) (service=0x07)" << std::endl;
        std::cout << "\t\tinfo - read info (service=0x09)" << std::endl;
        std::cout << "\t\tpermanent - read permanent fault codes (DTCs) (service=0x0a)" << std::endl;
        std::cout << std::endl;
        std::cout << "\tOptions:" << std::endl;
        std::cout << "\t\t-i <interface> - use this network interface" << std::endl;
        std::cout << "\t\t-e <ecu> - only request from this ECU, 0-8. Do not broadcast (0x7df)" << std::endl;
        std::cout << "\t\t-s <service> - service number, applies only to request command, hex number" << std::endl;
        std::cout << "\t\t-p <pid> - pid number, applies only to request command, hex number" << std::endl;
        std::cout << "\t\t-t <seconds> - wait for timeout before stopping packet receive, default=1s" << std::endl;

        return 0;
    }

    std::string cmd{argv[1]};
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

    std::string interface = "can0";
    int ecu = ANY_ECU;
    int service = -1;
    int pid = -1;

    // Any further arguments
    for (int i = 2; i < argc; i++)
    {
        const std::string arg{argv[i]};
        if (arg == "-i")
        {
            interface = argv[++i];
        }
        else if (arg == "-s")
        {
           service = std::stol(argv[++i], nullptr, 16);
        }
        else if (arg == "-p")
        {
            pid = std::stol(argv[++i], nullptr, 16);
        }
        else if (arg == "-e")
        {
            ecu = std::stol(argv[++i]);
            if (ecu >= MAX_ECUS)
            {
                std::cerr << "Warning: impossible ECU number, using broadcast" << std::endl;
                ecu = ANY_ECU;
            }
        }
        else if (arg == "-t")
        {
            const int sec = std::stol(argv[++i]);
            wait_override = std::chrono::seconds(sec);
        }
    }

    CANDevice can{interface};

    if (cmd == "enum" || cmd == "list")
    {
        // enumerate the ECUs
        enumerate(can);
    }
    else if (cmd == "show" || cmd == "data")
    {
        request(can, SHOW_DATA_SERVICE, pid, ecu);
    }
    else if (cmd == "frozen" || cmd == "freeze")
    {
        request(can, SHOW_FREEZE_FRAME_SERVICE, pid, ecu);
    }
    else if (cmd == "clear")
    {
        // Clear DTCs
        clear_dtc(can, ecu);
    }
    else if (cmd == "faults" || cmd == "dtc")
    {
        read_dtc(can, ecu);
    }
    else if (cmd == "pending")
    {
        read_dtc(can, ecu, fault_code_source::pending);
    }
    else if (cmd == "permanent" || cmd == "perm")
    {
        read_dtc(can, ecu, fault_code_source::permanent);
    }
    else if (cmd == "info")
    {
        if (pid <= MIN_PID)
        {
            read_info(can, ecu);
        }
        else
        {
            request(can, VEHICLE_INFO_SERVICE, pid, ecu);
        }
    }
    else if (cmd == "request" || cmd == "read")
    {
        request(can, service, pid, ecu);
    }
    else
    {
        std::cerr << "Unknown command" << std::endl;
    }

    return 0;
}
