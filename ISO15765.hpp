#ifndef __ISO15765_H
#define __ISO15765_H

#include <array>
#include <cstdint>
#include <vector>

static const int MAX_LENGTH = 4095;
enum header_type:uint8_t {single, first, consecutive, flow};
static const int ISO15765_DATA_OFFSET = 2;

class ISO15765Decoder
{
    public:
        ISO15765Decoder();
        ~ISO15765Decoder() = default;

        bool add_fragment(const std::array<uint8_t, 8> &data);
        const std::vector<uint8_t> get_data() const;

    private:
        std::array<uint8_t, MAX_LENGTH> defragmented;
        int length; // expected total length
        int index; // offset in buffer
        int last; // Last sequence
};

#endif //__ISO15765_H
