#include "ISO15765.hpp"
#include <iostream>

ISO15765Decoder::ISO15765Decoder() : length(0), index(0), last(0)
{
    std::fill(defragmented.begin(), defragmented.end(), 0x00);
}


bool ISO15765Decoder::add_fragment(const std::array<uint8_t, 8> &data)
{
    // returns true when final piece received

    const header_type type = (header_type)(data[0] >> 4);

    if (type == header_type::single)
    {
        length = data[0] & 0x07;

        std::copy(data.cbegin() + 1, data.cbegin() + length + 1, defragmented.begin());

        return false;
    }
    else if (type == header_type::first)
    {
        length = data[1] | (data[0] & 0x0f) << 8;

        std::copy(data.cbegin() + 2, data.cend(), defragmented.begin());

        index = data.size() - 2;
        last = 0;

        return true;
    }
    else if (type == header_type::consecutive)
    {
        const int seq = data[0] & 0x0f;
        if (seq != last + 1)
        {
            std::cerr << "WARNING: missed ISO15765 frame with sequence " << (last + 1) << std::endl;
            index += 7;
        }

        std::copy(data.cbegin() + 1, data.cend(), defragmented.begin() + index);
        index += (data.size() - 1);

        last = seq;

        return (index < length);
    }

    // flow control, do nothing

    return false;
}

const std::vector<uint8_t> ISO15765Decoder::get_data() const
{
    return std::vector<uint8_t>(defragmented.cbegin(), defragmented.cbegin() + length);
}
