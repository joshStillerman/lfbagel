#ifndef MDS_PUT_FRAME_HPP
#define MDS_PUT_FRAME_HPP

#include <vector>
#include "thread_safe_queue.hpp"
#include <mdsplus.hpp>
using namespace mdsplus;

struct Frame
{
    unsigned index;
    uint64_t timestamp;
    std::vector<uint8_t> data;
}
#endif
