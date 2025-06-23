#ifndef MDS_PUT_FRAME_H
#define MDS_PUT_FRAME_H

#include <vector>
#include <thread>
#include "thread_safe_queue.h"

        typedef struct Frame
        {
            unsigned index;
            uint64_t timestamp;
            instant_t frame_time;
            std::vector<uint8_t> data;
        } Frame;

        typedef struct StateStruct {
            unsigned width = 0;
            unsigned height = 0;
            std::string treename;
            std::string pathname;
            int shot_number;
            unsigned frameIndex = 0;
            thread_safe_queue<Frame *> frames;
            std::atomic_bool is_running = true;
            std::thread writer_thread;
        } StateStruct;
#endif
