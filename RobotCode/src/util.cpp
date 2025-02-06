#include "util.h"
#include <chrono>

uint64_t getUnixTimeMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
