#include "SerialForwarder.h"

int main() {
    SerialForwarder forwarder("/dev/ttyACM0", "/dev/ttyACM1", 115200);
    if (!forwarder.isConnected()) return 1;
    forwarder.run();
    return 0;
}
