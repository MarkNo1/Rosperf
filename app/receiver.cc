#include <rosperf/Receiver.h>


int main(int argc, char** argv) {
    // Init Ros
    ros::init(argc, argv, "receiver");
    // Sender
    rosperf::Receiver receiver;
    // Start
    receiver.start();

    return 0;
}