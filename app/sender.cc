#include <rosperf/Sender.h>


int main(int argc, char** argv) {
    // Init Ros
    ros::init(argc, argv, "sender");
    // Sender
    rosperf::Sender sender;
    // Start
    sender.start();
return 0;
}
