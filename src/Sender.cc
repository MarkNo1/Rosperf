//
// Created by devel on 5/29/19.
//

#include <rosperf/Sender.h>

rosperf::Sender::Sender() : _sequence_counter(0), _payload_dim(10),_initial_hz(100), _hz(100), _success_rate(false){
    initialize();
}

rosperf::Sender::~Sender() = default;

void rosperf::Sender::callback(const rosperf::Signal::ConstPtr &signal) {
    // Increase Payload and Reset
    if (signal->signal == rosperf::Signal::INCREASE_DM) {
        _payload_dim= _payload_dim + 10;
        _sequence_counter = 0;
        _hz = _initial_hz;
    }
    // Increase Hertz
    if (signal->signal == rosperf::Signal::INCREASE_HZ) {
        _success_rate = true;
        _hz = _hz + 1;
        _sequence_counter =0;
    }
}

void rosperf::Sender::send() {
    // Increase sequence counter
    _sequence_counter++;
    // Create Frame msg
    rosperf::Frame frame;
    frame.sequence = _sequence_counter;
    frame.hz = _hz;
    // Fill payload
    for (int i=0; i<_payload_dim; i++) {
        frame.payload.emplace_back(rosperf::Byte());
    }
    // Publish
    _pub.publish(frame);

}

void rosperf::Sender::initialize() {
    _sub = _node.subscribe("/Reset", 1, &Sender::callback, this,   ros::TransportHints().tcpNoDelay());
    _pub =  _node.advertise<rosperf::Frame>("/Frames", 1);
}

void rosperf::Sender::Log() {
        ROS_INFO_STREAM("-- Sending: " << "sequence(" << _sequence_counter << ") hz(" << _hz << ") dim(" << _payload_dim <<" byte) .");
}

void rosperf::Sender::start() {
    while(ros::ok()){
        // Set loop rate in hz
        ros::Rate loop(_hz);
        // Reset success condition
        _success_rate = false;
        // Loop
        while (! _success_rate && ros::ok()) {
            // Send Frame
            send();
            // Spin ros callbacks
            ros::spinOnce();
            // Sleep
            loop.sleep();
        }
    }
}
