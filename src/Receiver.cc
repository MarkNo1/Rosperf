//
// Created by devel on 5/29/19.
//

#include <rosperf/Receiver.h>

rosperf::Receiver::Receiver() : _last_sequence(0), _last_hz(0), _last_dim(0),
                                _success_counter(0), _success_threshold(10),
                                _fail_counter(0), _fail_threshold(3){
    initialize();
}

rosperf::Receiver::~Receiver() = default;

void rosperf::Receiver::callback(const rosperf::Frame::ConstPtr &frame) {
    // Success
    if ( _last_sequence == frame->sequence -1) {
        _last_sequence = frame->sequence;
        _last_hz = frame->hz;
        _last_dim = sizeof(rosperf::Byte) * frame->payload.size() + 16;
        _success_counter++;
        // Success threshold times
        if(_success_counter == _success_threshold) {
            // Reset
            reset();
            // Increase Hz
            send(rosperf::Signal::INCREASE_HZ);
        }
    }
    // Fail
    else {
        // Copy last sequence
        _last_sequence = frame->sequence;
        // Increase fail case
        _fail_counter++;
        // Fail threshold times
        if (_fail_counter == _fail_threshold) {
            Log(3);
            // Reset params
            reset();
            // Increase Payload
            send(rosperf::Signal::INCREASE_DM);
        }
    }
}

void rosperf::Receiver::send(int code) {
    // Create Signal msg
    rosperf::Signal signal;
    signal.signal = code;
    // Publish
    _pub.publish(signal);
}

void rosperf::Receiver::initialize() {
    _sub = _node.subscribe("/Frames", 1, &Receiver::callback, this,  ros::TransportHints().tcpNoDelay());
    _pub =  _node.advertise<rosperf::Signal>("/Reset", 1);
}

void rosperf::Receiver::reset() {
    _last_hz = 0;
    _last_dim = 0;
    _last_sequence = 0;
    _success_counter = 0;
    _fail_counter = 0;
}

void rosperf::Receiver::Log(int status) {
    // Warn
    if (status == 1 )
        ROS_WARN("-- sequence(%d) hz(%f) dim(%f byte) success(%d)", _last_sequence,_last_hz, _last_dim, _success_counter);
    // Info
    if (status == 2 )
        ROS_INFO("-- sequence(%d) hz(%f) dim(%f byte) success(%d)", _last_sequence,_last_hz, _last_dim, _success_counter);
    // Error
    if (status == 3 )
        ROS_ERROR("-- sequence(%d) hz(%f) dim(%f byte) success(%d)", _last_sequence,_last_hz, _last_dim, _success_counter);
}

void rosperf::Receiver::start() { ros::spin();}
