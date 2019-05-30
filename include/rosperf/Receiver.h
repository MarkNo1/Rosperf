//
// Created by devel on 5/29/19.
//

#ifndef MARK_RECEIVER_H
#define MARK_RECEIVER_H

#include <ros/ros.h>
// Messages
#include <rosperf/Frame.h>
#include <rosperf/Signal.h>

namespace rosperf {
    class Receiver {
    public:
        Receiver();
        ~Receiver();
        // Init communications
        void initialize();
        // Signal - Reset callback
        void callback(const rosperf::Frame::ConstPtr& frame);
        // Send
        void send(int code);
        // Reset
        void reset();
        // Log
        void Log(int status);
        // Start testing
        void start();

    private:
        // Node
        ros::NodeHandle _node;
        // Communications
        ros::Publisher _pub;
        ros::Subscriber _sub;
        // Params
        int _last_sequence;
        float _last_hz;
        float _last_dim;
        // Success param
        int _success_counter;
        int _success_threshold;
        // Fail param
        int _fail_counter;
        int _fail_threshold;



    };
}

#endif //MARK_RECEIVER_H
