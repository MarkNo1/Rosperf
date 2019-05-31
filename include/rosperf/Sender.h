//
// Created by devel on 5/29/19.
//

#ifndef MARK_SENDER_H
#define MARK_SENDER_H

#include <ros/ros.h>
// Messages
#include <rosperf/Frame.h>
#include <rosperf/Signal.h>

namespace rosperf {

    class Sender {
    public:
        Sender();
        ~Sender();
        // Init communications
        void initialize();
        // Signal - Reset callback
        void callback(const rosperf::Signal::ConstPtr& signal);
        // Send
        void send();
        // Log
        void Log();
        // Start testing
        void start();


    private:
        // Node
        ros::NodeHandle _node;
        // Communications
        ros::Publisher _pub;
        ros::Subscriber _sub;
        // Params
        float _hz;
        int _initial_hz;
        int _sequence_counter;
        int _payload_dim;
        // Success rate
        bool _success_rate;



    };
}

#endif //MARK_SENDER_H
