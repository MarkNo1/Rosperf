/*!****************************************************************************
 *  @file   Sender.cc
 *  @date   2019.05.30
 *  @author M. Treglia
 *  @brief  Definition Sender nodelet
 *****************************************************************************/

#include <rosperf/nodelets/Sender.h>

namespace rosnodelets {

    void Sender::InitializeNodeletName() {SetName("Sender");}

    void Sender::InitializeNodelet() {
        _sequence_counter =0;
        _payload_dim = 10;
        _initial_hz = 100;
        _hz= 100;
        _success_rate = false;
        LogInfo("Initialized");
    }

    void Sender::LoadRosParameters() {}

    void Sender::EstablishRosCommunication() {
        _sub = getMTNodeHandle().subscribe("/Reset", 1, &Sender::Callback, this, ros::TransportHints().tcpNoDelay());
        _pub = getMTNodeHandle().advertise<rosperf::Frame>("/Frames", 1);
        // Start
        StartLoop();
    }

    inline void Sender::CallbackBM(const rosperf::Signal::ConstPtr &signal) {
     Start();
     Callback(signal);
     End();
    }

    inline void Sender::Callback(const rosperf::Signal::ConstPtr &signal) {
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

    void Sender::Send() {
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

    void Sender::StartLoop() {
        while(ros::ok()){
            // Set loop rate in hz
            ros::Rate loop(_hz);
            // Reset success condition
            _success_rate = false;
            // Loop
            while (! _success_rate && ros::ok()) {
                // Send Frame
                Send();
                // Sleep
                loop.sleep();
            }
        }
    }

}  // namespace rosnodelets
