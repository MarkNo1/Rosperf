/*!****************************************************************************
 *  @file   Receiver.cc
 *  @date   2019.05.30
 *  @author M. Treglia
 *  @brief  Definition Receiver nodelet
 *****************************************************************************/

#include <rosperf/nodelets/Receiver.h>

namespace rosnodelets {

    void Receiver::InitializeNodeletName() {SetName("Receiver");}

    void Receiver::InitializeNodelet() {
        Reset();
        _success_threshold = 10;
        _fail_threshold = 3;
        LogInfo("Initialized");
    }

    void Receiver::LoadRosParameters() {}

    void Receiver::EstablishRosCommunication() {
        _sub = getMTNodeHandle().subscribe("/Frames", 1, &Receiver::Callback, this, ros::TransportHints().tcpNoDelay());
        _pub = getMTNodeHandle().advertise<rosperf::Signal>("/Reset", 1);
    }

    inline void Receiver::CallbackBM(const rosperf::Frame::ConstPtr &frame) {
     Start();
     Callback(frame);
     End();
    }

    inline void Receiver::Callback(const rosperf::Frame::ConstPtr &frame) {
        // Success
        if ( _last_sequence == frame->sequence -1) {
            _last_sequence = frame->sequence;
            _last_hz = frame->hz;
            _last_dim = sizeof(rosperf::Byte) * frame->payload.size() + 16;
            _last_delay = ros::Time::now() - frame->header.stamp;
            _success_counter++;
            // Success threshold times
            if(_success_counter == _success_threshold) {
                Log(1);
                // Reset
                Reset();
                // Increase Hz
                Send(rosperf::Signal::INCREASE_HZ);
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
                // Reset params
                Reset();
                // Increase Payload
                Send(rosperf::Signal::INCREASE_DM);
            }
        }
    }

    void Receiver::Send(int code) {
        // Create Signal msg
        rosperf::Signal signal;
        signal.signal = code;
        // Publish
        _pub.publish(signal);
    }

    void Receiver::Reset() {
        _last_hz =_last_dim = _last_sequence = _success_counter = _fail_counter  = 0;
        _last_delay = ros::Duration(0);
    }

    void Receiver::Log(int status) {
        // Warn
        if (status == 1 )
            ROS_WARN("hz(%f) dim(%f byte) success(%d/100) - delay (%f ms)",_last_hz, _last_dim, _success_counter, (float) _last_delay.nsec / 1000000);
        // Info
        if (status == 2 )
            ROS_INFO("-- sequence(%d) hz(%f) dim(%f byte) success(%d)", _last_sequence,_last_hz, _last_dim, _success_counter);
        // Error
        if (status == 3 )
            ROS_ERROR("-- sequence(%d) hz(%f) dim(%f byte) success(%d)", _last_sequence,_last_hz, _last_dim, _success_counter);
    }

}  // namespace rosnodelets
