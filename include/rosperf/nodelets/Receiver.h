/*!****************************************************************************
 *  @file 	 Receiver.h
 *  @date 	 2019.05.30
 *  @author  M. Treglia
 *  @brief
 *****************************************************************************/

#ifndef ROSNODELETS_RECEIVER_
#define ROSNODELETS_RECEIVER_

#include <ros/ros.h>
// Declare a Ros Nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
// Base Template Class for nodelets
#include <rosnodelets/template/BaseNodelet.h>
// Message
#include <rosperf/Frame.h>
#include <rosperf/Byte.h>
#include <rosperf/Signal.h>

/**
 *  Receiver namespace
 */
namespace rosnodelets {

/**
 *  Naming simplification
 */
//using :

/**
 * @class Receiver
 * @brief
 */
class Receiver : public base::NodeletBM {
 public:
  using base::NodeletBM::Nodelet;

    /**
     * Initialization nodelet name
     */
    void InitializeNodeletName() final;

  /**
   * Initialization all components
   */
  void InitializeNodelet() final;

  /**
   * Loading ros parameters
   */
  void LoadRosParameters() final;

  /**
   * Establish ros communications
   */
  void EstablishRosCommunication() final;

  /**
   *  Callback to velodyne packets
   */
  inline void Callback(const rosperf::Frame::ConstPtr &frame);

  /**
   *  Callback to benchmark the nodelet
   */
  inline void CallbackBM(const rosperf::Frame::ConstPtr &frame);

  /**
   * Send
   */
  void Send(int code);

  /**
   * Reset
   */
  void Reset();

  void Log(int status);

 private:
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
    // Last delay
    ros::Duration _last_delay;

};

}  // namespace rosnodelets

/**
 * Register the nodelet.
 */
PLUGINLIB_EXPORT_CLASS(rosnodelets::Receiver, nodelet::Nodelet);

#endif  // ROSNODELETS_RECEIVER_
