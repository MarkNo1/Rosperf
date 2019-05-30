/*!****************************************************************************
 *  @file 	 Sender.h
 *  @date 	 2019.05.30
 *  @author  M. Treglia
 *  @brief
 *****************************************************************************/

#ifndef ROSNODELETS_SENDER_
#define ROSNODELETS_SENDER_

#include <ros/ros.h>
// Declare a Ros Nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
// Base Template Class for nodelets
#include <rosnodelets/template/BaseNodelet.h>
// Messages
#include <rosperf/Frame.h>
#include <rosperf/Signal.h>
/**
 *  Sender namespace
 */
namespace rosnodelets {

/**
 *  Naming simplification
 */
//using :

/**
 * @class Sender
 * @brief
 */
class Sender : public base::NodeletBM {
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
  inline void Callback(const rosperf::Signal::ConstPtr& signal);

  /**
   *  Callback to benchmark the nodelet
   */
  inline void CallbackBM(const rosperf::Signal::ConstPtr& signal);

  /**
   * Send
   */
  void Send();

  /**
   * Start loop
   */
  void StartLoop();

 private:
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

}  // namespace rosnodelets

/**
 * Register the nodelet.
 */
PLUGINLIB_EXPORT_CLASS(rosnodelets::Sender, nodelet::Nodelet);

#endif  // ROSNODELETS_SENDER_
