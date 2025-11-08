// Copyright (C) 2022 Christian Brommer, Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <christian.brommer@ieee.org>

#ifndef MARSWRAPPERGPS_H
#define MARSWRAPPERGPS_H

#include <dynamic_reconfigure/server.h>
#include <mars/core_logic.h>
#include <mars/core_state.h>
#include <mars/sensors/gps/gps_measurement_type.h>
#include <mars/sensors/gps/gps_sensor_class.h>
#include <mars/sensors/imu/imu_sensor_class.h>
#include <mars_msg_conv.h>
#include <mars_ros/marsConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>

#include <boost/bind/bind.hpp>
#include <params/mars_dualpose_paramloader.h>

///
/// \brief The MarsWrapperGps class MaRS single GPS node
///
class MarsWrapperGps
{
public:
  MarsWrapperGps(ros::NodeHandle nh);

  // Settings
  ParamLoad m_sett_;

  // Node services
  ros::ServiceServer initialization_service_;  ///< Service handle for filter initialization

  ///
  /// \brief initServiceCallback Service to initialize / re-initialize the filter
  /// \return True if service call was successful
  ///
  bool initServiceCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  // Dynamic reconfigure components
  dynamic_reconfigure::Server<mars_ros::marsConfig> reconfigure_srv_;
  dynamic_reconfigure::Server<mars_ros::marsConfig>::CallbackType reconfigure_cb_;

  ///
  /// \brief configCallback Callback to set all dynamic configurations
  /// \param config
  /// \param level
  ///
  void configCallback(mars_ros::marsConfig& config, uint32_t level);

  bool common_gps_ref_is_set_{ false };  ///< Indicator that the common reference was set
  Eigen::Vector3d p_wi_init_;            ///< Latest position which will be used to initialize the filter
  Eigen::Quaterniond q_wi_init_;         ///< Latest orientation to initialize the filter
  bool do_state_init_{ false };          ///< Trigger if the filter should be initialized

  ///
  /// \brief init used by the reconfigure GUI to reset the the buffer and sensors
  /// \return
  ///
  bool init();

  ///
  /// \brief set_common_gps_reference called on each GPS measurement update
  /// \param reference
  ///
  /// This function sets common GPS reference coordinates for all GPS
  ///
  void set_common_gps_reference(const mars::GpsCoordinates& reference);

  // Initialize framework components
  std::shared_ptr<mars::ImuSensorClass> imu_sensor_sptr_;  ///< Propagation sensor instance
  std::shared_ptr<mars::CoreState> core_states_sptr_;      ///< Core State instance
  mars::CoreLogic core_logic_;                             ///< Core Logic instance

  // Sensor instances
  std::shared_ptr<mars::GpsSensorClass> gps1_sensor_sptr_;  ///< GPS 1 sensor instance

  // Subscriber
  ros::Subscriber sub_imu_measurement_;   ///< IMU measurement subscriber
  ros::Subscriber sub_gps1_measurement_;  ///< GPS 1 NavSatFixConstPtr measurement subscriber

  // Sensor Callbacks
  ///
  /// \brief ImuMeasurementCallback IMU measurment callback
  /// \param meas
  ///
  ///  Converting the ROS message to MaRS data type and running the propagation sensor routine
  ///
  void ImuMeasurementCallback(const sensor_msgs::ImuConstPtr& meas);

  ///
  /// \brief GpsMeasurementCallback GPS 1 measurement callback
  /// \param meas
  ///
  /// Converting the ROS message to MaRS data type and running the GpsMeasurementUpdate routine
  ///
  void Gps1MeasurementCallback(const sensor_msgs::NavSatFixConstPtr& meas);

  // Publisher
  ros::Publisher pub_ext_core_state_;       ///< Publisher for the Core-State mars_ros::ExtCoreState message
  ros::Publisher pub_core_pose_state_;      ///< Publisher for the Core-State pose stamped message
  ros::Publisher pub_core_odom_state_;      ///< Publisher for the Core-State as Odometry message
  ros::Publisher pub_ext_core_state_lite_;  ///< Publisher for the Core-State mars_ros::ExtCoreStateLite message
  ros::Publisher pub_core_path_;            ///< Publisher for all Core-States in buffer as path message
  MarsPathGen path_generator_;              ///< Generator and storage for nav_msgs::Path

  ros::Publisher pub_gps1_state_;     ///< Publisher for the GPS sensor calibration state
  ros::Publisher pub_gps1_enu_odom_;  ///< Publisher for the GPS ENU position Odometry message

  // Publish groups
  ///
  /// \brief RunCoreStatePublisher Runs on each update sensor routine to publish the core-state
  ///
  /// This publishes the ExtCoreState and the Pose core state
  ///
  void RunCoreStatePublisher();

  // Sensor Updates
  ///
  /// \brief GpsMeasurementUpdate Generic GPS sensor measurement update routine
  /// \param sensor_sptr Pointer of the sensor instance
  /// \param gps_meas Measurement to be used for the update
  /// \param timestamp Timestamp of the measurement
  ///
  void GpsMeasurementUpdate(std::shared_ptr<mars::GpsSensorClass> sensor_sptr, const mars::GpsMeasurementType& gps_meas,
                            const mars::Time& timestamp);
};

#endif  // MARSWRAPPERGPS_H
