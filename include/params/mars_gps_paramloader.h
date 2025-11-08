// Copyright (C) 2022 Martin Scheiber and Christian Brommer, Control of Networked Systems,
// University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <martin.scheiber@ieee.org>
// and <christian.brommer@ieee.org>.

#ifndef MARSGPS_LOADER_H
#define MARSGPS_LOADER_H

#include <ros/node_handle.h>
#include <XmlRpc.h>

#include <Eigen/Dense>

#define PARAM_PRINTER(args) std::cout << "[ParamLoader] " << args;

class ParamLoad
{
public:
  bool publish_on_propagation_{ true };   ///< Set true to publish the core state on propagation
  bool use_ros_time_now_{ false };        ///< Set to true to use rostime now for all sensor updates
  bool verbose_output_{ false };          ///< If true, all verbose infos are printed
  bool verbose_ooo_{ true };              ///< If true, only out of order verbose msgs are printed
  bool discard_ooo_prop_meas_{ false };   ///< If true, all out of order propagation sensor meas are discarded
  bool pub_cov_{ true };                  ///< Publish covariances in the ext core state message if true
  bool pub_path_{ false };                ///< Publish all core states as nav_msgs::Path (for rviz)
  uint32_t buffer_size_{ 2000 };          ///< Set mars buffersize

  bool use_tcpnodelay_{ true };  ///< Use tcp no delay for the ROS msg. system
  bool bypass_init_service_{ false };

  uint32_t pub_cb_buffer_size_{ 1 };         ///< Callback buffersize for all outgoing topics
  uint32_t sub_imu_cb_buffer_size_{ 200 };   ///< Callback buffersize for propagation sensor measurements
  uint32_t sub_sensor_cb_buffer_size_{ 1 };  ///< Callback buffersize for all non-propagation sensor measurements

  bool publish_gps_enu_{ false };         ///< Publish GPS as ENU in the ref. frame used by the filter
  bool enable_manual_yaw_init_{ false };  ///< Initialize the yaw based on 'yaw_init_deg_'
  double pitch_init_deg_{ 0 };            ///< Pitch for core state init
  double roll_init_deg_{ 0 };             ///< Roll for core state init
  double yaw_init_deg_{ 0 };              ///< Yaw for core state init if 'enable_manual_yaw_init_' is true
  Eigen::Vector3d gyro_bias_init_{ 0, 0, 0 };        ///< Gyro bias initialization [rad/s]
  Eigen::Vector3d antenna_lever_arm_{ 0, 0, 0 };     ///< Antenna lever arm in sensor frame [m]

  double g_rate_noise_;
  double g_bias_noise_;
  double a_noise_;
  double a_bias_noise_;

  Eigen::Vector3d core_init_cov_p_;
  Eigen::Vector3d core_init_cov_v_;
  Eigen::Vector3d core_init_cov_q_;
  Eigen::Vector3d core_init_cov_bw_;
  Eigen::Vector3d core_init_cov_ba_;

  Eigen::Vector3d gps1_pos_meas_noise_;
  Eigen::Vector3d gps1_vel_meas_noise_;
  bool gps1_use_dyn_meas_noise_{ false };
  Eigen::Vector3d gps1_cal_ig_;
  Eigen::Vector3d gps1_state_init_cov_;

  void check_size(const int& size_in, const int& size_comp)
  {
    if (size_comp != size_in)
    {
      std::cerr << "YAML array with wrong size" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  template <int _Rows>
  void check_and_load(Eigen::Matrix<double, _Rows, 1>& vec, const ros::NodeHandle& nh, const std::string& name)
  {
    std::vector<double> tmp_vec;
    std::string nh_namespace = nh.getNamespace();
    std::string full_param_path = nh_namespace + "/" + name;
    
    // Try relative path first with nh.param()
    nh.param(name, tmp_vec, std::vector<double>());
    std::cerr << "[ParamLoader] DEBUG: relative nh.param('" << name << "') -> size=" << tmp_vec.size();
    if (tmp_vec.size() > 0) {
      std::cerr << " [" << tmp_vec[0];
      for (size_t i = 1; i < tmp_vec.size(); i++) std::cerr << ", " << tmp_vec[i];
      std::cerr << "]";
    }
    std::cerr << std::endl;
    
    // If relative path didn't work, try absolute with ros::param::get()
    if (tmp_vec.size() == 0)
    {
      bool found = ros::param::get(full_param_path, tmp_vec);
      std::cerr << "[ParamLoader] DEBUG: absolute ros::param::get('" << full_param_path << "') -> found=" 
                << (found ? "true" : "false") << " size=" << tmp_vec.size();
      if (tmp_vec.size() > 0) {
        std::cerr << " [" << tmp_vec[0];
        for (size_t i = 1; i < tmp_vec.size(); i++) std::cerr << ", " << tmp_vec[i];
        std::cerr << "]";
      }
      std::cerr << std::endl;
    }
    
    if (tmp_vec.size() != _Rows)
    {
      std::cerr << "[ParamLoader] " << name << ": YAML array with wrong size (which is " << tmp_vec.size() << ")"
                << std::endl;
      // Keep default value if size is wrong
      return;
    }
    vec = Eigen::Map<Eigen::Matrix<double, _Rows, 1> >(tmp_vec.data());
    std::cerr << "[ParamLoader] " << name << " loaded successfully: [" << vec(0) << ", " << vec(1) << ", " << vec(2) << "]" << std::endl;
  }

  ParamLoad(const ros::NodeHandle& nh)
  {
    std::cerr << "[ParamLoader] Constructor called with NodeHandle namespace: '" << nh.getNamespace() << "'" << std::endl;
    
    // List all available parameters to debug timing issue
    std::vector<std::string> all_params;
    ros::param::getParamNames(all_params);
    std::cerr << "[ParamLoader] DEBUG: Total parameters on server: " << all_params.size() << std::endl;
    for (const auto& p : all_params) {
      if (p.find("mars_gps_node") != std::string::npos) {
        std::cerr << "[ParamLoader]   Found: " << p << std::endl;
      }
    }
    
    publish_on_propagation_ = nh.param<bool>("pub_on_prop", publish_on_propagation_);
    use_ros_time_now_ = nh.param<bool>("use_ros_time_now", use_ros_time_now_);
    verbose_output_ = nh.param<bool>("verbose", verbose_output_);
    verbose_ooo_ = nh.param<bool>("verbose_out_of_order", verbose_ooo_);
    discard_ooo_prop_meas_ = nh.param<bool>("discard_ooo_prop_meas", discard_ooo_prop_meas_);
    pub_cov_ = nh.param<bool>("pub_cov", pub_cov_);
    pub_path_ = nh.param<bool>("pub_path", pub_path_);
    buffer_size_ = nh.param<int>("buffer_size", buffer_size_);

    use_tcpnodelay_ = nh.param<bool>("use_tcpnodelay", use_tcpnodelay_);
    bypass_init_service_ = nh.param<bool>("bypass_init_service", bypass_init_service_);

    pub_cb_buffer_size_ = uint32_t(nh.param<int>("pub_cb_buffer_size", int(pub_cb_buffer_size_)));
    sub_imu_cb_buffer_size_ = uint32_t(nh.param<int>("sub_imu_cb_buffer_size", int(sub_imu_cb_buffer_size_)));
    sub_sensor_cb_buffer_size_ = uint32_t(nh.param<int>("sub_sensor_cb_buffer_size", int(sub_sensor_cb_buffer_size_)));

    publish_gps_enu_ = nh.param<bool>("publish_gps_enu", publish_gps_enu_);
    enable_manual_yaw_init_ = nh.param<bool>("enable_manual_yaw_init", enable_manual_yaw_init_);
    nh.param("pitch_init_deg", pitch_init_deg_, double());
    nh.param("roll_init_deg", roll_init_deg_, double());
    nh.param("yaw_init_deg", yaw_init_deg_, double());
    
    std::cerr << "[ParamLoader] DEBUG: pitch_init_deg=" << pitch_init_deg_ 
              << ", roll_init_deg=" << roll_init_deg_ 
              << ", yaw_init_deg=" << yaw_init_deg_ << std::endl;
    
    // Direct test of what the parameter server has
    std::vector<double> test_gyro, test_antenna;
    bool found_gyro = ros::param::get("/mars_gps_node/gyro_bias_init", test_gyro);
    bool found_antenna = ros::param::get("/mars_gps_node/antenna_lever_arm", test_antenna);
    std::cerr << "[ParamLoader] DEBUG: Direct global lookup - gyro_bias_init size=" << test_gyro.size() 
              << " antenna_lever_arm size=" << test_antenna.size() << std::endl;
    
    // Try getting as XmlRpc value to see what it actually is
    XmlRpc::XmlRpcValue gyro_xml, antenna_xml;
    if (nh.getParam("gyro_bias_init", gyro_xml)) {
      std::cerr << "[ParamLoader] DEBUG: gyro_bias_init via getParam - type=" << (int)gyro_xml.getType() 
                << " (0=invalid,1=bool,2=int,3=double,4=string,5=datetime,6=base64,7=array,8=struct)" << std::endl;
    } else {
      std::cerr << "[ParamLoader] DEBUG: gyro_bias_init via getParam - NOT FOUND" << std::endl;
    }
    
    check_and_load<3>(gyro_bias_init_, nh, "gyro_bias_init");
    check_and_load<3>(antenna_lever_arm_, nh, "antenna_lever_arm");

    nh.param("gyro_rate_noise", g_rate_noise_, double());
    nh.param("gyro_bias_noise", g_bias_noise_, double());
    nh.param("acc_noise", a_noise_, double());
    nh.param("acc_bias_noise", a_bias_noise_, double());

    std::vector<double> core_init_cov_p;
    nh.param("core_init_cov_p", core_init_cov_p, std::vector<double>());
    check_size(core_init_cov_p.size(), 3);
    core_init_cov_p_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_p.data());

    std::vector<double> core_init_cov_v;
    nh.param("core_init_cov_v", core_init_cov_v, std::vector<double>());
    check_size(core_init_cov_v.size(), 3);
    core_init_cov_v_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_v.data());

    std::vector<double> core_init_cov_q;
    nh.param("core_init_cov_q", core_init_cov_q, std::vector<double>());
    check_size(core_init_cov_q.size(), 3);
    core_init_cov_q_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_q.data());

    std::vector<double> core_init_cov_bw;
    nh.param("core_init_cov_bw", core_init_cov_bw, std::vector<double>());
    check_size(core_init_cov_bw.size(), 3);
    core_init_cov_bw_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_bw.data());

    std::vector<double> core_init_cov_ba;
    nh.param("core_init_cov_ba", core_init_cov_ba, std::vector<double>());
    check_size(core_init_cov_ba.size(), 3);
    core_init_cov_ba_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(core_init_cov_ba.data());

    // GPS Sensor
    std::vector<double> gps1_pos_meas_noise;
    nh.param("gps1_pos_meas_noise", gps1_pos_meas_noise, std::vector<double>());
    check_size(gps1_pos_meas_noise.size(), 3);
    gps1_pos_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_pos_meas_noise.data());

    std::vector<double> gps1_vel_meas_noise;
    nh.param("gps1_vel_meas_noise", gps1_vel_meas_noise, std::vector<double>());
    check_size(gps1_vel_meas_noise.size(), 3);
    gps1_vel_meas_noise_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_vel_meas_noise.data());

    gps1_use_dyn_meas_noise_ = nh.param<bool>("gps1_use_dyn_meas_noise", gps1_use_dyn_meas_noise_);

    std::vector<double> gps1_cal_ig;
    nh.param("gps1_cal_ig", gps1_cal_ig, std::vector<double>());
    check_size(gps1_cal_ig.size(), 3);
    gps1_cal_ig_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_cal_ig.data());

    std::vector<double> gps1_state_init_cov;
    nh.param("gps1_state_init_cov", gps1_state_init_cov, std::vector<double>());
    check_size(gps1_state_init_cov.size(), 3);
    gps1_state_init_cov_ = Eigen::Map<Eigen::Matrix<double, 3, 1> >(gps1_state_init_cov.data());
  }
};

#endif  // MARSGPS_LOADER_H
