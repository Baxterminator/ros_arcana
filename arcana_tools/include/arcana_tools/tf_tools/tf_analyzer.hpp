#ifndef ARCANA_TF_ANALYZER_HPP
#define ARCANA_TF_ANALYZER_HPP

/*
  =============================================================================
                             TF2 Packets Analyzer

  This node is aimed to provide help when troubleshooting with tf2.

  Author: Geoffrey "Meltwin" Côte (https://github.com/Meltwin)
  Copyright: (c) Meltwin - 2025
  Distributed under the MIT Licence
  =============================================================================
*/

#include <algorithm>
#include <arcana_cpp/circular_buffer.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cstdio>
#include <fmt/format.h>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>

using builtin_interfaces::msg::Time;
using geometry_msgs::msg::TransformStamped;
using tf2_msgs::msg::TFMessage;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace arcana {

/**
  Structure that allow hertz computation
*/
template <size_t N> struct HertzComputation {
  enum State { PUBLISHING, TIMEOUT };

  CircularBuffer<double, N> time_buffer;
  double cumul_time = 0.0;
  double last_time = 0.0;
  double last_ros_time = 0.0;
  State state = TIMEOUT;

  void add_time(const Time &time, const Time &ros_time) {
    if (time_buffer.is_full())
      cumul_time -= time_buffer.pop();

    // Compute time span
    double dtime = computeTime(time);
    double dt = (last_time < 1E-2) ? 0.0 : dtime - last_time;

    // Increase internal buffers
    cumul_time += dt; // Save msg dt
    last_time = dtime;
    last_ros_time = computeTime(ros_time);
    time_buffer.push(dt);
    state = PUBLISHING;
  }

  void update(const Time &ros_time) {
    if (state == TIMEOUT)
      return;
    if (time_buffer.size() == 0)
      return;
    // Decide to time out or not (10x the avg duration)
    if (computeTime(ros_time) - last_ros_time >
        10 * cumul_time / time_buffer.size()) {
      state = TIMEOUT;
      cumul_time = 0.0;
      time_buffer.flush();
    }
  }

  inline std::string get_hertz() const {
    if (state == TIMEOUT)
      return "0 Hz";
    if (cumul_time < 1E-4)
      return "NaN Hz";
    return fmt::format("{:> 8.3g} Hz", time_buffer.size() / cumul_time);
  }

  std::string str() const {
    return fmt::format("Hz[{:12s} | TO={:d}]", get_hertz(), state == TIMEOUT);
  }

  static double computeTime(const Time &time) {
    return time.sec + 1E-9 * time.nanosec;
  }
};

/** Structure to actually store the transform info */
struct TFInfo {

  static constexpr size_t N{6};
  std::string parent_link;
  std::string child_link;

  HertzComputation<N> header_hertz;
  HertzComputation<N> ros_time_hertz;

  rclcpp::Logger logger = rclcpp::get_logger("TFInfo");

  TFInfo(const std::string parent, const std::string child)
      : parent_link(parent), child_link(child) {}

  void add_time(const Time &header_time, const Time &ros_time) {
    header_hertz.add_time(header_time, ros_time);
    ros_time_hertz.add_time(ros_time, ros_time);
  }

  void update(const Time &ros_time) {
    header_hertz.update(ros_time);
    ros_time_hertz.update(ros_time);
  }

  std::string str() const {
    return fmt::format("H={:s} ROS={:s}", header_hertz.str(),
                       ros_time_hertz.str());
  }
};

/**
  Actual ROS2 node
*/
struct TFAnalyzer : rclcpp::Node {
  typedef std::vector<TFInfo> TfInfoList;
  typedef TfInfoList::iterator TFInfoIter;

  TFAnalyzer() : rclcpp::Node("tf_analyzer") {
    _tf_sub = create_subscription<TFMessage>(
        "/tf", 10, std::bind(&TFAnalyzer::tf_callback, this, _1));
    _display_timer =
        create_wall_timer(1s, std::bind(&TFAnalyzer::display_info, this));
  }

  void tf_callback(const TFMessage::SharedPtr msg) {

    // Look for the right transform
    for (auto &tf : msg->transforms) {
      const auto pred = [&tf](const TFInfo &info) {
        return (tf.header.frame_id.compare(info.parent_link) == 0) &&
               (tf.child_frame_id.compare(info.child_link) == 0);
      };

      // Find transform and fill info
      if (auto it = std::find_if(_tfinfos.begin(), _tfinfos.end(), pred);
          it == _tfinfos.end()) {
        RCLCPP_INFO(get_logger(), "Register new transform \"%s\" -> \"%s\"",
                    tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
        _tfinfos.push_back(TFInfo(tf.header.frame_id, tf.child_frame_id));

        auto last_it = (_tfinfos.end() - 1);
        fill_info(last_it, tf);
      } else {
        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Filling in received transform for transform \"%s\" -> \"%s\"!",
            tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
        fill_info(it, tf);
      }
    }
  }

  void fill_info(TFInfoIter &iter, const TransformStamped &tf) {
    iter->add_time(tf.header.stamp, get_clock()->now());
  }

  void display_info() {
    RCLCPP_INFO(get_logger(), "----------------------------------------------");
    for (auto &info : _tfinfos) {
      info.update(get_clock()->now());
      RCLCPP_INFO(get_logger(), "%s",
                  fmt::format("TF {:16s} -> {:16s} : {:s}",
                              info.parent_link.c_str(), info.child_link.c_str(),
                              info.str().c_str())
                      .c_str());
    }
    RCLCPP_INFO(get_logger(), "----------------------------------------------");
  }

protected:
  rclcpp::Subscription<TFMessage>::SharedPtr _tf_sub;
  rclcpp::TimerBase::SharedPtr _display_timer;
  TfInfoList _tfinfos;
};

} // namespace arcana

#endif