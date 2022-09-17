// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PERCEPTION_MSGS__MARKER_HPP_
#define PERCEPTION_MSGS__MARKER_HPP_

#include <color_names/color_names.hpp>
#include <perception_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

visualization_msgs::msg::MarkerArray toMarker(
  const std::vector<perception_msgs::msg::Detection2D> & detections, const rclcpp::Time & now,
  const std::string & camera_optical_frame, const std::string & detection_color = "limegreen",
  const std::string & frustum_color = "cyan")
{
  visualization_msgs::msg::MarkerArray marker;
  visualization_msgs::msg::Marker frustum_marker;
  frustum_marker.header.stamp = now;
  frustum_marker.header.frame_id = camera_optical_frame;
  frustum_marker.id = 0;
  frustum_marker.ns = "frustum";
  frustum_marker.action = visualization_msgs::msg::Marker::ADD;
  frustum_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  frustum_marker.scale.x = 0.01;
  frustum_marker.scale.y = 0.01;
  frustum_marker.scale.z = 0.01;
  frustum_marker.color = color_names::makeColorMsg(frustum_color, 1.0);
  geometry_msgs::msg::Point point_origin;
  point_origin.x = 0;
  point_origin.y = 0;
  point_origin.z = 0;
  cv::Point3d point_lu = cam_model_.projectPixelTo3dRay(cv::Point2d(0, 0));
  geometry_msgs::msg::Point point_lu_msg;
  point_lu_msg.x = point_lu.x;
  point_lu_msg.y = point_lu.y;
  point_lu_msg.z = point_lu.z;
  frustum_marker.points.emplace_back(point_origin);
  frustum_marker.points.emplace_back(point_lu_msg);
  cv::Point3d point_ru = cam_model_.projectPixelTo3dRay(cv::Point2d(horizontal_pixels_, 0));
  geometry_msgs::msg::Point point_ru_msg;
  point_ru_msg.x = point_ru.x;
  point_ru_msg.y = point_ru.y;
  point_ru_msg.z = point_ru.z;
  frustum_marker.points.emplace_back(point_origin);
  frustum_marker.points.emplace_back(point_ru_msg);
  cv::Point3d point_lb = cam_model_.projectPixelTo3dRay(cv::Point2d(0, vertical_pixels_));
  geometry_msgs::msg::Point point_lb_msg;
  point_lb_msg.x = point_lb.x;
  point_lb_msg.y = point_lb.y;
  point_lb_msg.z = point_lb.z;
  frustum_marker.points.emplace_back(point_origin);
  frustum_marker.points.emplace_back(point_lb_msg);
  cv::Point3d point_rb =
    cam_model_.projectPixelTo3dRay(cv::Point2d(horizontal_pixels_, vertical_pixels_));
  geometry_msgs::msg::Point point_rb_msg;
  point_rb_msg.x = point_rb.x;
  point_rb_msg.y = point_rb.y;
  point_rb_msg.z = point_rb.z;
  frustum_marker.points.emplace_back(point_origin);
  frustum_marker.points.emplace_back(point_rb_msg);
  // markers for edge
  frustum_marker.points.emplace_back(point_lu_msg);
  frustum_marker.points.emplace_back(point_ru_msg);
  frustum_marker.points.emplace_back(point_ru_msg);
  frustum_marker.points.emplace_back(point_rb_msg);
  frustum_marker.points.emplace_back(point_rb_msg);
  frustum_marker.points.emplace_back(point_lb_msg);
  frustum_marker.points.emplace_back(point_lb_msg);
  frustum_marker.points.emplace_back(point_lu_msg);
  frustum_marker.frame_locked = true;
  marker.markers.emplace_back(frustum_marker);
  visualization_msgs::msg::Marker detection_marker;
  detection_marker.header.stamp = now;
  detection_marker.header.frame_id = camera_optical_frame;
  detection_marker.id = 0;
  detection_marker.ns = "detection";
  detection_marker.action = visualization_msgs::msg::Marker::ADD;
  detection_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  detection_marker.scale.x = 0.01;
  detection_marker.scale.y = 0.01;
  detection_marker.scale.z = 0.01;
  detection_marker.color = color_names::makeColorMsg(detection_color, 1.0);
  detection_marker.frame_locked = true;
  for (const auto & detection : detections) {
    cv::Point2d point_lu_obj(
      detection.bbox.center.x - detection.bbox.size_x * 0.5,
      detection.bbox.center.y - detection.bbox.size_y * 0.5);
    cv::Point3d lu_ray = cam_model_.projectPixelTo3dRay(point_lu_obj);
    geometry_msgs::msg::Point point_lu_obj_msg;
    point_lu_obj_msg.x = lu_ray.x;
    point_lu_obj_msg.y = lu_ray.y;
    point_lu_obj_msg.z = lu_ray.z;
    cv::Point2d point_ru_point(
      detection.bbox.center.x + detection.bbox.size_x * 0.5,
      detection.bbox.center.y - detection.bbox.size_y * 0.5);
    cv::Point3d ru_ray = cam_model_.projectPixelTo3dRay(point_ru_point);
    geometry_msgs::msg::Point point_ru_obj_msg;
    point_ru_obj_msg.x = ru_ray.x;
    point_ru_obj_msg.y = ru_ray.y;
    point_ru_obj_msg.z = ru_ray.z;
    cv::Point2d point_rb_point(
      detection.bbox.center.x + detection.bbox.size_x * 0.5,
      detection.bbox.center.y + detection.bbox.size_y * 0.5);
    cv::Point3d rb_ray = cam_model_.projectPixelTo3dRay(point_rb_point);
    geometry_msgs::msg::Point point_rb_obj_msg;
    point_rb_obj_msg.x = rb_ray.x;
    point_rb_obj_msg.y = rb_ray.y;
    point_rb_obj_msg.z = rb_ray.z;
    cv::Point2d point_lb_point(
      detection.bbox.center.x - detection.bbox.size_x * 0.5,
      detection.bbox.center.y + detection.bbox.size_y * 0.5);
    cv::Point3d lb_ray = cam_model_.projectPixelTo3dRay(point_lb_point);
    geometry_msgs::msg::Point point_lb_obj_msg;
    point_lb_obj_msg.x = lb_ray.x;
    point_lb_obj_msg.y = lb_ray.y;
    point_lb_obj_msg.z = lb_ray.z;
    detection_marker.points.emplace_back(point_lu_obj_msg);
    detection_marker.points.emplace_back(point_ru_obj_msg);
    detection_marker.points.emplace_back(point_ru_obj_msg);
    detection_marker.points.emplace_back(point_rb_obj_msg);
    detection_marker.points.emplace_back(point_rb_obj_msg);
    detection_marker.points.emplace_back(point_lb_obj_msg);
    detection_marker.points.emplace_back(point_lb_obj_msg);
    detection_marker.points.emplace_back(point_lu_obj_msg);
  }
  marker.markers.emplace_back(detection_marker);
  return marker;
}

#endif  // PERCEPTION_MSGS__MARKER_HPP_
