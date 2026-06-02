#include "armor.hpp"

#include <algorithm>
#include <cmath>

namespace auto_aim
{
Lightbar::Lightbar(const cv::RotatedRect & rect, std::size_t id) : id(id), rotated_rect(rect)
{
  std::vector<cv::Point2f> corners(4);
  rect.points(&corners[0]);
  std::sort(
    corners.begin(), corners.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });

  center = rect.center;
  top = (corners[0] + corners[1]) / 2;
  bottom = (corners[2] + corners[3]) / 2;
  top2bottom = bottom - top;

  points.emplace_back(top);
  points.emplace_back(bottom);

  width = cv::norm(corners[0] - corners[1]);
  angle = std::atan2(top2bottom.y, top2bottom.x);
  angle_error = std::abs(angle - CV_PI / 2);
  length = cv::norm(top2bottom);
  ratio = length / width;
}

// 传统 CV 构造：两个灯条 → 装甲板
Armor::Armor(const Lightbar & left, const Lightbar & right)
: left(left), right(right), duplicated(false)
{
  color = left.color;
  center = (left.center + right.center) / 2;

  points.emplace_back(left.top);
  points.emplace_back(right.top);
  points.emplace_back(right.bottom);
  points.emplace_back(left.bottom);

  auto left2right = right.center - left.center;
  auto width = cv::norm(left2right);
  auto max_lightbar_length = std::max(left.length, right.length);
  auto min_lightbar_length = std::min(left.length, right.length);
  ratio = width / max_lightbar_length;
  side_ratio = max_lightbar_length / min_lightbar_length;

  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_re = std::abs(left.angle - roll - CV_PI / 2);
  auto right_re = std::abs(right.angle - roll - CV_PI / 2);
  rectangular_error = std::max(left_re, right_re);
}

// 神经网络构造：class_id → 查表得 color/name/type
Armor::Armor(int class_id, float confidence, const cv::Rect & box,
             std::vector<cv::Point2f> armor_keypoints)
: class_id(class_id), confidence(confidence), box(box), points(armor_keypoints)
{
  if (class_id >= 0 && static_cast<size_t>(class_id) < armor_properties.size()) {
    auto [c, n, t] = armor_properties[class_id];
    color = c;
    name = n;
    type = t;
  } else {
    color = blue;
    name = not_armor;
    type = small;
  }
  compute_geometry();
}

// NN + offset 构造（ROI 模式）
Armor::Armor(int class_id, float confidence, const cv::Rect & box,
             std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset)
: class_id(class_id), confidence(confidence), box(box), points(armor_keypoints)
{
  std::transform(
    points.begin(), points.end(), points.begin(),
    [&offset](const cv::Point2f & p) { return p + offset; });
  if (class_id >= 0 && static_cast<size_t>(class_id) < armor_properties.size()) {
    auto [c, n, t] = armor_properties[class_id];
    color = c;
    name = n;
    type = t;
  } else {
    color = blue;
    name = not_armor;
    type = small;
  }
  compute_geometry();
}

// YOLOv5 构造（color_id + num_id 分开输入）
Armor::Armor(int color_id, int num_id, float confidence, const cv::Rect & box,
             std::vector<cv::Point2f> armor_keypoints)
: confidence(confidence), box(box), points(armor_keypoints)
{
  color = color_id == 0 ? blue : color_id == 1 ? red : extinguish;
  name = num_id == 0 ? sentry : num_id > 5 ? ArmorName(num_id) : ArmorName(num_id - 1);
  type = num_id == 1 ? big : small;
  compute_geometry();
}

// YOLOv5 + offset 构造（ROI 模式）
Armor::Armor(int color_id, int num_id, float confidence, const cv::Rect & box,
             std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset)
: confidence(confidence), box(box), points(armor_keypoints)
{
  std::transform(
    points.begin(), points.end(), points.begin(),
    [&offset](const cv::Point2f & p) { return p + offset; });
  color = color_id == 0 ? blue : color_id == 1 ? red : extinguish;
  name = num_id == 0 ? sentry : num_id > 5 ? ArmorName(num_id) : ArmorName(num_id - 1);
  type = num_id == 1 ? big : small;
  compute_geometry();
}

void Armor::compute_geometry()
{
  // points 顺序: 0=左上 1=右上 2=右下 3=左下
  center = (points[0] + points[1] + points[2] + points[3]) / 4;

  auto left_width = cv::norm(points[0] - points[3]);
  auto right_width = cv::norm(points[1] - points[2]);
  auto max_width = std::max(left_width, right_width);

  auto top_length = cv::norm(points[0] - points[1]);
  auto bottom_length = cv::norm(points[3] - points[2]);
  auto max_length = std::max(top_length, bottom_length);

  auto left_center = (points[0] + points[3]) / 2;
  auto right_center = (points[1] + points[2]) / 2;
  auto left2right = right_center - left_center;

  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_re = std::abs(
    std::atan2((points[3] - points[0]).y, (points[3] - points[0]).x) - roll - CV_PI / 2);
  auto right_re = std::abs(
    std::atan2((points[2] - points[1]).y, (points[2] - points[1]).x) - roll - CV_PI / 2);
  rectangular_error = std::max(left_re, right_re);

  ratio = max_length / max_width;
}

}  // namespace auto_aim
