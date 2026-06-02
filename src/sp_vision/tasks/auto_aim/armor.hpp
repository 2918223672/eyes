#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <tuple>
#include <vector>

namespace auto_aim
{
enum Color { red, blue, extinguish, purple };
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType { big, small };
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName { one, two, three, four, five, sentry, outpost, base, not_armor };
const std::vector<std::string> ARMOR_NAMES = {
  "one", "two", "three", "four", "five", "sentry", "outpost", "base", "not_armor"};

enum ArmorPriority { first = 1, second, third, forth, fifth };

const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},    {red, sentry, small},    {extinguish, sentry, small},
  {blue, one, small},       {red, one, small},       {extinguish, one, small},
  {blue, two, small},       {red, two, small},       {extinguish, two, small},
  {blue, three, small},     {red, three, small},     {extinguish, three, small},
  {blue, four, small},      {red, four, small},      {extinguish, four, small},
  {blue, five, small},      {red, five, small},      {extinguish, five, small},
  {blue, outpost, small},   {red, outpost, small},   {extinguish, outpost, small},
  {blue, base, big},        {red, base, big},        {extinguish, base, big},       {purple, base, big},
  {blue, base, small},      {red, base, small},      {extinguish, base, small},     {purple, base, small},
  {blue, three, big},       {red, three, big},       {extinguish, three, big},
  {blue, four, big},        {red, four, big},        {extinguish, four, big},
  {blue, five, big},        {red, five, big},        {extinguish, five, big}};

struct Lightbar
{
  std::size_t id;
  Color color;
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle, angle_error, length, width, ratio;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() = default;
};

struct Armor
{
  Color color;
  Lightbar left, right;
  cv::Point2f center;
  cv::Point2f center_norm;
  std::vector<cv::Point2f> points;

  double ratio;
  double side_ratio;
  double rectangular_error;

  ArmorType type;
  ArmorName name;
  ArmorPriority priority;
  int class_id;
  cv::Rect box;
  cv::Mat pattern;
  double confidence;
  bool duplicated;

  Eigen::Vector3d xyz_in_gimbal;
  Eigen::Vector3d xyz_in_world;
  Eigen::Vector3d ypr_in_gimbal;
  Eigen::Vector3d ypr_in_world;
  Eigen::Vector3d ypd_in_world;
  double yaw_raw;

  Armor(const Lightbar & left, const Lightbar & right);
  Armor(int class_id, float confidence, const cv::Rect & box,
        std::vector<cv::Point2f> armor_keypoints);
  Armor(int class_id, float confidence, const cv::Rect & box,
        std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
  Armor(int color_id, int num_id, float confidence, const cv::Rect & box,
        std::vector<cv::Point2f> armor_keypoints);
  Armor(int color_id, int num_id, float confidence, const cv::Rect & box,
        std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);

private:
  void compute_geometry();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
