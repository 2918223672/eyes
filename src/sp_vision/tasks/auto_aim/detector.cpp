#include "detector.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/utils/img_tools.hpp"
#include "tools/utils/logger.hpp"

namespace auto_aim
{
Detector::Detector(const std::string & config_path, bool debug) : debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  threshold_ = yaml["threshold"].as<double>();
  max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3;
  min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
  max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();
  min_lightbar_length_ = yaml["min_lightbar_length"].as<double>();
  min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();
  max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();
  max_side_ratio_ = yaml["max_side_ratio"].as<double>();
  max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3;

  save_path_ = "patterns";
  std::filesystem::create_directory(save_path_);
}

std::list<Armor> Detector::detect(const cv::Mat & bgr_img, int frame_count)
{
  // 1. 灰度化 + 二值化
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);

  // 2. 找轮廓 → 灯条候选
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    if (!check_geometry(lightbar)) continue;

    lightbar.color = get_color(bgr_img, contour);
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  // 3. 灯条从左到右排序 → 两两配对
  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  std::list<Armor> armors;
  for (auto left = lightbars.begin(); left != lightbars.end(); left++) {
    for (auto right = std::next(left); right != lightbars.end(); right++) {
      if (left->color != right->color) continue;

      auto armor = Armor(*left, *right);
      if (!check_geometry(armor)) continue;

      armor.type = get_type(armor);
      armor.center_norm = get_center_norm(bgr_img, armor.center);
      armors.emplace_back(armor);
    }
  }

  // 4. 去重：共用灯条的装甲板只保留一个
  for (auto armor1 = armors.begin(); armor1 != armors.end(); armor1++) {
    for (auto armor2 = std::next(armor1); armor2 != armors.end(); armor2++) {
      if (
        armor1->left.id != armor2->left.id && armor1->left.id != armor2->right.id &&
        armor1->right.id != armor2->left.id && armor1->right.id != armor2->right.id) {
        continue;
      }

      // 重叠：保留 ROI 小的
      if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id) {
        auto area1 = armor1->pattern.cols * armor1->pattern.rows;
        auto area2 = armor2->pattern.cols * armor2->pattern.rows;
        area1 < area2 ? armor2->duplicated = true : armor1->duplicated = true;
      }

      // 相连：保留置信度高的
      if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id) {
        armor1->confidence < armor2->confidence ? armor1->duplicated = true
                                                 : armor2->duplicated = true;
      }
    }
  }

  armors.remove_if([](const Armor & a) { return a.duplicated; });

  if (debug_) show_result(binary_img, bgr_img, lightbars, armors, frame_count);

  return armors;
}

// NN 检测结果精修：在 ROI 内用传统 CV 找到实际灯条角点
bool Detector::detect(Armor & armor, const cv::Mat & bgr_img)
{
  auto tl = armor.points[0];
  auto tr = armor.points[1];
  auto br = armor.points[2];
  auto bl = armor.points[3];

  auto lt2b = bl - tl;
  auto rt2b = br - tr;
  auto tl1 = (tl + bl) / 2 - lt2b;
  auto bl1 = (tl + bl) / 2 + lt2b;
  auto br1 = (tr + br) / 2 + rt2b;
  auto tr1 = (tr + br) / 2 - rt2b;
  auto tl2tr = tr1 - tl1;
  auto bl2br = br1 - bl1;
  auto tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr;
  auto tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr;
  auto bl2 = (bl1 + br) / 2 - 0.75 * bl2br;
  auto br2 = (bl1 + br) / 2 + 0.75 * bl2br;

  std::vector<cv::Point> points = {tl2, tr2, br2, bl2};
  auto armor_rotaterect = cv::minAreaRect(points);
  cv::Rect boundingBox = armor_rotaterect.boundingRect();

  if (
    boundingBox.x < 0 || boundingBox.y < 0 ||
    boundingBox.x + boundingBox.width > bgr_img.cols ||
    boundingBox.y + boundingBox.height > bgr_img.rows) {
    return false;
  }

  cv::Mat armor_roi = bgr_img(boundingBox);
  if (armor_roi.empty()) return false;

  cv::Mat gray_img;
  cv::cvtColor(armor_roi, gray_img, cv::COLOR_BGR2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);
    if (!check_geometry(lightbar)) continue;
    lightbar.color = get_color(bgr_img, contour);
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  if (lightbars.size() < 2) return false;

  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  Lightbar * closest_left = nullptr;
  Lightbar * closest_right = nullptr;
  float min_dist_tl_bl = std::numeric_limits<float>::max();
  float min_dist_br_tr = std::numeric_limits<float>::max();

  for (auto & lb : lightbars) {
    float dist_tl_bl =
      cv::norm(tl - (lb.top + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(bl - (lb.bottom + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (dist_tl_bl < min_dist_tl_bl) {
      min_dist_tl_bl = dist_tl_bl;
      closest_left = &lb;
    }
    float dist_br_tr =
      cv::norm(br - (lb.bottom + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(tr - (lb.top + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (dist_br_tr < min_dist_br_tr) {
      min_dist_br_tr = dist_br_tr;
      closest_right = &lb;
    }
  }

  if (closest_left && closest_right && min_dist_br_tr + min_dist_tl_bl < 15) {
    armor.points[0] = closest_left->top + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[1] = closest_right->top + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[2] = closest_right->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[3] = closest_left->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
    return true;
  }

  return false;
}

// ===== 几何检查 =====

bool Detector::check_geometry(const Lightbar & l) const
{
  return l.angle_error < max_angle_error_ && l.ratio > min_lightbar_ratio_ &&
         l.ratio < max_lightbar_ratio_ && l.length > min_lightbar_length_;
}

bool Detector::check_geometry(const Armor & a) const
{
  return a.ratio > min_armor_ratio_ && a.ratio < max_armor_ratio_ &&
         a.side_ratio < max_side_ratio_ && a.rectangular_error < max_rectangular_error_;
}

// ===== 颜色提取 =====

Color Detector::get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const
{
  int red_sum = 0, blue_sum = 0;

  for (const auto & point : contour) {
    red_sum += bgr_img.at<cv::Vec3b>(point)[2];
    blue_sum += bgr_img.at<cv::Vec3b>(point)[0];
  }

  return blue_sum > red_sum ? Color::blue : Color::red;
}

// ===== 装甲板 ROI =====

cv::Mat Detector::get_pattern(const cv::Mat & bgr_img, const Armor & armor) const
{
  auto tl = armor.left.center - armor.left.top2bottom * 1.125;
  auto bl = armor.left.center + armor.left.top2bottom * 1.125;
  auto tr = armor.right.center - armor.right.top2bottom * 1.125;
  auto br = armor.right.center + armor.right.top2bottom * 1.125;

  auto roi_left = std::max<int>(std::min(tl.x, bl.x), 0);
  auto roi_top = std::max<int>(std::min(tl.y, tr.y), 0);
  auto roi_right = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);
  auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows);

  return bgr_img(cv::Rect(cv::Point(roi_left, roi_top), cv::Point(roi_right, roi_bottom)));
}

// ===== 大小装甲板判断 =====

ArmorType Detector::get_type(const Armor & armor)
{
  if (armor.ratio > 3.0) return ArmorType::big;
  if (armor.ratio < 2.5) return ArmorType::small;

  return ArmorType::small;
}

// ===== 归一化坐标 =====

cv::Point2f Detector::get_center_norm(
  const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  return {center.x / bgr_img.cols, center.y / bgr_img.rows};
}

// ===== 可视化 =====

void Detector::show_result(
  const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
  const std::list<Armor> & armors, int frame_count) const
{
  auto detection = bgr_img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

  for (const auto & l : lightbars) {
    auto info = fmt::format(
      "{:.1f} {:.1f} {:.1f} {}", l.angle_error * 57.3, l.ratio, l.length, COLORS[l.color]);
    tools::draw_text(detection, info, l.top, {0, 255, 255});
    tools::draw_points(detection, l.points, {0, 255, 255}, 3);
  }

  for (const auto & a : armors) {
    auto info = fmt::format(
      "{:.2f} {:.2f} {:.1f} {} {}", a.ratio, a.side_ratio, a.rectangular_error * 57.3,
      ARMOR_TYPES[a.type], COLORS[a.color]);
    tools::draw_points(detection, a.points, {0, 255, 0});
    tools::draw_text(detection, info, a.center, {0, 255, 0});
  }

  cv::Mat binary_small, detection_small;
  cv::resize(binary_img, binary_small, {}, 0.5, 0.5);
  cv::resize(detection, detection_small, {}, 0.5, 0.5);

  cv::imshow("binary", binary_small);
  cv::imshow("detection", detection_small);
}

}  // namespace auto_aim
