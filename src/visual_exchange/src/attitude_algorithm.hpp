#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/util/ForwardDeclarations.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <queue>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include "../include/pnp/pnp_solver.hpp"

// #define IMG_SHOW
namespace EngineerVisual {

class AttitudeAlgorithm {
public:
  AttitudeAlgorithm() : camera_points_(4), world_points_(4), pnpsolver() {
    //初始化相机参数
    pnpsolver.SetCameraMatrix(1.722231837421459e+03, 1.724876404292754e+03,
                              7.013056440882832e+02, 5.645821718351237e+02);
    //设置畸变参数
    pnpsolver.SetDistortionCoefficients(-0.064232403853946, -0.087667493884102,
                                        0, 0, 0.792381808294582);
    for (int i = 0; i < 10; i++) {
      filter.push({});
    }
  };
  void Calculate(cv::Mat &image) {
    original = cv::Mat(image);
    cv::Mat roi;
    cv::RotatedRect roiRect;
    Preprocessing(image, 127);
    GetRoi(image, roi, roiRect);
    Preprocessing(roi, 180);
    GetHorn(roi);
    pnpSolve();
  }
  Eigen::Quaternionf &Rotate() { return rotate; }
  Eigen::Vector3f &Position() { return position; }

private:
  void Preprocessing(cv::Mat &image, float thresh) {
    // 颜色过滤
    std::vector<cv::Mat> rgb;
    cv::split(image, rgb);
    // image = rgb.at(2) - rgb.at(0);
    // //转成灰度图片
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    // #二值化
    //     ret, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY);
    cv::threshold(image, image, thresh, 255, cv::THRESH_BINARY);
#ifdef IMG_SHOW
    cv::imshow("threshold", image);
    cv::waitKey(1);
#endif
  };

  void GetRoi(cv::Mat &image, cv::Mat &roi, cv::RotatedRect &roiRect) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<cv::Point2f> outlines;
    for (auto i : contours) {
      if (cv::contourArea(i) < 1000)
        continue;
      for (auto j : i)
        outlines.push_back(j);
    }
    if (outlines.size() != 0)
      roiRect = cv::minAreaRect(outlines);
    cv::Point2f box[4];
    roiRect.points(box);
    cv::Point boxd[] = {box[0], box[1], box[2], box[3]};
    const cv::Point *ptr[] = {boxd};
    const int n[] = {4};
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::fillPoly(mask, ptr, n, 1, cv::Scalar(255));
    cv::copyTo(original, roi, mask);
#ifdef IMG_SHOW
    cv::imshow("roi", roi);
    cv::waitKey(1);
#endif
  }

  void GetHorn(cv::Mat roi) {
    std::vector<cv::Point2f> coners;
    cv::goodFeaturesToTrack(roi, coners, 200, 0.5, 20, cv::Mat(), 10);
    if (coners.size() == 0)
      return;

    cv::TermCriteria criteria = cv::TermCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.000001);
    cv::cornerSubPix(roi, coners, cv::Size(8, 8), cv::Size(3, 3), criteria);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<cv::Point> horns;
    for (auto cor : contours) {
      if (cv::contourArea(cor) < 100)
        continue;

      cv::approxPolyDP(cor, cor, cv::arcLength(cor, true) * 0.01, true);

      for (int k = (int)cor.size(), dk = 2 * k, i = k; i < dk; i++) {
        if ((cor[(i - 1) % k] - cor[i % k])
                .cross(cor[(i + 1) % k] - cor[i % k]) > 0)
          continue;
        float min = 100000;
        auto temp = cor[i % k];
        cv::Point point2push;
        for (auto center : coners) {
          auto k = pow(center.x - temp.x, 2) + pow(center.y - temp.y, 2);
          if (k >= min)
            continue;
          point2push = center;
          min = k;
        }
        horns.push_back(point2push);
      }
    }

    if (horns.size() != 4)
      return;

    auto rotatedRect = cv::minAreaRect(horns);
    cv::Point center = rotatedRect.center;
    cv::circle(original, center, 10, cv::Scalar(255, 197, 107));
    for (auto point : horns) {
      cv::circle(original, point, 10, cv::Scalar(255, 197, 107));
      auto p = point - center;
      if (p.x < 0 && p.y < 0) {
        camera_points_[0] = point;
        world_points_[0] = worldpoints[0];
      } else if (p.x >= 0 && p.y < 0) {
        camera_points_[1] = point;
        world_points_[1] = worldpoints[1];
      } else if (p.x >= 0 && p.y >= 0) {
        camera_points_[2] = point;
        world_points_[2] = worldpoints[2];
      } else if (p.x < 0 && p.y >= 0) {
        camera_points_[3] = point;
        world_points_[3] = worldpoints[3];
      }
    }

    // for (auto horn : camera_points_) {
    //   cout << horn << ",";
    // }
    // for (auto horn : world_points_) {
    //   cout << horn << ",";
    // }
    // cout << endl;
#ifdef IMG_SHOW
    cv::imshow("horns", original);
    cv::waitKey(1);
#endif
  }
  void pnpSolve() {
    pnpsolver.Points2D = camera_points_;
    pnpsolver.Points3D = world_points_;
    if (pnpsolver.Solve(PNPSolver::CV_IPPESQ))
      return;

    // cout << "horn" << endl;
    Eigen::Matrix3f e_rote;
    cv::cv2eigen(pnpsolver.RoteM, e_rote);
    rotate = Eigen::Quaternionf(e_rote);
    auto position_ = pnpsolver.Position_OwInC;
    position = Eigen::Vector3f(position_.x, position_.y, position_.z);
  }
  cv::Mat original;
  cv::Point3f worldpoints[4]{
      cv::Point3f(-125, 125, 0), cv::Point3f(125, 125, 0),
      cv::Point3f(125, -125, 0), cv::Point3f(-125, -125, 0)};
  std::vector<cv::Point2f> camera_points_;
  std::vector<cv::Point3f> world_points_;
  Eigen::Quaternionf rotate;
  Eigen::Vector3f position;
  std::queue<Eigen::Quaternionf> filter;
  PNPSolver pnpsolver;
};

} // namespace EngineerVisual