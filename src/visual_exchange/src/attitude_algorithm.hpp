#pragma once
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

#include "pnp_solver.hpp"

namespace EngineerVisual {

class AttitudeAlgorithm {
public:
  AttitudeAlgorithm() : pnpsolver() {

    //初始化相机参数

    pnpsolver.SetCameraMatrix(2783.51380399845, 2795.03600410101,
                              1932.22472201653, 1153.49477364402);
    //设置畸变参数
    pnpsolver.SetDistortionCoefficients(0.0407902477934583, -0.547170508860230,
                                        0, 0, 1.72889744087393);
  };
  void Calculate(cv::Mat &image) {
    std::vector<cv::Mat> rois;
    Preprocessing(image);
    Processing(image);
  }
  Eigen::Quaternionf &Rotate() { return rotate; }
  Eigen::Vector3f &Position() { return position; }

private:
  void Preprocessing(cv::Mat &image) {
    original = image.clone();
    // 颜色过滤
    std::vector<cv::Mat> rgb;
    cv::split(image, rgb);
    image = rgb.at(2) - rgb.at(0);
    // //转成灰度图片
    // cv::img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY);
    // #二值化
    //     ret, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY);
    cv::threshold(image, image, 70, 255, cv::THRESH_BINARY);
  };

  void GetLineIntersection(std::vector<Eigen::Vector3f> &l1,
                           std::vector<Eigen::Vector3f> &l2,
                           Eigen::Vector2f &intersection) {
    auto a1 = l1[1].y() - l1[0].y();
    auto b1 = l1[0].x() - l1[1].x();
    auto c1 = l1[1].x() * l1[0].y() - l1[1].y() * l1[0].x();

    auto a2 = l2[1].y() - l2[0].y();
    auto b2 = l2[0].x() - l2[1].x();
    auto c2 = l2[1].x() * l2[0].y() - l2[1].y() * l2[0].x();

    intersection = Eigen::Vector2f((c2 * b1 - c1 * b2) / (a1 * b2 - a2 * b1),
                                   (c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1));
  }

  void Processing(cv::Mat &image) {
    // cv::imshow("1", image);
    // cv::waitKey(1);
    std::vector<std::vector<cv::Point>> lightContours;
    cv::findContours(image, lightContours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    // image = cv::Mat::zeros(image.size(), CV_8UC1); //绘制
    // if (lightContours.size() > 0)
    //   drawContours(image, lightContours, 0, cv::Scalar(255), 1, 8);
    // 获取可能的区域
    cv::Size inflationSize(20, 20);
    std::vector<cv::Mat> rois_temp;
    std::vector<std::vector<cv::Point>> poly;

    for (auto cors : lightContours) {
      if (cv::contourArea(cors) > 5000) {
        // cv::convexHull(cors, hull, false, true);
        cv::approxPolyDP(cors, cors, cv::arcLength(cors, true) * 0.02, true);
        poly.push_back(cors);
      }
    }

    // for (auto k : poly) {
    //   for (size_t i = 0, j = k.size(); i < j; i++) {
    //     cv::line(original, k[i], k[(i + 1) % j], cv::Scalar(100, 255, 100),
    //     10); cv::putText(original, std::to_string(i), k[i], 1, 3,Eigen

    // 取线
    std::vector<std::vector<Eigen::Vector3f>> lines;
    for (auto points : poly) {
      auto k = points.size();
      if (k < 6)
        continue;
      for (size_t i = k; i < 2 * k; i++) {
        auto p = points[(i - 1) % k] - points[i % k];
        Eigen::Vector3f l1(p.x, p.y, 0);
        p = points[(i + 1) % k] - points[i % k];
        Eigen::Vector3f l2(p.x, p.y, 0);
        Eigen::Vector3f a = l1.cross(l2);
        if (a.z() < 0) {
          l1 = (l1 + l2);
          l2 = Eigen::Vector3f(points[i % k].x, points[i % k].y, 0);
          lines.push_back({l2, l2 + l1});
          cv::line(original, cv::Point(l2.x(), l2.y()),
                   cv::Point(l2.x() + l1.x(), l2.y() + l1.y()),
                   cv::Scalar(255, 200, 255), 4);
          cv::circle(original, points[i % k], 20, cv::Scalar(255, 200, 255), 4);
        }
      }
    }

    int k = lines.size();
    // if (k < 2)
    //   return;
    Eigen::Vector2f inters(0, 0);
    Eigen::Vector2f intersec(0, 0);
    float times = 0;
    for (int i = 0; i < k; i++) {
      Eigen::Vector3f t1 = lines[i][1] - lines[i][0];
      Eigen::Vector3f t2 = lines[(i + 1) % k][1] - lines[(i + 1) % k][0];
      t1 = t1.normalized();
      t2 = t2.normalized();
      if (abs(t1.dot(t2)) > 0.8f)
        continue;

      GetLineIntersection(lines[i], lines[(i + 1) % k], intersec);
      inters += intersec;
      times++;
    }
    if (times != 0)
      inters /= times;

    cv::circle(original, cv::Point2f(inters.x(), inters.y()), 20,
               cv::Scalar(150, 100, 255), -1);

    std::vector<cv::Point3f> input_world; //{cv::Point3f(0, 0, 0)};
    std::vector<cv::Point2f>
        input_camera; //{cv::Point2f(inters.x(), inters.y())};
    if (lines.size() <= 3) {
      input_world.push_back(cv::Point3f(0, 0, 0));
      input_camera.push_back(cv::Point2f(inters.x(), inters.y()));
    }
    int last = -1;
    for (auto p : lines) {
      auto vec = p[1] - p[0];
      if (vec.x() < 0 && vec.y() < 0) {
        input_world.push_back(worldpoints[3]);
        input_camera.push_back(cv::Point(p[0].x(), p[0].y()));
        cv::putText(original, "2", cv::Point2f(p[0].x(), p[0].y()), 1, 5,
                    cv::Scalar(150, 100, 255));
        last = 3;
      } else if (vec.x() >= 0 && vec.y() < 0) {
        input_world.push_back(worldpoints[1]);
        input_camera.push_back(cv::Point(p[0].x(), p[0].y()));
        cv::putText(original, "1", cv::Point2f(p[0].x(), p[0].y()), 1, 5,
                    cv::Scalar(150, 100, 255));
        last = 1;
      } else if (vec.x() < 0 && vec.y() >= 0) {
        input_world.push_back(worldpoints[2]);
        input_camera.push_back(cv::Point(p[0].x(), p[0].y()));
        cv::putText(original, "3", cv::Point2f(p[0].x(), p[0].y()), 1, 5,
                    cv::Scalar(150, 100, 255));
        last = 2;
      } else {
        input_world.push_back(worldpoints[0]);
        input_camera.push_back(cv::Point(p[0].x(), p[0].y()));
        cv::putText(original, "0", cv::Point2f(p[0].x(), p[0].y()), 1, 5,
                    cv::Scalar(150, 100, 255));
        last = 0;
      }
    }
    if (input_camera.size() == 3) {
      input_camera.push_back(input_camera.front() * 2 - input_camera.back());
      input_world.push_back(worldpoints[3 - last]);
    }

    pnpsolver.Points2D = input_camera;
    pnpsolver.Points3D = input_world;

    if (pnpsolver.Solve(PNPSolver::METHOD::CV_P3P) == 0) {

      Eigen::Matrix3f r_eigen;
      cv::cv2eigen(pnpsolver.RoteM, r_eigen);
      rotate = Eigen::Quaternionf(r_eigen);
      position = Eigen::Vector3f(pnpsolver.Position_OcInW.x,
                                 pnpsolver.Position_OcInW.y,
                                 pnpsolver.Position_OcInW.z);
      std::cout << "test2:CV_EPNP方法: 相机位姿→"
                << "Oc坐标=" << pnpsolver.Position_OcInW
                << "    相机旋转=" << pnpsolver.Theta_W2C << endl;
    }
    cv::imshow("Point of Contours",
               original); //向量contours内保存的所有轮廓点集
    cv::waitKey(10);
  }
  cv::Mat original;
  cv::Point3f worldpoints[4]{
      cv::Point3f(0, 150, 150), cv::Point3f(0, 150, -150),
      cv::Point3f(0, -150, 150), cv::Point3f(0, -150, -150)};
  Eigen::Quaternionf rotate;
  Eigen::Vector3f position;
  PNPSolver pnpsolver;
};
} // namespace EngineerVisual