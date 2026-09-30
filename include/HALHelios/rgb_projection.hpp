#pragma once

#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace hal
{
// OpenCV orientation files map Helios millimetres into the Triton optical frame.
class RgbProjection
{
public:
    cv::Mat camera, distortion, rotation, translation, rotationMatrix;
    std::string distortionModel = "plumb_bob";
    int width = 0, height = 0;

    void load(const std::string &orientation, const std::string &intrinsics)
    {
        cv::FileStorage extrinsics(orientation, cv::FileStorage::READ);
        cv::FileStorage calibration(intrinsics, cv::FileStorage::READ);
        if (!extrinsics.isOpened() || !calibration.isOpened())
            throw std::runtime_error("Cannot open RGB orientation/intrinsics files");
        extrinsics["cameraMatrix"] >> camera;
        extrinsics["distCoeffs"] >> distortion;
        extrinsics["rotationVector"] >> rotation;
        extrinsics["translationVector"] >> translation;
        calibration["image_width"] >> width;
        calibration["image_height"] >> height;
        calibration["distortion_model"] >> distortionModel;
        if (distortionModel.empty())
            distortionModel = "plumb_bob";
        cv::Mat intrinsicCamera, intrinsicDistortion;
        calibration["cameraMatrix"] >> intrinsicCamera;
        calibration["distCoeffs"] >> intrinsicDistortion;
        auto valid = [](const cv::Mat &m) {
            return !m.empty() && m.type() == CV_64FC1 && cv::checkRange(m);
        };
        if (!valid(camera) || camera.rows != 3 || camera.cols != 3 ||
            !valid(distortion) || (distortion.rows != 1 && distortion.cols != 1) ||
            (distortion.total() != 4 && distortion.total() != 5 &&
             distortion.total() != 8 && distortion.total() != 12 && distortion.total() != 14) ||
            !valid(rotation) || rotation.total() != 3 ||
            !valid(translation) || translation.total() != 3 ||
            width <= 0 || height <= 0 || camera.at<double>(0, 0) <= 0 ||
            camera.at<double>(1, 1) <= 0)
            throw std::runtime_error("Invalid RGB calibration matrices or image dimensions");
        if (!valid(intrinsicCamera) || intrinsicCamera.size() != camera.size() ||
            !valid(intrinsicDistortion) || intrinsicDistortion.total() != distortion.total() ||
            cv::norm(camera, intrinsicCamera, cv::NORM_INF) > 1e-6 ||
            cv::norm(distortion.reshape(1, 1), intrinsicDistortion.reshape(1, 1), cv::NORM_INF) > 1e-6)
            throw std::runtime_error("Orientation and Triton intrinsics do not match");
        rotation = rotation.reshape(1, 3);
        translation = translation.reshape(1, 3);
        cv::Rodrigues(rotation, rotationMatrix);
    }

    std::vector<uint32_t> colorize(const std::vector<cv::Point3f> &meters,
                                   const cv::Mat &rgb, size_t &matched) const
    {
        matched = 0;
        std::vector<uint32_t> colors(meters.size(), 0);
        if (rgb.empty())
            return colors;
        if (rgb.type() != CV_8UC3 || rgb.cols <= 0 || rgb.rows <= 0)
            throw std::runtime_error("Triton image format or dimensions are invalid");
        const double scaleX = static_cast<double>(rgb.cols) / width;
        const double scaleY = static_cast<double>(rgb.rows) / height;
        std::vector<cv::Point3f> points;
        std::vector<size_t> indices;
        for (size_t i = 0; i < meters.size(); ++i)
        {
            const auto &p = meters[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z <= 0)
                continue;
            cv::Point3f mm = p * 1000.0f;
            const double z = rotationMatrix.at<double>(2, 0) * mm.x +
                             rotationMatrix.at<double>(2, 1) * mm.y +
                             rotationMatrix.at<double>(2, 2) * mm.z + translation.at<double>(2);
            if (z <= 0)
                continue;
            points.push_back(mm);
            indices.push_back(i);
        }
        if (points.empty())
            return colors;
        std::vector<cv::Point2f> pixels;
        cv::projectPoints(points, rotation, translation, camera, distortion, pixels);
        for (size_t i = 0; i < pixels.size(); ++i)
        {
            const auto &uv = pixels[i];
            if (!std::isfinite(uv.x) || !std::isfinite(uv.y) ||
                uv.x < 0 || uv.y < 0 || uv.x >= width || uv.y >= height)
                continue;
            const int x = std::min(static_cast<int>(uv.x * scaleX), rgb.cols - 1);
            const int y = std::min(static_cast<int>(uv.y * scaleY), rgb.rows - 1);
            const auto &c = rgb.at<cv::Vec3b>(y, x);
            colors[indices[i]] = (uint32_t(c[0]) << 16) | (uint32_t(c[1]) << 8) | c[2];
            ++matched;
        }
        return colors;
    }
};
}
