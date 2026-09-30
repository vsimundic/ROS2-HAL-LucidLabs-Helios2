#include "HALHelios/rgb_projection.hpp"
#include <cstdio>
#include <iostream>
#include <limits>

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}
int main()
{
    hal::RgbProjection projection;
    projection.width = projection.height = 3;
    projection.camera = (cv::Mat_<double>(3, 3) << 1, 0, 1, 0, 1, 1, 0, 0, 1);
    projection.distortion = cv::Mat::zeros(14, 1, CV_64F);
    projection.rotation = cv::Mat::zeros(3, 1, CV_64F);
    projection.translation = (cv::Mat_<double>(3, 1) << 1000, 0, 0);
    projection.rotationMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat rgb(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    rgb.at<cv::Vec3b>(1, 2) = cv::Vec3b(0x12, 0x34, 0x56);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    size_t matched;
    auto colors = projection.colorize({{0, 0, 1}, {100, 0, 1}, {nan, 0, 1}, {0, 0, -1}}, rgb, matched);
    require(matched == 1 && colors[0] == 0x123456, "millimetre translation or RGB channel packing failed");
    require(colors[1] == 0 && colors[2] == 0 && colors[3] == 0, "invalid/outside points must be uncolored");
    cv::Mat scaledRgb(6, 6, CV_8UC3, cv::Scalar(0, 0, 0));
    scaledRgb.at<cv::Vec3b>(2, 4) = cv::Vec3b(0x65, 0x43, 0x21);
    require(projection.colorize({{0, 0, 1}}, scaledRgb, matched)[0] == 0x654321 && matched == 1,
            "scaled RGB projection failed");
    projection.translation.at<double>(2) = -2000;
    require(projection.colorize({{0, 0, 1}}, rgb, matched)[0] == 0 && matched == 0,
            "point behind Triton must be rejected");
    require(projection.colorize({{0, 0, 1}}, cv::Mat(), matched)[0] == 0,
            "missing RGB must be uncolored");
    require(projection.colorize({}, rgb, matched).empty(), "empty cloud failed");
    bool rejected = false;
    try { projection.colorize({{0, 0, 1}}, cv::Mat(2, 2, CV_8UC1), matched); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "incorrect image format must be rejected");
    // Exercise file loading, 14-coefficient intrinsics, and mismatched calibration rejection.
    const std::string file = "/tmp/helios_rgb_projection_test.yml";
    {
        cv::FileStorage fs(file, cv::FileStorage::WRITE);
        fs << "image_width" << 3 << "image_height" << 3;
        fs << "cameraMatrix" << projection.camera << "distCoeffs" << projection.distortion;
        fs << "rotationVector" << projection.rotation << "translationVector" << projection.translation;
    }
    projection.load(file, file);
    require(projection.distortion.total() == 14, "distortion coefficients were lost");
    const std::string mismatch = "/tmp/helios_rgb_projection_mismatch.yml";
    {
        cv::FileStorage fs(mismatch, cv::FileStorage::WRITE);
        cv::Mat other = projection.camera.clone();
        other.at<double>(0, 0) += 10;
        fs << "image_width" << 3 << "image_height" << 3;
        fs << "cameraMatrix" << other << "distCoeffs" << projection.distortion;
    }
    rejected = false;
    try { projection.load(file, mismatch); }
    catch (const std::runtime_error &) { rejected = true; }
    require(rejected, "mismatched calibration must be rejected");
    std::remove(mismatch.c_str());
    std::remove(file.c_str());
    std::cout << "RGB projection tests passed\n";
}
