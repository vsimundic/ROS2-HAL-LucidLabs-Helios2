#ifndef HAL_LUCIDLABS_HELIOS2
#define HAL_LUCIDLABS_HELIOS2

#include <string>
#include <cstdlib>
#include <vector>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "HALHelios/rgb_projection.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <opencv2/core.hpp>

#include "ArenaApi.h"

namespace hal
{

  class LucidlabsHelios2 : public rclcpp::Node, public Arena::IImageCallback
  {
  private:
    Arena::ISystem *pSystem = nullptr;
    std::vector<Arena::DeviceInfo> deviceInfos;
    Arena::IDevice *pDevice = nullptr;
    GenApi::INodeMap *pNodeMap = nullptr;
    GenApi::INodeMap *pStreamNodeMap = nullptr;
    Arena::IDevice *rgbDevice_ = nullptr;
    bool rgbEnabled_ = false;
    bool rgbStreaming_ = false;
    double rgbMaxAgeMs_ = 200.0;
    int rgbOutputWidth_ = 0;
    int rgbOutputHeight_ = 0;
    std::atomic<bool> rgbRunning_{false};
    std::thread rgbThread_;
    std::mutex rgbMutex_;
    cv::Mat latestRgb_;
    std::chrono::steady_clock::time_point latestRgbTime_;
    RgbProjection rgbProjection_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbCameraInfoPublisher_;
    sensor_msgs::msg::CameraInfo rgbCameraInfoMsg_;
    std::string rgbFrameId_;
    void initRgb();
    void captureRgb();
    void stopRgb() noexcept;
    void addRgb(sensor_msgs::msg::PointCloud2 &cloud);
    float offX, offY, offZ;
    float scaleX, scaleY, scaleZ;
    float sX, sY, sZ, oX, oY, oZ;
    int confidenceFilterThreshold, flyingFilterThreshold, accum, exposure, mode, hdrMode;
    bool bConfidenceFilter, bSpatialFilter, bFlyingFilter;
    bool bStructuredCloud, bPublishIntensity, bPublishDepth;
    bool hasY;

    std::string frameID;
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    Arena::IImageCallback *pCallbackHandler;

    int imgWidth_ = 640;
    int imgHeight_ = 480;
    const int n_distortion_coefficients = 5;

    std::string intensityImgTopic, depthImgTopic, pointsTopic, camInfoTopic;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensityImgPublisher_, depthImgPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoPublisher_;
    sensor_msgs::msg::CameraInfo camInfoMsg_;
    sensor_msgs::msg::Image intensityMsg_;
    sensor_msgs::msg::Image depthMsg_;
    sensor_msgs::msg::PointCloud2 cloudMsg_;
    bool cloudInitialized_ = false;
    bool cloudHasIntensity_ = false;
    bool imagesInitialized_ = false;
    bool camInfoReady_ = false;
    cv::Mat cameraMatrix, distortionCoeffs;
    cv::Mat rvec;
    cv::Mat tvec;



    // For ABCY16: 4 uint16 per pixel
    int STRIDE = 4;

    enum class cam_model
    {
      HELIOS_2,
      HELIOS_2_PLUS,
      HELIOS_2_RAY
    };
    cam_model m;

    Arena::IDevice *findDevice();
    Arena::IImage *tryGetImage();
    void runtime();
    void OnImage(Arena::IImage *pImage) override;

    template <typename PointT>
    void getPointCloudAndImages(Arena::IImage *pImage);
    // void publishIntensityImage(Arena::IImage* pImage, const rclcpp::Time& stamp);
    void initCameraInfo();
    void publishCameraInfo(const rclcpp::Time &stamp);
    bool nodeReadable(GenApi::INodeMap *nm, const char *name);
    void readCalibrationFromHelios(Arena::IDevice *pDeviceHLT);
    cv::Mat projectAll3DPointsOnImage(const cv::Mat& xyz);
  public:
    LucidlabsHelios2(const rclcpp::NodeOptions &options);
    ~LucidlabsHelios2();
  };

} // namespace hal

#endif // HAL_LUCIDLABS_HELIOS2
