#include "HALHelios/lucidlabs_helios2.hpp"
#include <cstring>
#include <memory>
#include <opencv2/imgproc.hpp>

namespace hal
{
void LucidlabsHelios2::initRgb()
{
    rgbEnabled_ = declare_parameter<bool>("rgb.enable", false);
    const auto serial = declare_parameter<std::string>("rgb.serial", "");
    const auto orientation = declare_parameter<std::string>("rgb.orientation_file", "");
    const auto intrinsics = declare_parameter<std::string>("rgb.intrinsics_file", "");
    rgbFrameId_ = declare_parameter<std::string>("rgb.frame_id", "triton_optical_frame");
    rgbMaxAgeMs_ = declare_parameter<double>("rgb.max_age_ms", 200.0);
    rgbOutputWidth_ = declare_parameter<int>("rgb.output_width", 0);
    rgbOutputHeight_ = declare_parameter<int>("rgb.output_height", 0);
    if (!rgbEnabled_)
        return;
    if (!std::isfinite(rgbMaxAgeMs_) || rgbMaxAgeMs_ <= 0)
        throw std::runtime_error("rgb.max_age_ms must be positive and finite");
    if ((rgbOutputWidth_ == 0) != (rgbOutputHeight_ == 0) ||
        rgbOutputWidth_ < 0 || rgbOutputHeight_ < 0)
        throw std::runtime_error("rgb.output_width and rgb.output_height must both be zero or both be positive");
    rgbProjection_.load(orientation, intrinsics);
    if (cv::norm(rgbProjection_.translation) > 10000.0)
        RCLCPP_WARN(get_logger(), "RGB calibration translation exceeds 10 metres; check orientation.yml (units: mm)");
    std::vector<Arena::DeviceInfo> candidates;
    for (auto info : deviceInfos)
    {
        const std::string model = info.ModelName().c_str();
        if (model.rfind("TRI", 0) == 0 &&
            (serial.empty() || serial == info.SerialNumber().c_str()))
            candidates.push_back(info);
    }
    if (candidates.size() != 1)
        throw std::runtime_error("RGB requires exactly one matching Triton; set rgb.serial when multiple are connected");
    rgbDevice_ = pSystem->CreateDevice(candidates.front());
    auto *nodes = rgbDevice_->GetNodeMap();
    GenApi::CEnumerationPtr format = nodes->GetNode("PixelFormat");
    bool selected = false;
    for (const char *name : {"BayerRG8", "BayerGB8", "BayerGR8", "BayerBG8", "RGB8"})
    {
        auto entry = format->GetEntryByName(name);
        if (GenApi::IsAvailable(entry) && GenApi::IsReadable(entry))
        {
            format->SetIntValue(entry->GetValue());
            selected = true;
            break;
        }
    }
    if (!selected)
        throw std::runtime_error("Selected Triton does not support RGB8 or 8-bit Bayer color");
    // Calibration assumes full-frame pixels, without crop or binning.
    for (const char *name : {"BinningHorizontal", "BinningVertical", "DecimationHorizontal", "DecimationVertical"})
    {
        GenApi::CIntegerPtr node = nodes->GetNode(name);
        if (GenApi::IsReadable(node) && node->GetValue() != 1)
            throw std::runtime_error(std::string("RGB calibration requires ") + name + "=1");
    }
    for (const char *name : {"OffsetX", "OffsetY"})
    {
        GenApi::CIntegerPtr node = nodes->GetNode(name);
        if (GenApi::IsReadable(node) && node->GetValue() != 0)
            throw std::runtime_error(std::string("RGB calibration requires ") + name + "=0");
    }
    if (Arena::GetNodeValue<int64_t>(nodes, "Width") != rgbProjection_.width ||
        Arena::GetNodeValue<int64_t>(nodes, "Height") != rgbProjection_.height)
        throw std::runtime_error("Triton resolution differs from calibrated resolution");
    if (rgbOutputWidth_ == 0)
    {
        rgbOutputWidth_ = rgbProjection_.width;
        rgbOutputHeight_ = rgbProjection_.height;
    }
    Arena::SetNodeValue<GenICam::gcstring>(nodes, "TriggerMode", "Off");
    Arena::SetNodeValue<GenICam::gcstring>(nodes, "AcquisitionMode", "Continuous");
    auto *stream = rgbDevice_->GetTLStreamNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(stream, "StreamBufferHandlingMode", "NewestOnly");
    Arena::SetNodeValue<bool>(stream, "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(stream, "StreamPacketResendEnable", true);
    rgbPublisher_ = create_publisher<sensor_msgs::msg::Image>("/" + frameID + "/rgb/image_raw", 1);
    rgbCameraInfoPublisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/" + frameID + "/rgb/camera_info", 1);

    const double sx = static_cast<double>(rgbOutputWidth_) / rgbProjection_.width;
    const double sy = static_cast<double>(rgbOutputHeight_) / rgbProjection_.height;
    const double fx = rgbProjection_.camera.at<double>(0, 0) * sx;
    const double fy = rgbProjection_.camera.at<double>(1, 1) * sy;
    const double cx = rgbProjection_.camera.at<double>(0, 2) * sx;
    const double cy = rgbProjection_.camera.at<double>(1, 2) * sy;
    rgbCameraInfoMsg_.header.frame_id = rgbFrameId_;
    rgbCameraInfoMsg_.width = static_cast<uint32_t>(rgbOutputWidth_);
    rgbCameraInfoMsg_.height = static_cast<uint32_t>(rgbOutputHeight_);
    rgbCameraInfoMsg_.distortion_model = rgbProjection_.distortionModel;
    const cv::Mat distortion = rgbProjection_.distortion.reshape(1, 1);
    rgbCameraInfoMsg_.d.assign(
        distortion.ptr<double>(), distortion.ptr<double>() + distortion.total());
    rgbCameraInfoMsg_.k = {fx, 0.0, cx,
                           0.0, fy, cy,
                           0.0, 0.0, 1.0};
    rgbCameraInfoMsg_.r = {1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0};
    rgbCameraInfoMsg_.p = {fx, 0.0, cx, 0.0,
                           0.0, fy, cy, 0.0,
                           0.0, 0.0, 1.0, 0.0};
    rgbDevice_->StartStream();
    rgbStreaming_ = true;
    rgbRunning_ = true;
    rgbThread_ = std::thread(&LucidlabsHelios2::captureRgb, this);
    RCLCPP_INFO(get_logger(),
                "RGB enabled for Triton %s; output %dx%d; maximum host frame age %.1f ms",
                candidates.front().SerialNumber().c_str(), rgbOutputWidth_, rgbOutputHeight_, rgbMaxAgeMs_);
}

void LucidlabsHelios2::captureRgb()
{
    while (rgbRunning_)
    {
        Arena::IImage *raw = nullptr;
        Arena::IImage *converted = nullptr;
        try
        {
            raw = rgbDevice_->GetImage(250);
            const auto receivedAt = std::chrono::steady_clock::now();
            const auto stamp = get_clock()->now();
            if (!raw->IsIncomplete())
            {
                converted = Arena::ImageFactory::Convert(raw, RGB8);
                cv::Mat convertedRgb(static_cast<int>(converted->GetHeight()),
                                     static_cast<int>(converted->GetWidth()), CV_8UC3,
                                     const_cast<uint8_t *>(converted->GetData()));
                // Keep the calibrated native-resolution image for point-cloud
                // colorization. Scaling is applied only to the published image.
                cv::Mat fullRgb = convertedRgb.clone();
                cv::Mat publishedRgb;
                if (convertedRgb.cols == rgbOutputWidth_ && convertedRgb.rows == rgbOutputHeight_)
                    publishedRgb = fullRgb;
                else
                {
                    cv::resize(fullRgb, publishedRgb, cv::Size(rgbOutputWidth_, rgbOutputHeight_),
                               0.0, 0.0, cv::INTER_AREA);
                }
                {
                    std::lock_guard<std::mutex> lock(rgbMutex_);
                    latestRgb_ = std::move(fullRgb);
                    latestRgbTime_ = receivedAt;
                }
                sensor_msgs::msg::Image msg;
                msg.header.stamp = stamp;
                msg.header.frame_id = rgbFrameId_;
                msg.width = publishedRgb.cols;
                msg.height = publishedRgb.rows;
                msg.encoding = "rgb8";
                msg.step = msg.width * 3;
                msg.data.assign(publishedRgb.data,
                                publishedRgb.data + publishedRgb.total() * publishedRgb.elemSize());
                rgbPublisher_->publish(msg);
                rgbCameraInfoMsg_.header.stamp = stamp;
                rgbCameraInfoPublisher_->publish(rgbCameraInfoMsg_);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Triton capture: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Triton capture failed");
        }
        // Release SDK-owned buffers on success, incomplete frames, and conversion failures.
        if (converted)
        {
            try { Arena::ImageFactory::Destroy(converted); } catch (...) {}
        }
        if (raw)
        {
            try { rgbDevice_->RequeueBuffer(raw); } catch (...) {}
        }
    }
}

void LucidlabsHelios2::stopRgb() noexcept
{
    rgbRunning_ = false;
    if (rgbThread_.joinable())
        rgbThread_.join();
    if (rgbDevice_)
    {
        if (rgbStreaming_)
        {
            try { rgbDevice_->StopStream(); } catch (...) {}
        }
        try { pSystem->DestroyDevice(rgbDevice_); } catch (...) {}
        rgbDevice_ = nullptr;
        rgbStreaming_ = false;
    }
}

void LucidlabsHelios2::addRgb(sensor_msgs::msg::PointCloud2 &cloud)
{
    cv::Mat rgb;
    {
        std::lock_guard<std::mutex> lock(rgbMutex_);
        const double age = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - latestRgbTime_).count();
        if (!latestRgb_.empty() && age <= rgbMaxAgeMs_)
            rgb = latestRgb_;
    }
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z)
        points.emplace_back(*x, *y, *z);
    size_t matched = 0;
    const auto colors = rgbProjection_.colorize(points, rgb, matched);
    if (rgb.empty())
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No fresh Triton frame; RGB values are zero");
    else if (!points.empty() && matched == 0)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No points project into Triton image; check calibration and camera overlap");

    sensor_msgs::msg::PointField field;
    field.name = "rgb";
    field.offset = cloud.point_step;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
    const auto oldStep = cloud.point_step;
    const auto oldRowStep = cloud.row_step;
    cloud.point_step += sizeof(uint32_t);
    cloud.row_step = cloud.width * cloud.point_step;
    std::vector<uint8_t> colored(static_cast<size_t>(cloud.row_step) * cloud.height);
    for (size_t i = 0; i < colors.size(); ++i)
    {
        auto *dst = colored.data() + i * cloud.point_step;
        const auto *src = cloud.data.data() + (i / cloud.width) * oldRowStep + (i % cloud.width) * oldStep;
        std::memcpy(dst, src, oldStep);
        std::memcpy(dst + oldStep, &colors[i], sizeof(uint32_t));
    }
    cloud.data = std::move(colored);
}
}
