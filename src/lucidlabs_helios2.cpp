#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <functional>
#include <cstdlib>
#include <memory>
#include <cassert>
#include <vector>
#include <type_traits>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/qos.hpp"
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "ArenaApi.h"

#include "HALHelios/lucidlabs_helios2.hpp"

#define SYSTEM_TIMEOUT 2000
#define IMAGE_TIMEOUT 2000

using std::placeholders::_1;

namespace hal
{

    LucidlabsHelios2::LucidlabsHelios2(const rclcpp::NodeOptions &options) : Node("helios2_hal", options)
    {
        // output_topic = declare_parameter<std::string>("output_topic", "/point_cloud");
        frameID = declare_parameter<std::string>("frame_id", "camera");
        exposure = declare_parameter<int>("exposure_level", 0);
        hdrMode = declare_parameter<int>("hdr_mode", 0);
        mode = declare_parameter<int>("mode", 0);
        accum = declare_parameter<int>("accumulate_frames", 1);
        bConfidenceFilter = declare_parameter<bool>("confidence_filter.enable", true);
        confidenceFilterThreshold = declare_parameter<int>("confidence_filter.threshold", 0);
        bSpatialFilter = declare_parameter<bool>("spatial_filter.enable", true);
        bFlyingFilter = declare_parameter<bool>("flying_filter.enable", true);
        flyingFilterThreshold = declare_parameter<int>("flying_filter.threshold", 0);
        bStructuredCloud = declare_parameter<bool>("structured_cloud", true);
        bPublishIntensity = declare_parameter<bool>("publish_intensity", true);
        bPublishDepth = declare_parameter<bool>("publish_depth", true);

        pDevice = findDevice();

        try
        {
            pNodeMap = pDevice->GetNodeMap();
            pStreamNodeMap = pDevice->GetTLStreamNodeMap();

            /************************/

            GenApi::CEnumerationPtr pModeSel = pNodeMap->GetNode("Scan3dModeSelector");
            GenApi::CEnumEntryPtr pModeProcessed = pModeSel->GetEntryByName("Processed");
            pModeSel->SetIntValue(pModeProcessed->GetValue());

            /************************/

            Arena::SetNodeValue<bool>(pDevice->GetNodeMap(), "ChunkModeActive", false);

            /************************/

            GenApi::CEnumerationPtr pStreamBufferHandlingMode = pStreamNodeMap->GetNode("StreamBufferHandlingMode");
            GenApi::CEnumEntryPtr pNewestOnly = pStreamBufferHandlingMode->GetEntryByName("NewestOnly");
            pStreamBufferHandlingMode->SetIntValue(pNewestOnly->GetValue());

            /************************/

            GenApi::CEnumerationPtr pPixelMode = pNodeMap->GetNode("PixelFormat");
            GenApi::CEnumEntryPtr pFormat = pPixelMode->GetEntryByName(
                bPublishIntensity ? "Coord3D_ABCY16" : "Coord3D_ABC16");
            pPixelMode->SetIntValue(pFormat->GetValue());

            STRIDE = bPublishIntensity ? 4 : 3;
            hasY = bPublishIntensity;

            /************************/

            Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
            Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

            /************************/

            /*
            Enumeration: 'ExposureTimeSelector'
                EnumEntry: 'Exp62_5Us'
                EnumEntry: 'Exp250Us'
                EnumEntry: 'Exp1000Us'
                EnumEntry: 'Exp350Us',
                EnumEntry: 'Exp88Us',
                EnumEntry: 'Exp13Us',
            */
            static std::vector<std::string> exposures = {
                "Exp62_5Us",
                "Exp250Us",
                "Exp1000Us",
                "Exp350Us",
                "Exp88Us",
                "Exp13Us"};

            GenApi::CEnumerationPtr pExposureTime = pNodeMap->GetNode("ExposureTimeSelector");
            GenApi::CEnumEntryPtr pExp = pExposureTime->GetEntryByName(exposures[exposure].c_str());
            pExposureTime->SetIntValue(pExp->GetValue());
            RCLCPP_INFO(get_logger(), "Exposure setting: %s", exposures[exposure].c_str());

            /************************/

            /*
            Enumeration: 'Scan3dOperatingMode'
                EnumEntry: 'HighSpeedDistance2500mmSingleFreq'
                EnumEntry: 'HighSpeedDistance1250mmSingleFreq'
                EnumEntry: 'HighSpeedDistance625mmSingleFreq'
                EnumEntry: 'Freq90MHz' (Not available)
                EnumEntry: 'Distance8300mmMultiFreq'
                EnumEntry: 'Distance6000mmSingleFreq'
                EnumEntry: 'Distance5000mmMultiFreq'
                EnumEntry: 'Distance4000mmSingleFreq'
                EnumEntry: 'Distance3000mmSingleFreq'
                EnumEntry: 'Distance1250mmSingleFreq'
            */
            static std::vector<std::string> modes = {
                "HighSpeedDistance2500mmSingleFreq",
                "HighSpeedDistance1250mmSingleFreq",
                "HighSpeedDistance625mmSingleFreq",
                "Distance8300mmMultiFreq",
                "Distance6000mmSingleFreq",
                "Distance5000mmMultiFreq",
                "Distance4000mmSingleFreq",
                "Distance3000mmSingleFreq",
                "Distance1250mmSingleFreq"};

            if (m == cam_model::HELIOS_2 && mode <= 2)
            {
                RCLCPP_ERROR(get_logger(), "Helios 2 does not support Scan3dOperatingMode=%s", modes[mode].c_str());
                std::abort();
            }

            GenApi::CEnumerationPtr pOperatingMode = pNodeMap->GetNode("Scan3dOperatingMode");
            GenApi::CEnumEntryPtr pMode = pOperatingMode->GetEntryByName(modes[mode].c_str());
            pOperatingMode->SetIntValue(pMode->GetValue());
            RCLCPP_INFO(get_logger(), "Mode setting: %s", modes[mode].c_str());

            /************************/

            GenApi::CBooleanPtr pConfidenceFilter = pNodeMap->GetNode("Scan3dConfidenceThresholdEnable");
            pConfidenceFilter->SetValue(bConfidenceFilter);
            GenApi::CIntegerPtr pConfThresh = pNodeMap->GetNode("Scan3dConfidenceThresholdMin");
            pConfThresh->SetValue(confidenceFilterThreshold);

            /************************/

            GenApi::CBooleanPtr pSpatialFilter = pNodeMap->GetNode("Scan3dSpatialFilterEnable");
            pSpatialFilter->SetValue(bSpatialFilter);

            /************************/

            GenApi::CBooleanPtr pFlyingFilter = pNodeMap->GetNode("Scan3dFlyingPixelsRemovalEnable");
            pFlyingFilter->SetValue(bFlyingFilter);
            // Maybe not available on Helios2
            GenApi::CIntegerPtr pFlyingThresh = pNodeMap->GetNode("Scan3dFlyingPixelsDistanceThreshold");
            pFlyingThresh->SetValue(flyingFilterThreshold);

            /************************/

            static std::vector<std::string> hdr_modes = {
                "LowNoiseHDRX8",
                "LowNoiseHDRX4",
                "StandardHDR",
                "Off"};

            if ((m == cam_model::HELIOS_2 || m == cam_model::HELIOS_2_RAY) && hdrMode != 3)
            {
                RCLCPP_ERROR(get_logger(), "Helios 2 does not support Scan3dHDRMode");
                std::abort();
            }
            else if (m == cam_model::HELIOS_2_PLUS)
            {
                GenApi::CEnumerationPtr pScan3dHDRMode = pNodeMap->GetNode("Scan3dHDRMode");
                GenApi::CEnumEntryPtr pHDRMode = pScan3dHDRMode->GetEntryByName(hdr_modes[hdrMode].c_str());
                pScan3dHDRMode->SetIntValue(pHDRMode->GetValue());
                RCLCPP_INFO(get_logger(), "HDR Mode setting: %s", hdr_modes[hdrMode].c_str());
            }

            if (hdrMode == 3)
            {
                Arena::SetNodeValue<int64_t>(pDevice->GetNodeMap(), "Scan3dImageAccumulation", accum);
            }

            /************************/

            GenApi::CEnumerationPtr pScan3DCoordinateSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
            GenApi::CEnumEntryPtr pCoordinateA = pScan3DCoordinateSelector->GetEntryByName("CoordinateA");
            GenApi::CEnumEntryPtr pCoordinateB = pScan3DCoordinateSelector->GetEntryByName("CoordinateB");
            GenApi::CEnumEntryPtr pCoordinateC = pScan3DCoordinateSelector->GetEntryByName("CoordinateC");
            GenApi::CFloatPtr pScan3DCoordinateOffset = pNodeMap->GetNode("Scan3dCoordinateOffset");
            GenApi::CFloatPtr pScan3DCoordinateScale = pNodeMap->GetNode("Scan3dCoordinateScale");

            pScan3DCoordinateSelector->SetIntValue(pCoordinateA->GetValue());
            offX = pScan3DCoordinateOffset->GetValue();
            scaleX = pScan3DCoordinateScale->GetValue();
            pScan3DCoordinateSelector->SetIntValue(pCoordinateB->GetValue());
            offY = pScan3DCoordinateOffset->GetValue();
            scaleY = pScan3DCoordinateScale->GetValue();
            pScan3DCoordinateSelector->SetIntValue(pCoordinateC->GetValue());
            offZ = pScan3DCoordinateOffset->GetValue();
            scaleZ = pScan3DCoordinateScale->GetValue();

            sX = scaleX * 0.001f;
            sY = scaleY * 0.001f;
            sZ = scaleZ * 0.001f;
            oX = offX * 0.001f;
            oY = offY * 0.001f;
            oZ = offZ * 0.001f;

            RCLCPP_INFO(get_logger(), "Offset = %f | %f | %f", offX, offY, offZ);
            RCLCPP_INFO(get_logger(), "Scale  = %f | %f | %f", scaleX, scaleY, scaleZ);

            /************************/

            pCallbackHandler = this;
            pDevice->RegisterImageCallback(pCallbackHandler);
        }
        catch (GenICam::GenericException &ge)
        {
            RCLCPP_ERROR(get_logger(), "GenICam exception thrown: %s", ge.what());
            std::abort();
        }
        catch (std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Standard exception thrown: %s", ex.what());
            std::abort();
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "Unexpected exception thrown");
            std::abort();
        }

        intensityImgTopic = "/" + frameID + "/intensity/image_raw";
        depthImgTopic = "/" + frameID + "/depth/image_raw";
        pointsTopic = "/" + frameID + "/points";
        camInfoTopic = "/" + frameID + "/camera_info";

        using namespace std::chrono_literals;
        // auto qos = rclcpp::SensorDataQoS().keep_last(1);
        pcPublisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointsTopic, 1);
        if (bPublishIntensity)
            intensityImgPublisher_ = create_publisher<sensor_msgs::msg::Image>(intensityImgTopic, 1);
        if (bPublishDepth)
            depthImgPublisher_ = create_publisher<sensor_msgs::msg::Image>(depthImgTopic, 1);
        camInfoPublisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(camInfoTopic, 1);

        rvec = cv::Mat::zeros(3, 1, CV_64F);
        tvec = cv::Mat::zeros(3, 1, CV_64F);

        pDevice->StartStream();

        initCameraInfo();
    }

    Arena::IDevice *LucidlabsHelios2::findDevice()
    {

        try
        {
            pSystem = Arena::OpenSystem();
            pSystem->UpdateDevices(SYSTEM_TIMEOUT);
            deviceInfos = pSystem->GetDevices();
            if (deviceInfos.size() == 0)
            {
                RCLCPP_ERROR(get_logger(), "No camera connected");
                std::abort();
            }

            unsigned int id;
            unsigned int found_id;
            bool found = false;
            for (id = 0; id < deviceInfos.size(); id++)
            {
                RCLCPP_INFO(get_logger(), "Model name: %s", deviceInfos[id].ModelName().c_str());
                if (deviceInfos[id].ModelName() == "HLT003S-001")
                {
                    m = cam_model::HELIOS_2;
                    found = true;
                    found_id = id;
                    break;
                }
                else if (deviceInfos[id].ModelName() == "HTP003S-001")
                {
                    m = cam_model::HELIOS_2_PLUS;
                    found = true;
                    found_id = id;
                    break;
                }
                else if (deviceInfos[id].ModelName() == "HTR003S-001")
                {
                    m = cam_model::HELIOS_2_RAY;
                    found = true;
                    found_id = id;
                    break;
                }
            }

            if (!found)
            {
                RCLCPP_ERROR(get_logger(), "No Helios 2 Camera found.");
                std::abort();
            }
            return pSystem->CreateDevice(deviceInfos[found_id]);
        }
        catch (GenICam::GenericException &ge)
        {
            RCLCPP_ERROR(get_logger(), "GenICam exception thrown: %s", ge.what());
            std::abort();
        }
        catch (std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Standard exception thrown: %s", ex.what());
            std::abort();
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "Unexpected exception thrown");
            std::abort();
        }
    }

    void LucidlabsHelios2::runtime()
    {
        auto img = tryGetImage();
        if (img != nullptr)
        {
            getPointCloudAndImages<pcl::PointXYZ>(img);
            // pDevice->RequeueBuffer(img);
        }
    }

    void LucidlabsHelios2::OnImage(Arena::IImage *img)
    {
        if (!img)
            return;

        if (bPublishIntensity)
            getPointCloudAndImages<pcl::PointXYZI>(img);
        else
            getPointCloudAndImages<pcl::PointXYZ>(img);

        // pDevice->RequeueBuffer(img);
    }

    LucidlabsHelios2::~LucidlabsHelios2()
    {
        pDevice->DeregisterImageCallback(pCallbackHandler);
        if (pDevice)
        {
            pDevice->StopStream();
            pSystem->DestroyDevice(pDevice);
        }
        if (pSystem)
        {
            Arena::CloseSystem(pSystem);
        }
    }

    Arena::IImage *LucidlabsHelios2::tryGetImage()
    {
        try
        {
            auto img = pDevice->GetImage(15);
            if (img->IsIncomplete())
            {
                RCLCPP_WARN(get_logger(), "Incomplete frame");
                // pDevice->RequeueBuffer(img);
                return nullptr;
            }
            return img;
        }
        catch (...)
        {
            return nullptr;
        }
    }

    // template <typename PointT>
    // void LucidlabsHelios2::getPointCloudAndImages(Arena::IImage *pImage)
    // {
    //     if (!pImage || pImage->IsIncomplete())
    //         return;

    //     rclcpp::Time stamp = clock->now();

    //     const int width = static_cast<int>(pImage->GetWidth());
    //     const int height = static_cast<int>(pImage->GetHeight());
    //     imgWidth_ = width;
    //     imgHeight_ = height;

    //     // // Determine if the buffer actually contains Y (ABCY16 vs ABC16)
    //     // const int64_t pf = pImage->GetPixelFormat();
    //     // const bool hasY = (pf == Coord3D_ABCY16) || (pf == Coord3D_ABCY16s);
    //     // STRIDE = hasY ? 4 : 3;

    //     // ---- Intensity image (mono16) ----
    //     uint16_t *intensityDst = nullptr;
    //     const bool publish_intensity_now = (bPublishIntensity && intensityImgPublisher_ && hasY);
    //     if (publish_intensity_now)
    //     {
    //         intensityMsg_.header.stamp = stamp;
    //         intensityMsg_.header.frame_id = frameID;
    //         const bool size_changed =
    //             (!imagesInitialized_) ||
    //             (static_cast<int>(intensityMsg_.width) != width) ||
    //             (static_cast<int>(intensityMsg_.height) != height);
    //         if (size_changed)
    //         {
    //             intensityMsg_.width = width;
    //             intensityMsg_.height = height;
    //             intensityMsg_.encoding = sensor_msgs::image_encodings::MONO16;
    //             intensityMsg_.is_bigendian = false;
    //             intensityMsg_.step = width * sizeof(uint16_t);
    //             intensityMsg_.data.resize(static_cast<size_t>(intensityMsg_.step) * intensityMsg_.height);
    //         }
    //         intensityDst = reinterpret_cast<uint16_t *>(intensityMsg_.data.data());
    //     }

    //     // ---- Depth image (32FC1) ----
    //     float *depthDst = nullptr;
    //     const bool publish_depth_now = (bPublishDepth && depthImgPublisher_);
    //     if (publish_depth_now)
    //     {
    //         depthMsg_.header.stamp = stamp;
    //         depthMsg_.header.frame_id = frameID;
    //         const bool size_changed =
    //             (!imagesInitialized_) ||
    //             (static_cast<int>(depthMsg_.width) != width) ||
    //             (static_cast<int>(depthMsg_.height) != height);
    //         if (size_changed)
    //         {
    //             depthMsg_.width = width;
    //             depthMsg_.height = height;
    //             depthMsg_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    //             depthMsg_.is_bigendian = false;
    //             depthMsg_.step = width * sizeof(float);
    //             depthMsg_.data.resize(static_cast<size_t>(depthMsg_.step) * depthMsg_.height);
    //         }
    //         depthDst = reinterpret_cast<float *>(depthMsg_.data.data());
    //     }
    //     imagesInitialized_ = true;

    //     // ---- Point cloud ----
    //     pcl::PointCloud<PointT> cloud_;
    //     cloud_.reserve(static_cast<size_t>(width) * height);
    //     if (bStructuredCloud)
    //     {
    //         cloud_.resize(static_cast<size_t>(width) * height);
    //         cloud_.width = width;
    //         cloud_.height = height;
    //     }

    //     const uint16_t *data = reinterpret_cast<const uint16_t *>(pImage->GetData());
    //     const int N = width * height;
    //     // PointT *p = nullptr;
    //     // bool lastInvalid = false;
    //     const float qnan = std::numeric_limits<float>::quiet_NaN();

    //     for (int o = 0; o < N; ++o)
    //     {
    //         const uint16_t A = data[o * STRIDE + 0];
    //         const uint16_t B = data[o * STRIDE + 1];
    //         const uint16_t C = data[o * STRIDE + 2];
    //         const uint16_t Y = hasY ? data[o * 4 + 3] : 0;

    //         const bool valid = (A != 0xFFFF && B != 0xFFFF && C != 0xFFFF);

    //         // Intensity
    //         if (publish_intensity_now)
    //             intensityDst[o] = valid ? Y : 0;

    //         // Depth (Z in meters)
    //         if (publish_depth_now)
    //             depthDst[o] = valid ? (C * sZ + oZ) : qnan;

    //         // Point Cloud
    //         // if (bStructuredCloud)
    //         // {
    //         //     p = &cloud_[o];
    //         // }
    //         // else if (!lastInvalid)
    //         // {
    //         //     cloud_.emplace_back();
    //         //     p = &cloud_.back();
    //         // }

    //         // if (A != 0xFFFF && B != 0xFFFF && C != 0xFFFF)
    //         // {
    //         //     p->x = (A * scaleX + offX) / 1000.0f;
    //         //     p->y = (B * scaleY + offY) / 1000.0f;
    //         //     p->z = (C * scaleZ + offZ) / 1000.0f;

    //         //     if constexpr (std::is_same<PointT, pcl::PointXYZI>::value)
    //         //         p->intensity = static_cast<float>(Y);

    //         //     lastInvalid = false;
    //         // }
    //         // else
    //         // {
    //         //     p->x = p->y = p->z = 0.0f;
    //         //     if constexpr (std::is_same<PointT, pcl::PointXYZI>::value)
    //         //         p->intensity = 0.0f;
    //         //     lastInvalid = true;
    //         // }
    //         if (bStructuredCloud)
    //         {
    //             PointT &pt = cloud_[o];
    //             if (valid)
    //             {
    //                 pt.x = A * sX + oX;
    //                 pt.y = B * sY + oY;
    //                 pt.z = C * sZ + oZ;

    //                 if constexpr (std::is_same<PointT, pcl::PointXYZI>::value)
    //                     pt.intensity = hasY ? static_cast<float>(Y) : 0.0f;
    //             }
    //             else
    //             {
    //                 pt.x = pt.y = pt.z = 0.0f;
    //                 if constexpr (std::is_same<PointT, pcl::PointXYZI>::value)
    //                     pt.intensity = 0.0f;
    //             }
    //         }
    //         else
    //         {
    //             if (valid)
    //             {
    //                 PointT pt;
    //                 pt.x = A * sX + oX;
    //                 pt.y = B * sY + oY;
    //                 pt.z = C * sZ + oZ;
    //                 if constexpr (std::is_same<PointT, pcl::PointXYZI>::value)
    //                     pt.intensity = hasY ? static_cast<float>(Y) : 0.0f;
    //                 cloud_.push_back(pt);
    //             }
    //         }
    //     }

    //     if (!bStructuredCloud)
    //     {
    //         cloud_.width = static_cast<uint32_t>(cloud_.size());
    //         cloud_.height = 1;
    //     }

    //     // if (!bStructuredCloud && lastInvalid && !cloud_.empty())
    //     //     cloud_.resize(cloud_.size() - 1);

    //     sensor_msgs::msg::PointCloud2 cloudMsg;
    //     pcl::toROSMsg(cloud_, cloudMsg);
    //     cloudMsg.header.stamp = stamp;
    //     cloudMsg.header.frame_id = frameID;
    //     cloudMsg.is_dense = false;

    //     pcPublisher_->publish(cloudMsg);

    //     if (publish_intensity_now)
    //         intensityImgPublisher_->publish(intensityMsg_);

    //     if (publish_depth_now)
    //         depthImgPublisher_->publish(depthMsg_);

    //     publishCameraInfo(stamp);
    // }

    template <typename PointT>
    void LucidlabsHelios2::getPointCloudAndImages(Arena::IImage *pImage)
    {
        if (!pImage || pImage->IsIncomplete())
            return;

        const rclcpp::Time stamp = clock->now();

        const int width = static_cast<int>(pImage->GetWidth());
        const int height = static_cast<int>(pImage->GetHeight());
        imgWidth_ = width;
        imgHeight_ = height;

        const float qnan = std::numeric_limits<float>::quiet_NaN();

        // ---------- Intensity (MONO16) ----------
        uint16_t *intensityDst = nullptr;
        const bool publish_intensity_now = (bPublishIntensity && intensityImgPublisher_ && hasY);

        if (publish_intensity_now)
        {
            intensityMsg_.header.stamp = stamp;
            intensityMsg_.header.frame_id = frameID;

            const bool size_changed =
                (!imagesInitialized_) ||
                (static_cast<int>(intensityMsg_.width) != width) ||
                (static_cast<int>(intensityMsg_.height) != height);

            if (size_changed)
            {
                intensityMsg_.width = width;
                intensityMsg_.height = height;
                intensityMsg_.encoding = sensor_msgs::image_encodings::MONO16;
                intensityMsg_.is_bigendian = false;
                intensityMsg_.step = width * sizeof(uint16_t);
                intensityMsg_.data.resize(static_cast<size_t>(intensityMsg_.step) * intensityMsg_.height);
            }

            intensityDst = reinterpret_cast<uint16_t *>(intensityMsg_.data.data());
        }

        // ---------- Depth (32FC1) ----------
        float *depthDst = nullptr;
        const bool publish_depth_now = (bPublishDepth && depthImgPublisher_);

        if (publish_depth_now)
        {
            depthMsg_.header.stamp = stamp;
            depthMsg_.header.frame_id = frameID;

            const bool size_changed =
                (!imagesInitialized_) ||
                (static_cast<int>(depthMsg_.width) != width) ||
                (static_cast<int>(depthMsg_.height) != height);

            if (size_changed)
            {
                depthMsg_.width = width;
                depthMsg_.height = height;
                depthMsg_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
                depthMsg_.is_bigendian = false;
                depthMsg_.step = width * sizeof(float);
                depthMsg_.data.resize(static_cast<size_t>(depthMsg_.step) * depthMsg_.height);
            }

            depthDst = reinterpret_cast<float *>(depthMsg_.data.data());
        }

        imagesInitialized_ = true;

        // ---------- Point Cloud ----------
        pcl::PointCloud<PointT> cloud_;
        if (bStructuredCloud)
        {
            cloud_.resize(static_cast<size_t>(width) * height);
            cloud_.width = width;
            cloud_.height = height;
        }
        else
        {
            cloud_.reserve(static_cast<size_t>(width) * height);
        }

        const uint16_t *data = reinterpret_cast<const uint16_t *>(pImage->GetData());
        const int N = width * height;

        for (int o = 0; o < N; ++o)
        {
            const uint16_t A = data[o * STRIDE + 0];
            const uint16_t B = data[o * STRIDE + 1];
            const uint16_t C = data[o * STRIDE + 2];
            const uint16_t Y = hasY ? data[o * STRIDE + 3] : 0;

            const bool valid = (A != 0xFFFF && B != 0xFFFF && C != 0xFFFF);

            if (publish_intensity_now)
                intensityDst[o] = valid ? Y : 0;

            if (publish_depth_now)
                depthDst[o] = valid ? (C * sZ + oZ) : qnan;

            if (bStructuredCloud)
            {
                PointT &pt = cloud_[o];
                if (valid)
                {
                    pt.x = A * sX + oX;
                    pt.y = B * sY + oY;
                    pt.z = C * sZ + oZ;

                    if constexpr (std::is_same<PointT, pcl::PointXYZI>::value)
                        pt.intensity = hasY ? static_cast<float>(Y) : 0.0f;
                }
                else
                {
                    // better than (0,0,0) because 0 becomes a "real" point at the origin
                    pt.x = pt.y = pt.z = qnan;
                    if constexpr (std::is_same<PointT, pcl::PointXYZI>::value)
                        pt.intensity = 0.0f;
                }
            }
            else
            {
                if (valid)
                {
                    PointT pt;
                    pt.x = A * sX + oX;
                    pt.y = B * sY + oY;
                    pt.z = C * sZ + oZ;
                    if constexpr (std::is_same<PointT, pcl::PointXYZI>::value)
                        pt.intensity = hasY ? static_cast<float>(Y) : 0.0f;
                    cloud_.push_back(pt);
                }
            }
        }

        if (!bStructuredCloud)
        {
            cloud_.width = static_cast<uint32_t>(cloud_.size());
            cloud_.height = 1;
        }

        sensor_msgs::msg::PointCloud2 cloudMsg;
        pcl::toROSMsg(cloud_, cloudMsg);
        cloudMsg.header.stamp = stamp;
        cloudMsg.header.frame_id = frameID;
        cloudMsg.is_dense = false;

        pcPublisher_->publish(cloudMsg);

        if (publish_intensity_now)
            intensityImgPublisher_->publish(intensityMsg_);

        if (publish_depth_now)
            depthImgPublisher_->publish(depthMsg_);

        publishCameraInfo(stamp);
    }

    void LucidlabsHelios2::initCameraInfo()
    {
        // Use actual device dimensions
        imgWidth_ = static_cast<int>(Arena::GetNodeValue<int64_t>(pNodeMap, "Width"));
        imgHeight_ = static_cast<int>(Arena::GetNodeValue<int64_t>(pNodeMap, "Height"));

        double fx = 1.0, fy = 1.0;
        double cx = imgWidth_ / 2.0;
        double cy = imgHeight_ / 2.0;

        try
        {
            if (nodeReadable(pNodeMap, "CalibFocalLengthX") &&
                nodeReadable(pNodeMap, "CalibFocalLengthY"))
            {
                fx = Arena::GetNodeValue<double>(pNodeMap, "CalibFocalLengthX");
                fy = Arena::GetNodeValue<double>(pNodeMap, "CalibFocalLengthY");

                if (nodeReadable(pNodeMap, "CalibOpticalCenterX"))
                    cx = Arena::GetNodeValue<double>(pNodeMap, "CalibOpticalCenterX");
                if (nodeReadable(pNodeMap, "CalibOpticalCenterY"))
                    cy = Arena::GetNodeValue<double>(pNodeMap, "CalibOpticalCenterY");

                RCLCPP_INFO(get_logger(), "CameraInfo using Calib* intrinsics");
            }
            else if (nodeReadable(pNodeMap, "Scan3dFocalLength"))
            {
                fx = fy = Arena::GetNodeValue<double>(pNodeMap, "Scan3dFocalLength");

                if (nodeReadable(pNodeMap, "Scan3dPrincipalPointU"))
                    cx = Arena::GetNodeValue<double>(pNodeMap, "Scan3dPrincipalPointU");
                if (nodeReadable(pNodeMap, "Scan3dPrincipalPointV"))
                    cy = Arena::GetNodeValue<double>(pNodeMap, "Scan3dPrincipalPointV");

                RCLCPP_INFO(get_logger(), "CameraInfo using Scan3d* intrinsics");
            }
            else
            {
                RCLCPP_WARN(get_logger(),
                            "No readable intrinsics nodes found; using defaults.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(),
                        "Exception reading intrinsics (%s); using defaults.", e.what());
        }
        catch (...)
        {
            RCLCPP_WARN(get_logger(),
                        "Unknown exception reading intrinsics; using defaults.");
        }

        camInfoMsg_.width = imgWidth_;
        camInfoMsg_.height = imgHeight_;

        // Distortion: try to populate if CalibLensDistortionModel exists
        camInfoMsg_.distortion_model = "plumb_bob";
        camInfoMsg_.d = {0, 0, 0, 0, 0};

        try
        {
            if (nodeReadable(pNodeMap, "CalibLensDistortionModel"))
            {
                auto model = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "CalibLensDistortionModel");
                camInfoMsg_.distortion_model = model.c_str();

                // Optional: many setups expose 8 distortion values like the example.
                // ROS CameraInfo typically uses 5 for plumb_bob, but we can store more.
                if (nodeReadable(pNodeMap, "CalibLensDistortionValueSelector") &&
                    nodeReadable(pNodeMap, "CalibLensDistortionValue"))
                {
                    std::vector<double> d;
                    static const char *selectors[] = {
                        "Value0", "Value1", "Value2", "Value3", "Value4", "Value5", "Value6", "Value7"};

                    for (auto s : selectors)
                    {
                        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "CalibLensDistortionValueSelector", s);
                        d.push_back(Arena::GetNodeValue<double>(pNodeMap, "CalibLensDistortionValue"));
                    }
                    camInfoMsg_.d = d;
                }
            }
        }
        catch (...)
        {
            // keep default distortion if anything goes wrong
        }

        camInfoMsg_.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        camInfoMsg_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        camInfoMsg_.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};

        camInfoReady_ = true;

        RCLCPP_INFO(get_logger(), "CameraInfo: fx=%.3f fy=%.3f cx=%.3f cy=%.3f, model=%s, d.size=%zu",
                    fx, fy, cx, cy, camInfoMsg_.distortion_model.c_str(), camInfoMsg_.d.size());
    }

    void LucidlabsHelios2::publishCameraInfo(const rclcpp::Time &stamp)
    {
        if (!camInfoReady_)
            return;

        camInfoMsg_.header.stamp = stamp;
        camInfoMsg_.header.frame_id = frameID;

        camInfoPublisher_->publish(camInfoMsg_);
    }

    bool LucidlabsHelios2::nodeReadable(GenApi::INodeMap *nm, const char *name)
    {
        if (!nm)
            return false;
        GenApi::INode *n = nm->GetNode(name);
        return n && GenApi::IsReadable(n);
    }

    void LucidlabsHelios2::readCalibrationFromHelios(Arena::IDevice *pDeviceHLT)
    {
        GenApi::INodeMap *nodeMap = pDeviceHLT->GetNodeMap();
        cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
        RCLCPP_INFO(get_logger(), "Reading intrinsics from nodes: CalibFocalLengthX, CalibFocalLengthY, CalibOpticalCenterX, CalibOpticalCenterY");
        cameraMatrix.at<double>(0, 0) = Arena::GetNodeValue<double>(nodeMap, "CalibFocalLengthX");
        cameraMatrix.at<double>(1, 1) = Arena::GetNodeValue<double>(nodeMap, "CalibFocalLengthY");
        cameraMatrix.at<double>(0, 2) = Arena::GetNodeValue<double>(nodeMap, "CalibOpticalCenterX");
        cameraMatrix.at<double>(1, 2) = Arena::GetNodeValue<double>(nodeMap, "CalibOpticalCenterY");
        cameraMatrix.at<double>(2, 2) = 1;
        RCLCPP_INFO(get_logger(), "Focal length is %0.5f pixels\n", cameraMatrix.at<double>(0, 0));
        RCLCPP_INFO(get_logger(), "Optical center is (%0.3f, %0.3f) pixels\n", cameraMatrix.at<double>(0, 2), cameraMatrix.at<double>(1, 2));
        const int n_distortion_coefficients = 5; // 5 for HLT/HTP, 8 for HTW
        distortionCoeffs = cv::Mat::zeros(1, n_distortion_coefficients, CV_64FC1);
        RCLCPP_INFO(get_logger(), "Reading %d distortion values from camera\n", n_distortion_coefficients);

        for (int i = 0; i < n_distortion_coefficients; ++i)
        {
            char selector_value[32];
            std::snprintf(selector_value, sizeof(selector_value), "Value%d", i);

            Arena::SetNodeValue<GenICam::gcstring>(
                nodeMap, "CalibLensDistortionValueSelector",
                GenICam::gcstring(selector_value));

            distortionCoeffs.at<double>(0, i) =
                Arena::GetNodeValue<double>(nodeMap, "CalibLensDistortionValue");
        }
    }

    cv::Mat LucidlabsHelios2::projectAll3DPointsOnImage(const cv::Mat &xyz_mm)
    {
        CV_Assert(xyz_mm.type() == CV_32FC3);
        const int H = xyz_mm.rows;
        const int W = xyz_mm.cols;

        const float qnan = std::numeric_limits<float>::quiet_NaN();
        cv::Mat uv(H, W, CV_32FC2, cv::Scalar(qnan, qnan));

        std::vector<cv::Point3f> obj;
        std::vector<int> linear_idx;
        obj.reserve(static_cast<size_t>(H) * W);
        linear_idx.reserve(static_cast<size_t>(H) * W);

        for (int r = 0; r < H; ++r)
        {
            const cv::Vec3f *row = xyz_mm.ptr<cv::Vec3f>(r);
            for (int c = 0; c < W; ++c)
            {
                const float X = row[c][0];
                const float Y = row[c][1];
                const float Z = row[c][2];

                // Only project valid points in front of the camera
                if (std::isfinite(X) && std::isfinite(Y) && std::isfinite(Z) && Z > 1e-6f)
                {
                    obj.emplace_back(X, Y, Z);
                    linear_idx.push_back(r * W + c);
                }
            }
        }

        if (obj.empty())
            return uv;

        std::vector<cv::Point2f> img;
        img.reserve(obj.size());

        cv::projectPoints(obj, rvec, tvec, cameraMatrix, distortionCoeffs, img);

        for (size_t i = 0; i < img.size(); ++i)
        {
            const int k = linear_idx[i];
            const int r = k / W;
            const int c = k % W;
            uv.at<cv::Vec2f>(r, c) = cv::Vec2f(img[i].x, img[i].y); // (u,v) = (col,row)
        }

        return uv;
    }

} // namespace hal
