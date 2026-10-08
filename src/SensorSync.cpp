/**
* This file is part of slict.
* 
* Copyright (C) 2020 Thien-Minh Nguyen <thienminh.nguyen at ntu dot edu dot sg>,
* School of EEE
* Nanyang Technological Univertsity, Singapore
* 
* For more information please see <https://britsknguyen.github.io>.
* or <https://github.com/brytsknguyen/slict>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* slict is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* slict is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with slict.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by Thien-Minh Nguyen on 01/08/22.
//

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <thread>
#include <condition_variable>
#include <unordered_map>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/PointField.h"
#include "slict/FeatureCloud.h"

// Package specials
// #include "preprocess.hpp"
#include "utility.h"

namespace
{
size_t PointFieldSize(uint8_t datatype)
{
    using PF = sensor_msgs::PointField;
    switch (datatype)
    {
        case PF::INT8:
        case PF::UINT8:
            return 1;
        case PF::INT16:
        case PF::UINT16:
            return 2;
        case PF::INT32:
        case PF::UINT32:
        case PF::FLOAT32:
            return 4;
        case PF::FLOAT64:
            return 8;
        default:
            return 0;
    }
}

bool HostIsBigEndian()
{
    const uint16_t value = 0x0102;
    return *reinterpret_cast<const uint8_t *>(&value) == 0x01;
}

template <typename T>
T ReadValue(const uint8_t *source, bool swap_bytes)
{
    array<uint8_t, sizeof(T)> bytes{};
    memcpy(bytes.data(), source, sizeof(T));
    if (swap_bytes && sizeof(T) > 1)
        reverse(bytes.begin(), bytes.end());

    T value;
    memcpy(&value, bytes.data(), sizeof(T));
    return value;
}

double ReadScalar(const uint8_t *source, uint8_t datatype, bool swap_bytes)
{
    using PF = sensor_msgs::PointField;
    switch (datatype)
    {
        case PF::INT8:    return static_cast<double>(ReadValue<int8_t>(source, false));
        case PF::UINT8:   return static_cast<double>(ReadValue<uint8_t>(source, false));
        case PF::INT16:   return static_cast<double>(ReadValue<int16_t>(source, swap_bytes));
        case PF::UINT16:  return static_cast<double>(ReadValue<uint16_t>(source, swap_bytes));
        case PF::INT32:   return static_cast<double>(ReadValue<int32_t>(source, swap_bytes));
        case PF::UINT32:  return static_cast<double>(ReadValue<uint32_t>(source, swap_bytes));
        case PF::FLOAT32: return static_cast<double>(ReadValue<float>(source, swap_bytes));
        case PF::FLOAT64: return ReadValue<double>(source, swap_bytes);
        default:          return numeric_limits<double>::quiet_NaN();
    }
}

const sensor_msgs::PointField *FindField(
    const unordered_map<string, sensor_msgs::PointField> &fields,
    const string &name)
{
    const auto it = fields.find(name);
    return it == fields.end() ? nullptr : &it->second;
}
} // namespace

struct CloudPacket
{
    double startTime;
    double endTime;

    CloudXYZITPtr cloud;
    
    CloudPacket(){};
    CloudPacket(double startTime_, double endTime_, CloudXYZITPtr cloud_)
        : startTime(startTime_), endTime(endTime_), cloud(cloud_)
    {}
};

class SensorSync
{
private:
        
    // Node handler
    ros::NodeHandlePtr nh_ptr;

    // Subscribers
    vector<ros::Subscriber> lidar_sub;
    ros::Subscriber imu_sub;

    mutex lidar_buf_mtx;
    mutex lidar_leftover_buf_mtx;
    mutex imu_buf_mtx;

    deque<deque<CloudPacket>> lidar_buf;
    deque<deque<CloudPacket>> lidar_leftover_buf;
    deque<sensor_msgs::Imu::ConstPtr> imu_buf;

    mutex merged_cloud_buf_mtx;
    deque<CloudPacket> merged_cloud_buf;

    ros::Publisher merged_pc_pub;
    ros::Publisher data_pub;

    // Lidar extrinsics
    deque<Matrix3d> R_B_L;
    deque<Vector3d> t_B_L;

    // IMU extrinsics
    Matrix3d R_B_I;
    Vector3d t_B_I;

    int MAX_THREAD = std::thread::hardware_concurrency();
    int Nlidar;
    int Nimu;

    vector<string> lidar_type;
    bool preserve_lidar_header = false;
    double airy_max_scan_duration = 0.25;

    double cutoff_time = -1;
    double cutoff_time_new = -1;
    double min_range = 0.5;
    vector<int> ds_rate = {1};
    int sweep_len = 1;

    thread sync_lidar;
    thread sync_data;

    double acce_scale = 1.0;

public:
    // Destructor
    ~SensorSync()
    {
        if (sync_lidar.joinable())
            sync_lidar.join();
        if (sync_data.joinable())
            sync_data.join();
    }

    SensorSync(ros::NodeHandlePtr &nh_ptr_) : nh_ptr(nh_ptr_)
    {
        // Initialize the variables and subsribe/advertise topics here
        Initialize();
    }

    void Initialize()
    {

        /* #region Lidar --------------------------------------------------------------------------------------------*/
        
        // Read the lidar topic
        vector<string> lidar_topic = {"/os_cloud_node/points"};
        nh_ptr->getParam("/lidar_topic", lidar_topic);

        Nlidar = lidar_topic.size();

        lidar_type = vector<string>(Nlidar, "ouster");
        nh_ptr->getParam("/lidar_type", lidar_type);
        ROS_ASSERT_MSG(lidar_type.size() == static_cast<size_t>(Nlidar),
                       "lidar_type must contain one entry per lidar topic");
        nh_ptr->param("/preserve_lidar_header", preserve_lidar_header, false);
        nh_ptr->param("/airy_max_scan_duration", airy_max_scan_duration, 0.25);

        // Read the extrincs of lidars
        vector<double> lidar_extr = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        nh_ptr->getParam("/lidar_extr", lidar_extr);

        ROS_ASSERT_MSG( (lidar_extr.size() / 16) == Nlidar,
                        "Lidar extrinsics not complete: %d < %d (= %d*16)\n",
                         lidar_extr.size(), Nlidar, Nlidar*16);

        printf("Received %d lidar(s) with extrinsics: \n", Nlidar);
        for(int i = 0; i < Nlidar; i++)
        {
            // Confirm the topics
            printf("Lidar topic #%02d: %s (%s)\n", i, lidar_topic[i].c_str(), lidar_type[i].c_str());

            Matrix4d extrinsicTf = Matrix<double, 4, 4, RowMajor>(&lidar_extr[i*16]);
            cout << "extrinsicTf: " << endl;
            cout << extrinsicTf << endl;

            R_B_L.push_back(extrinsicTf.block<3, 3>(0, 0));
            t_B_L.push_back(extrinsicTf.block<3, 1>(0, 3));

            lidar_buf.push_back(deque<CloudPacket>(0));
            lidar_leftover_buf.push_back(deque<CloudPacket>(0));

            // Subscribe to the lidar topic
            lidar_sub.push_back(nh_ptr->subscribe<sensor_msgs::PointCloud2>
                                            (lidar_topic[i], 100,
                                             boost::bind(&SensorSync::PcHandler, this,
                                                         _1, i, extrinsicTf(3, 2), (int)extrinsicTf(3, 3),
                                                         lidar_type[i])));
        }

        nh_ptr->getParam("/min_range", min_range);
        printf("Lidar minimum range: %f\n", min_range);

        nh_ptr->getParam("/ds_rate", ds_rate);
        printf("Down samping rate: ");
        for(auto rate : ds_rate)
            printf("%d ", rate);
        cout << endl;

        nh_ptr->getParam("/sweep_len", sweep_len);
        printf("Sweep len: %d\n", sweep_len);

        // Advertise lidar topic
        string merged_lidar_topic;
        nh_ptr->getParam("/merged_lidar_topic", merged_lidar_topic);
        merged_pc_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>(merged_lidar_topic, 100);

        /* #endregion Lidar -----------------------------------------------------------------------------------------*/


        /* #region IMU ----------------------------------------------------------------------------------------------*/

        // Read the IMU topic
        string imu_topic = "/imu_vn_100/imu";
        nh_ptr->getParam("/imu_topic", imu_topic);

        // Get the scale factor
        nh_ptr->getParam("/acce_scale", acce_scale);

        // Read the extrincs of lidars
        vector<double> imu_extr = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        nh_ptr->getParam("/imu_extr", imu_extr);

        // Confirm the topic(s)
        printf("IMU topic: %s\n", imu_topic.c_str());

        Matrix4d extrinsicTf = Matrix<double, 4, 4, RowMajor>(&imu_extr[0]);
        cout << "extrinsicTf: " << endl;
        cout << extrinsicTf << endl;

        R_B_I = extrinsicTf.block<3, 3>(0, 0);
        t_B_I = extrinsicTf.block<3, 1>(0, 3);

        // Subscribe to the IMU topic
        imu_sub = nh_ptr->subscribe<sensor_msgs::Imu>(imu_topic, 10000, &SensorSync::ImuHandler, this);

        /* #endregion IMU -------------------------------------------------------------------------------------------*/

        data_pub = nh_ptr->advertise<slict::FeatureCloud>("/sensors_sync", 100);

        // Create the synchronizing threads
        sync_lidar = thread(&SensorSync::SyncLidar, this);
        sync_data  = thread(&SensorSync::SyncData, this);
    }

    bool DecodeAiryCloud(const sensor_msgs::PointCloud2::ConstPtr &msg, int idx,
                         double time_offset, int stamp_type, CloudPacket &packet)
    {
        unordered_map<string, sensor_msgs::PointField> fields;
        for (const auto &field : msg->fields)
            fields[field.name] = field;

        for (const string &name : {"x", "y", "z", "timestamp"})
        {
            const sensor_msgs::PointField *field = FindField(fields, name);
            if (field == nullptr || field->count < 1 || PointFieldSize(field->datatype) == 0 ||
                static_cast<size_t>(field->offset) + PointFieldSize(field->datatype) > msg->point_step)
            {
                ROS_ERROR_THROTTLE(1.0, "Airy PointCloud2 has a missing or unreadable '%s' field", name.c_str());
                return false;
            }
        }

        const bool swap_bytes = msg->is_bigendian != HostIsBigEndian();
        const auto scalar = [&](const uint8_t *point, const string &name, double fallback)
        {
            const sensor_msgs::PointField *field = FindField(fields, name);
            if (field == nullptr || field->count < 1 || PointFieldSize(field->datatype) == 0 ||
                static_cast<size_t>(field->offset) + PointFieldSize(field->datatype) > msg->point_step)
                return fallback;
            return ReadScalar(point + field->offset, field->datatype, swap_bytes);
        };

        struct AiryPoint
        {
            PointXYZIT point;
            double raw_time;
        };

        vector<AiryPoint> decoded;
        decoded.reserve(static_cast<size_t>(msg->width) * msg->height);
        vector<double> raw_times;
        raw_times.reserve(decoded.capacity());

        for (uint32_t row = 0; row < msg->height; row++)
        {
            const size_t row_offset = static_cast<size_t>(row) * msg->row_step;
            for (uint32_t col = 0; col < msg->width; col++)
            {
                const size_t offset = row_offset + static_cast<size_t>(col) * msg->point_step;
                if (offset + msg->point_step > msg->data.size())
                    continue;

                const uint8_t *data = msg->data.data() + offset;
                const double x = scalar(data, "x", numeric_limits<double>::quiet_NaN());
                const double y = scalar(data, "y", numeric_limits<double>::quiet_NaN());
                const double z = scalar(data, "z", numeric_limits<double>::quiet_NaN());
                const double raw_time = scalar(data, "timestamp", numeric_limits<double>::quiet_NaN());
                if (!isfinite(x) || !isfinite(y) || !isfinite(z) || !isfinite(raw_time))
                    continue;
                if (sqrt(x*x + y*y + z*z) < min_range)
                    continue;

                Vector3d p_inB = R_B_L[idx] * Vector3d(x, y, z) + t_B_L[idx];
                const double intensity = scalar(data, "intensity", 0.0);

                AiryPoint airy_point;
                airy_point.point.x = static_cast<float>(p_inB.x());
                airy_point.point.y = static_cast<float>(p_inB.y());
                airy_point.point.z = static_cast<float>(p_inB.z());
                airy_point.point.intensity = isfinite(intensity) ? static_cast<float>(intensity) : 0.0f;
                airy_point.raw_time = raw_time;
                decoded.push_back(airy_point);
                raw_times.push_back(raw_time);
            }
        }

        if (decoded.empty())
        {
            ROS_WARN_THROTTLE(1.0, "Airy PointCloud2 contains no valid points");
            return false;
        }

        const auto raw_bounds = minmax_element(raw_times.begin(), raw_times.end());
        const double raw_span = *raw_bounds.second - *raw_bounds.first;
        vector<double> median_times = raw_times;
        nth_element(median_times.begin(), median_times.begin() + median_times.size()/2, median_times.end());
        const double median_time = median_times[median_times.size()/2];
        const double header_time = msg->header.stamp.toSec() + time_offset;

        const bool absolute_seconds =
            fabs(median_time - msg->header.stamp.toSec()) < 2.0 * airy_max_scan_duration &&
            raw_span <= airy_max_scan_duration;

        double relative_scale = 1.0;
        if (!absolute_seconds && raw_span > 1e-12)
        {
            const double candidates[] = {1.0, 1e-3, 1e-6, 1e-9};
            double best_error = numeric_limits<double>::infinity();
            for (double candidate : candidates)
            {
                const double duration = raw_span * candidate;
                if (duration <= 0.0 || duration > airy_max_scan_duration)
                    continue;
                const double error = fabs(duration - 0.1);
                if (error < best_error)
                {
                    best_error = error;
                    relative_scale = candidate;
                }
            }
        }

        CloudXYZITPtr cloud(new CloudXYZIT());
        cloud->reserve(decoded.size());
        double first_point_time = numeric_limits<double>::infinity();
        double last_point_time = -numeric_limits<double>::infinity();
        for (auto &airy_point : decoded)
        {
            const double point_time = absolute_seconds
                ? airy_point.raw_time + time_offset
                : header_time + (airy_point.raw_time - *raw_bounds.first) * relative_scale;
            if (!isfinite(point_time))
                continue;
            airy_point.point.t = point_time;
            first_point_time = min(first_point_time, point_time);
            last_point_time = max(last_point_time, point_time);
            cloud->push_back(airy_point.point);
        }

        sort(cloud->points.begin(), cloud->points.end(),
             [](const PointXYZIT &lhs, const PointXYZIT &rhs) { return lhs.t < rhs.t; });

        if (cloud->empty())
            return false;

        const double start_time = stamp_type == 1 ? header_time : min(header_time, first_point_time);
        const double end_time = stamp_type == 1 ? max(header_time, last_point_time) : header_time;
        if (!(end_time > start_time) || end_time - start_time > airy_max_scan_duration)
        {
            ROS_ERROR_THROTTLE(1.0, "Invalid Airy scan interval %.6f -> %.6f", start_time, end_time);
            return false;
        }

        static bool reported_schema = false;
        if (!reported_schema)
        {
            reported_schema = true;
            string names;
            for (const auto &field : msg->fields)
                names += (names.empty() ? "" : ",") + field.name;
            ROS_INFO("Airy PointCloud2 fields=[%s], points=%zu, time=%s, span=%.6f s, header_to_first=%.6f s",
                     names.c_str(), cloud->size(), absolute_seconds ? "absolute_seconds" : "relative_auto",
                     last_point_time - first_point_time, first_point_time - header_time);
        }

        packet = CloudPacket(start_time, end_time, cloud);
        return true;
    }

    void PcHandler(const sensor_msgs::PointCloud2::ConstPtr &msg, int idx, double time_offset,
                   int stamp_type, const string &type)
    {
        if (type == "airy")
        {
            CloudPacket packet;
            if (!DecodeAiryCloud(msg, idx, time_offset, stamp_type, packet))
                return;
            lock_guard<mutex> lock(lidar_buf_mtx);
            lidar_buf[idx].push_back(packet);
            return;
        }

        if (idx == 0)
        {
            // Lump the pointclouds together
            typedef sensor_msgs::PointCloud2::ConstPtr RosCloudPtr;
            static deque<RosCloudPtr> cloudBuf;
            cloudBuf.push_back(msg);

            if (cloudBuf.size() >= sweep_len)
            {
                double startTime = -1, endTime = -1;
                CloudXYZITPtr cloud_inB(new CloudXYZIT());

                // Extract the point, cloud by cloud
                for (RosCloudPtr &cloudMsg : cloudBuf)
                {
                    CloudOusterPtr cloud_inL(new CloudOuster());
                    pcl::fromROSMsg(*cloudMsg, *cloud_inL);

                    double cloud_start_time = -1, cloud_end_time = -1;
                    double sweep_dur = (cloud_inL->points.back().t - cloud_inL->points.front().t)/1.0e9;
                    double sweep_dur_err = fabs(sweep_dur - 0.1);
                    // ROS_ASSERT_MSG(sweep_dur_err < 5e-3, "Sweep length %f not exactly 0.1s.\n", sweep_dur, sweep_dur_err);
                    // Calculate the proper time stamps at the two ends
                    if (stamp_type == 1)
                    {
                        cloud_start_time = cloudMsg->header.stamp.toSec() + time_offset;
                        cloud_end_time   = cloud_start_time + sweep_dur;
                    }
                    else
                    {
                        cloud_end_time   = cloudMsg->header.stamp.toSec() + time_offset;
                        cloud_start_time = cloud_end_time - sweep_dur;
                    }

                    startTime = (startTime < 0) ? cloud_start_time : min(startTime, cloud_start_time);
                    endTime = (endTime < 0) ? cloud_end_time : max(endTime, cloud_end_time);

                    // Check the min dist and restamp the points
                    for(int i = 0; i < cloud_inL->size(); i++)
                    {
                        PointOuster &point_inL = cloud_inL->points[i];

                        // Discard of the point if range is zero
                        if (point_inL.range/1000.0 < min_range)
                            continue;

                        Vector3d p_inL(point_inL.x, point_inL.y, point_inL.z);
                        p_inL = R_B_L[idx]*p_inL + t_B_L[idx];

                        PointXYZIT point_inB;
                        point_inB.x = p_inL(0);
                        point_inB.y = p_inL(1);
                        point_inB.z = p_inL(2);
                        point_inB.intensity = point_inL.intensity;
                        point_inB.t = point_inL.t / 1e9 + cloud_start_time;

                        cloud_inB->push_back(point_inB);
                    }
                }

                lidar_buf_mtx.lock();
                lidar_buf[idx].push_back(CloudPacket(startTime, endTime, cloud_inB));
                lidar_buf_mtx.unlock();
                
                cloudBuf.clear();
            }
        }
        else if (idx != 0)
        {
            double startTime, endTime;
            if (stamp_type == 1)
            {
                startTime = msg->header.stamp.toSec() + time_offset;
                endTime   = startTime + 0.1;
            }
            else
            {
                endTime   = msg->header.stamp.toSec() + time_offset;
                startTime = endTime - 0.1;
            }

            CloudOusterPtr cloud_inL(new CloudOuster());
            pcl::fromROSMsg(*msg, *cloud_inL);

            CloudXYZITPtr cloud_inB(new CloudXYZIT());

            // Check the min dist and restamp the points
            for(int i = 0; i < cloud_inL->size(); i++)
            {
                PointOuster &point_inL = cloud_inL->points[i];

                // Discard of the point if range is zero
                if (point_inL.range/1000.0 < min_range)
                    continue;

                Vector3d p_inL(point_inL.x, point_inL.y, point_inL.z);
                p_inL = R_B_L[idx]*p_inL + t_B_L[idx];

                PointXYZIT point_inB;
                point_inB.x = p_inL(0);
                point_inB.y = p_inL(1);
                point_inB.z = p_inL(2);
                point_inB.intensity = point_inL.intensity;
                point_inB.t = point_inL.t / 1e9 + startTime;

                cloud_inB->push_back(point_inB);
            }
            
            lidar_buf_mtx.lock();
            lidar_buf[idx].push_back(CloudPacket(startTime, endTime, cloud_inB));
            lidar_buf_mtx.unlock();
        }
        else
            return;
    }

    void ImuHandler(const sensor_msgs::Imu::ConstPtr &msg)
    {
        imu_buf_mtx.lock();

        if (acce_scale != 1.0)
        {
            sensor_msgs::Imu::Ptr scaled_imu(new sensor_msgs::Imu());
            
            *scaled_imu = *msg;
            scaled_imu->linear_acceleration.x *= acce_scale;
            scaled_imu->linear_acceleration.y *= acce_scale;
            scaled_imu->linear_acceleration.z *= acce_scale;

            imu_buf.push_back(scaled_imu);
        }
        else
            imu_buf.push_back(msg);

        imu_buf_mtx.unlock();
    }
    
    void SyncLidar()
    {
        while(ros::ok())
        {
            // Loop if the secondary buffers don't over lap
            if(!LidarBufReady())
            {
                this_thread::sleep_for(chrono::milliseconds(10));
                continue;
            }

            // The challenge has one Airy LiDAR and evaluates poses at each
            // unmodified cloud header. Bypass the multi-lidar cutoff logic so
            // the next packet cannot inherit the previous scan's end time.
            if (preserve_lidar_header && Nlidar == 1)
            {
                CloudPacket packet;
                {
                    lock_guard<mutex> lock(lidar_buf_mtx);
                    packet = lidar_buf[0].front();
                    lidar_buf[0].pop_front();
                }
                if (!packet.cloud->empty())
                {
                    lock_guard<mutex> lock(merged_cloud_buf_mtx);
                    merged_cloud_buf.push_back(packet);
                }
                continue;
            }

            if (lidar_buf.size() > 1)
            {
                printf("Buf 0: Start: %.3f. End: %.3f / %.3f. Points: %d / %d. Size: %d\n"
                       "Buf 1: Start: %.3f. End: %.3f / %.3f. Points: %d / %d. Size: %d\n",
                        lidar_buf[0].front().startTime,
                        lidar_buf[0].front().endTime, lidar_buf[0].back().endTime,
                        lidar_buf[0].front().cloud->size(), lidar_buf[0].back().cloud->size(), lidar_buf[0].size(),
                        lidar_buf[1].front().startTime,
                        lidar_buf[1].front().endTime, lidar_buf[1].back().endTime,
                        lidar_buf[1].front().cloud->size(), lidar_buf[1].back().cloud->size(), lidar_buf[1].size());
            }

            // Extract the points
            CloudPacket extracted_points;
            ExtractLidarPoints(extracted_points);

            printf("Extracted: Start: %.3f. End: %.3f. Points: %d. CoT: %.3f. New CoT: %.3f\n",
                    extracted_points.startTime, extracted_points.endTime, extracted_points.cloud->size(),
                    cutoff_time, cutoff_time_new);
            if(!lidar_buf[0].empty())
            {
                printf("Buf 0: Start: %.3f. End: %.3f / %.3f. Points: %d / %d. Size: %d\n",
                       lidar_buf[0].front().startTime,
                       lidar_buf[0].front().endTime, lidar_buf[0].back().endTime,
                       lidar_buf[0].front().cloud->size(), lidar_buf[0].back().cloud->size(), lidar_buf[0].size());    
            }

            if (lidar_buf.size() > 1)
            {
                if(!lidar_buf[1].empty())
                {
                    printf("Buf 1: Start: %.3f. End: %.3f / %.3f. Points: %d / %d. Size: %d\n",
                            lidar_buf[1].front().startTime,
                            lidar_buf[1].front().endTime, lidar_buf[1].back().endTime,
                            lidar_buf[1].front().cloud->size(), lidar_buf[1].back().cloud->size(), lidar_buf[1].size());
                }
            }
            cout << endl;

            // Update the cutoff time
            cutoff_time = cutoff_time_new;

            // Store the merged pointcloud
            if(extracted_points.cloud->size() != 0)
            {
                lock_guard<mutex> lock(merged_cloud_buf_mtx);
                merged_cloud_buf.push_back(extracted_points);
            }
        }
    }

    bool LidarBufReady()
    {
        // If any buffer is empty, lidar is not ready
        for(int i = 0; i < Nlidar; i++)
        {
            if (lidar_buf[i].empty())
            {
                return false;
            }
        }
        
        // If any secondary buffer's end still has not passed the first endtime in the primary buffer, loop lah!
        for(int i = 1; i < Nlidar; i++)
        {
            if (lidar_buf[i].back().endTime < lidar_buf[0].front().endTime)
            {
                return false;
            }
        }

        return true;    
    }

    void ExtractLidarPoints(CloudPacket &extracted_points)
    {
        // Initiate the cutoff time.
        if (cutoff_time == -1)
            cutoff_time = lidar_buf[0].front().startTime;

        cutoff_time_new = lidar_buf[0].front().endTime;

        // Go through each buffer and extract the valid points
        deque<CloudXYZITPtr> extracted_clouds(Nlidar);
        for(int i = 0; i < Nlidar; i++)
        {
            extracted_clouds[i] = CloudXYZITPtr(new CloudXYZIT());

            CloudPacket leftover_cloud;
            leftover_cloud.startTime = cutoff_time_new;
            leftover_cloud.endTime = -1;
            leftover_cloud.cloud = CloudXYZITPtr(new CloudXYZIT());

            while(!lidar_buf[i].empty())
            {
                CloudPacket &front_cloud = lidar_buf[i].front();

                if ( front_cloud.endTime < cutoff_time )
                {
                    lock_guard<mutex> lock(lidar_buf_mtx);
                    lidar_buf[i].pop_front();
                    continue;
                }
                else if( front_cloud.startTime > cutoff_time_new )
                    break;

                int one_in_n = ds_rate[i];
                static int ds_count = -1;

                // Copy the points into the extracted pointcloud or the leftover
                for( auto &point : front_cloud.cloud->points )
                {
                    ds_count++; if (ds_count % one_in_n != 0) continue; // Skip one every

                    double point_time = point.t;

                    if (point_time >= cutoff_time && point_time <= cutoff_time_new)
                    {
                        extracted_clouds[i]->push_back(point);
                        // extracted_clouds[i]->points.back().t = point_time - cutoff_time;
                    }
                    else if (point_time > cutoff_time_new)
                    {
                        leftover_cloud.cloud->push_back(point);
                        // leftover_cloud.cloud->points.back().t = point_time - cutoff_time_new;

                        if (point_time > leftover_cloud.endTime)
                            leftover_cloud.endTime = point_time;
                    }
                }

                {
                    lock_guard<mutex> lock(lidar_buf_mtx);
                    lidar_buf[i].pop_front();
                }

                // Check the leftover buffer and insert extra points
                while(lidar_leftover_buf[i].size() != 0)
                {
                    if (lidar_leftover_buf[i].front().endTime < cutoff_time)
                    {
                        lidar_leftover_buf[i].pop_front();
                        continue;
                    }

                    if (lidar_leftover_buf[i].front().startTime > cutoff_time_new)
                        continue;

                    // Extract the first packet
                    CloudPacket leftover_frontcloud = lidar_leftover_buf[i].front();
                    lidar_leftover_buf[i].pop_front();

                    // Insert the leftover points back in the buffer
                    for( auto &point : leftover_frontcloud.cloud->points )
                    {
                        double point_time = point.t;

                        if (point_time >= cutoff_time && point_time <= cutoff_time_new)
                        {
                            extracted_clouds[i]->push_back(point);
                            // extracted_clouds[i]->points.back().t = point_time - cutoff_time;
                        }
                        else if (point_time > cutoff_time_new)
                        {
                            leftover_cloud.cloud->push_back(point);
                            // leftover_cloud.cloud->points.back().t = point_time - cutoff_time_new;

                            if (point_time > leftover_cloud.endTime)
                                leftover_cloud.endTime = point_time;
                        }
                    }
                }

                if (i == 0)
                    break;
            }

            if (leftover_cloud.cloud->size() > 0)
                lidar_leftover_buf[i].push_back(leftover_cloud);
        }

        // Merge the extracted clouds
        extracted_points.startTime = cutoff_time;
        extracted_points.endTime = cutoff_time_new;
        extracted_points.cloud = CloudXYZITPtr(new CloudXYZIT());
        for(int i = 0; i < Nlidar; i++)
            *extracted_points.cloud += *extracted_clouds[i];
    }

    bool ImuEmpty()
    {
        return imu_buf.empty();
    }

    double ImuEndTime()
    {
        return imu_buf.back()->header.stamp.toSec();
    }

    double ImuStartTime()
    {
        return imu_buf.front()->header.stamp.toSec();
    }

    void SyncData()
    {
        while (ros::ok())
        {
            /* #region Probing the key buffers ----------------------------------------------------------------------*/

            // Case 0: If any buffer is empty, then loop until all have data
            if (merged_cloud_buf.empty() || ImuEmpty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Case 1: If latest imu data is still earlier than the earliest
            // feature message, then wait for these measurements to catch up:

            // |___________[mfc]___mfc___mfc____________
            // |_imu_[imu]______________________________
            // |----------------------------------------> t+
            if (merged_cloud_buf.front().endTime > ImuEndTime())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Case 2: If earliest imu data is later than the earliest mfc's
            // end time, then discard the earliest cloud message.
            // This should only occur at the beginning

            // |_[mfc]___mfc___mfc__mfc________________
            // |_____________________[imu]_imu_imu_____
            // |---------------------------------------> t+
            if (merged_cloud_buf.front().endTime < ImuStartTime())
            {
                printf(KYEL "Popping mfc. Start: %.3f, End: %.3f. Size: %d\n" RESET,
                             merged_cloud_buf.front().startTime,
                             merged_cloud_buf.front().endTime,
                             merged_cloud_buf.front().cloud->size());

                merged_cloud_buf_mtx.lock();
                merged_cloud_buf.pop_front();
                merged_cloud_buf_mtx.unlock();
                continue;
            }

            // Case 3: A regular scenario, extract all imu measurement in the scan period

            // |________mfc___mfc___mfc________________
            // |_imu_imu_imu_imu_______________________
            // |---------------------------------------> t+

            // Extract the cloud packet
            CloudPacket merged_cloud = merged_cloud_buf.front();
            merged_cloud_buf_mtx.lock();
            merged_cloud_buf.pop_front();
            merged_cloud_buf_mtx.unlock();

            slict::FeatureCloud msg;
            msg.header.stamp    = ros::Time(merged_cloud.startTime);
            msg.extracted_cloud = Util::publishCloud(merged_pc_pub, *merged_cloud.cloud, ros::Time(merged_cloud.startTime), string("body"));
            msg.scanStartTime   = merged_cloud.startTime;
            msg.scanEndTime     = merged_cloud.endTime;

            vector<sensor_msgs::Imu> &imu_bundle = msg.imu_msgs;
            double imu_start_time = -1, imu_end_time = -1;
            while(!imu_buf.empty())
            {
                sensor_msgs::Imu imu_sample = *imu_buf.front();
                imu_sample.header.seq = 0;

                double imu_stamp = imu_sample.header.stamp.toSec();

                if ( imu_stamp <= merged_cloud.endTime )
                {
                    lock_guard<mutex> lock(imu_buf_mtx);
                    imu_buf.pop_front();
                }
                else
                {
                    // ASSUMPTION: Data from IMU i is available and has been stored
                    // ROS_ASSERT(!imu_bundle.empty());
                    // ROS_ASSERT_MSG(imu_bundle.back().header.seq == 0,
                    //                 "seq: %d. i: %d. Sz: %d. ScanEndTime: %f. IMUTime: %f\n",
                    //                 imu_bundle.back().header.seq, 0, imu_buf.size(), merged_cloud.endTime, imu_stamp);

                    // Linearly interpolating the imu sample
                    double ta = imu_bundle.back().header.stamp.toSec();
                    double tb = imu_sample.header.stamp.toSec();

                    // ASSUMPTION: IMU is spaced out
                    ROS_ASSERT( tb - ta > 0 );

                    Vector3d gyro_ta(imu_bundle.back().angular_velocity.x,
                                     imu_bundle.back().angular_velocity.y,
                                     imu_bundle.back().angular_velocity.z);
                    Vector3d gyro_tb(imu_sample.angular_velocity.x,
                                     imu_sample.angular_velocity.y,
                                     imu_sample.angular_velocity.z);
                    Vector3d acce_ta(imu_bundle.back().linear_acceleration.x,
                                     imu_bundle.back().linear_acceleration.y,
                                     imu_bundle.back().linear_acceleration.z);
                    Vector3d acce_tb(imu_sample.linear_acceleration.x,
                                     imu_sample.linear_acceleration.y,
                                     imu_sample.linear_acceleration.z);   
                    
                    // Make an interpolated sample
                    double t_itp = merged_cloud.endTime;
                    double s = (t_itp - ta)/(tb - ta);
                    Vector3d gyro_itp = (1-s)*gyro_ta + s*gyro_tb;
                    Vector3d acce_itp = (1-s)*acce_ta + s*acce_tb;
                    
                    imu_stamp = t_itp;
                    imu_sample.header.stamp = ros::Time(t_itp);
                    imu_sample.header.seq = 0;
                    imu_sample.angular_velocity.x = gyro_itp(0);
                    imu_sample.angular_velocity.y = gyro_itp(1);
                    imu_sample.angular_velocity.z = gyro_itp(2);
                    imu_sample.linear_acceleration.x = acce_itp(0);
                    imu_sample.linear_acceleration.y = acce_itp(1);
                    imu_sample.linear_acceleration.z = acce_itp(2);
                }
                
                if (imu_stamp >= merged_cloud.startTime && imu_stamp <= merged_cloud.endTime)
                {
                    imu_bundle.push_back(imu_sample);

                    if (imu_start_time == -1 || imu_start_time > imu_stamp)
                        imu_start_time = imu_stamp;

                    if (imu_end_time == -1 || imu_end_time < imu_stamp)
                        imu_end_time = imu_stamp;
                }

                if (imu_stamp == merged_cloud.endTime)
                    break;
            }

            /* #endregion Probing the key buffers -------------------------------------------------------------------*/

            // Publish the synchronized data
            data_pub.publish(msg);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_sync");
    ros::NodeHandle nh("~");
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    ROS_INFO(KGRN "----> Sensor Sync Started." RESET);

    SensorSync sensor_sync(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();
    
    return 0;
}
