/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OAKDataProvider.h
 * @brief  Parse OAK device Streams.
 * @author Sachin Guruswamy
 */

#include "kimera-vio/dataprovider/OAK3DFeatureDataProvider.h"

#include <algorithm>  // for max
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <utility>  // for pair<>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/YamlParser.h"
#include "kimera-vio/utils/Timer.h"


namespace VIO {

/* -------------------------------------------------------------------------- */
OAK3DFeatureDataProvider::OAK3DFeatureDataProvider(const std::string& dataset_path, const VioParams& vio_params)
    : OAKDataProvider(dataset_path, vio_params), depth_image_count_(0), 
    feature_msg_count_(0), depth_image_fps_(0), feature_msg_fps_(0) {
        // spinInputRosBag();
    }


/* -------------------------------------------------------------------------- */
OAK3DFeatureDataProvider::~OAK3DFeatureDataProvider() {
  LOG(INFO) << "OAK3DFeatureDataProvider destructor called.";
}

void OAK3DFeatureDataProvider::setDepthFeatureQueues(std::shared_ptr<dai::DataOutputQueue> depth_queue, std::shared_ptr<dai::DataOutputQueue> features_queue){
    depth_queue_ = depth_queue;
    feature_queue_ = features_queue;
}

void OAK3DFeatureDataProvider::setRightInputQueues(std::shared_ptr<dai::DataInputQueue> right_input_queue){
      right_input_queue_ = right_input_queue;
}

bool OAK3DFeatureDataProvider::spinInputRosBag() {
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> image_serialization;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    int64_t left_image_seq = 0, right_image_seq = 0;
    if (dataset_path_.empty()) {
        throw std::runtime_error("ROSbag data path is empty.");
    }
    while (reader_.has_next()) {
        auto bag_message = reader_.read_next();
        // topics.push_back(bag_message->topic_name);
        // TODO(Sachin): Remove hardcoded topic names
        if( bag_message->topic_name == "/oaks2/left/image_raw/compressed" || bag_message->topic_name == "/oaks2/right/image_raw/compressed"){
            sensor_msgs::msg::CompressedImage extracted_image_msg;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            image_serialization.deserialize_message(&extracted_serialized_msg, &extracted_image_msg);
            // VLOG(2) << "Topic name -> " << bag_message->topic_name << " With Serialized timestamp (Mostly record time ?) -> " << bag_message->time_stamp;
            // dai_time_point time_point = rosTimeStamp(extracted_image_msg.header.stamp);
            dai::ImgFrame dai_image;
            rosCompressedImageMsgToDepthaiImage(extracted_image_msg, dai_image);
            if (bag_message->topic_name == "/oaks2/left/image_raw/compressed"){
                dai_image.setInstanceNum(1);
                dai_image.setSequenceNum(left_image_seq);
                left_image_seq += 1;
                left_input_queue_->send(dai_image);
            }
            else if (bag_message->topic_name == "/oaks2/right/image_raw/compressed"){
                dai_image.setInstanceNum(2);
                dai_image.setSequenceNum(right_image_seq);
                right_image_seq += 1;
                right_input_queue_->send(dai_image);
            }
            // VLOG(2) << "Topic name -> " << bag_message->topic_name << " And timestamp in epoch ->  " << time_point.time_since_epoch().count() << " nanosec";
            // VLOG(2) << "Topic name -> " << bag_message->topic_name << " of format -> " << extracted_image_msg.format;
        }
        if (bag_message->topic_name == "/oaks2/imu/data" ){
            sensor_msgs::msg::Imu extracted_imu_msg;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            imu_serialization.deserialize_message(&extracted_serialized_msg, &extracted_imu_msg);
            ImuMeasurement imu_measurement;
            rosImuMsgToImuMeasurement(extracted_imu_msg, imu_measurement);
            if (imu_measurement.timestamp_ > last_input_imu_timestamp_){
                last_input_imu_timestamp_ = imu_measurement.timestamp_;
                imu_single_callback_(imu_measurement);
            }
            else{
               LOG(ERROR) << "Dropping an IMU timestanp that is not monotonically increasing";
            }
        }
    }
    return true;
}

/* -------------------------------------------------------------------------- */
bool OAK3DFeatureDataProvider::spin() {
    // Spin.
    CHECK_EQ(vio_params_.camera_params_.size(), 2u); // As usual mimicing stereo
    LOG(INFO) << "Data OAK3DFeatureDataProvider Interface: <-------------- Spinning -------------->";
    auto data_grab_tic = VIO::utils::Timer::tic();

    std::string name = "";
    while (!shutdown_) {

        std::shared_ptr<dai::ADatatype> left_image  = left_queue_->get<dai::ADatatype>();
        std::shared_ptr<dai::ADatatype> depth_image = depth_queue_->get<dai::ADatatype>();
        std::shared_ptr<dai::ADatatype> feature_map = feature_queue_->get<dai::ADatatype>();

        if (dataset_path_.empty()) {
            std::vector<std::shared_ptr<dai::ADatatype>> imu_measurements_list = imu_queue_->getAll<dai::ADatatype>();
            for(auto& imu_measurements : imu_measurements_list) {
                imuCallback(name, imu_measurements);
            }
        }
        syncImageFeatureGrab(left_image, depth_image, feature_map);
        while(!sync_msgs_.empty()){
            std::shared_ptr<dai::ADatatype> synced_left_image, synced_depth_image, synced_feature_map;
            std::tie(synced_left_image, synced_depth_image, synced_feature_map) = sync_msgs_.front();
            sync_msgs_.pop();
            leftImageFeatureCallback(synced_left_image, synced_feature_map);
            depthImageCallback("depth", synced_depth_image);
            feature_msg_count_++;
            depth_image_count_++;
            left_image_count_++;
            std::shared_ptr<dai::ImgFrame> image_msg  = std::dynamic_pointer_cast<dai::ImgFrame>(synced_left_image);
            recent_left_image_timestamp_ = timestampAtFrame(image_msg->getTimestamp());
        }
        if (!vio_params_.parallel_run_) {
            return true;
        }
        bool mod_count = !(left_image_count_ % 10);
        if (mod_count) {
            auto spin_duration = VIO::utils::Timer::toc(data_grab_tic);
            auto spin_duration_double = std::chrono::duration<double, std::milli>(spin_duration);
            VLOG(4) << "\n Left image count: " << left_image_count_
                    << "\n Imu msg count:    " << imu_msg_count_
                    << "\n Left FPS: " << (left_image_count_ / spin_duration_double.count()) * 1000 // converting to secs
                    << "\n Imu hz:   " << (imu_msg_count_ / spin_duration_double.count()) * 1000 // converting to secs
                    << "\n recent_left_image_timestamp_ ->: " << recent_left_image_timestamp_
                    << "\n recent_imu_timestamp_        ->: " << recent_imu_timestamp_;
        }
    //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  LOG_IF(INFO, shutdown_) << "OAK3DFeature DataProvider shutdown requested.";
  return false;
}

// TODO(saching): This sync function is leading to a lot of backlog images in queues
void OAK3DFeatureDataProvider::syncImageFeatureGrab(std::shared_ptr<dai::ADatatype> image, std::shared_ptr<dai::ADatatype> depth, std::shared_ptr<dai::ADatatype> feature_map){
    left_sync_queue_.push(image);
    depth_sync_queue_.push(depth);
    feature_sync_queue_.push(feature_map);
    if (left_sync_queue_.size() > 8 || depth_sync_queue_.size() > 8 || feature_sync_queue_.size() > 8){
        LOG(ERROR) << "Queue Sizes exceeded the max of 8 and has "<< left_sync_queue_.size() << ", " << depth_sync_queue_.size() << " and " << feature_sync_queue_.size();
    }
    while(!left_sync_queue_.empty() && !depth_sync_queue_.empty() && !feature_sync_queue_.empty()){
        std::shared_ptr<dai::ImgFrame> left_msg  = std::dynamic_pointer_cast<dai::ImgFrame>(left_sync_queue_.front());
        std::shared_ptr<dai::ImgFrame> depth_msg = std::dynamic_pointer_cast<dai::ImgFrame>(depth_sync_queue_.front());
        std::shared_ptr<dai::TrackedFeatures> feature_msg = std::dynamic_pointer_cast<dai::TrackedFeatures>(feature_sync_queue_.front());

        int64_t left_seq_num = left_msg->getSequenceNum();
        int64_t depth_seq_num = depth_msg->getSequenceNum();
        int64_t feature_seq_num = feature_msg->getSequenceNum();
        if (left_seq_num == depth_seq_num && left_seq_num == feature_seq_num){
            depth_msg->setTimestamp(left_msg->getTimestamp());
            feature_msg->setTimestamp(left_msg->getTimestamp());
            sync_msgs_.push(std::make_tuple(left_sync_queue_.front(), depth_sync_queue_.front(), feature_sync_queue_.front()));

            left_sync_queue_.pop();
            depth_sync_queue_.pop();
            feature_sync_queue_.pop();
        }
        else {
            int64_t maxSeq = std::max(std::max(left_seq_num, depth_seq_num), feature_seq_num);
            if(left_seq_num < maxSeq){
                left_sync_queue_.pop();
            }
            if(depth_seq_num < maxSeq){
                depth_sync_queue_.pop();
            }
            if(feature_seq_num < maxSeq){
                feature_sync_queue_.pop();
            }
        }
    }
}

void OAK3DFeatureDataProvider::leftImageFeatureCallback(std::shared_ptr<dai::ADatatype> image, std::shared_ptr<dai::ADatatype> feature_map){

    CHECK(left_frame_callback_) << "Did you forget to register the left image callback to the VIO Pipeline?";
    auto daiDataPtr = std::dynamic_pointer_cast<dai::ImgFrame>(image);
    auto tracked_features = std::dynamic_pointer_cast<dai::TrackedFeatures>(feature_map)->trackedFeatures;
    VLOG(3) << "Left image frame seq number >------------> " << daiDataPtr->getSequenceNum();
    cv::Mat imageFrame = daiDataPtr->getCvFrame();    
    Timestamp localTimestamp = timestampAtFrame(daiDataPtr->getTimestamp());
    // TODO(saching): Add option to equalize the image from histogram
    std::unique_ptr<Frame> left_frame = VIO::make_unique<Frame>(daiDataPtr->getSequenceNum(),
                                                                localTimestamp,
                                                                left_cam_info_,
                                                                imageFrame);
    left_frame->landmarks_.reserve(tracked_features.size());
    left_frame->landmarks_age_.reserve(tracked_features.size());
    left_frame->scores_.reserve(tracked_features.size());
    left_frame->keypoints_.reserve(tracked_features.size());
    left_frame->versors_.reserve(tracked_features.size());
    VLOG(6) << "Total Tracked Points : " << tracked_features.size();

    for (auto& feature : tracked_features) {
        KeypointCV keypoint(feature.position.x, feature.position.y);
        left_frame->landmarks_.push_back(feature.id);
        left_frame->landmarks_age_.push_back(feature.age);
        left_frame->scores_.push_back(feature.harrisScore);
        left_frame->keypoints_.push_back(keypoint);
        left_frame->versors_.push_back(
        UndistorterRectifier::UndistortKeypointAndGetVersor(keypoint, left_frame->cam_param_));
        // VLOG(12) << "Printing KeypointCV: x -> " << keypoint.x << " y -> " << keypoint.y << " id -> " << feature.id;
    } 
    left_frame_callback_(std::move(left_frame));
}

void OAK3DFeatureDataProvider::depthImageCallback(std::string name, std::shared_ptr<dai::ADatatype> data){
    CHECK(depth_frame_callback_) << "Did you forget to register the depth image callback to the VIO Pipeline?";
    auto daiDataPtr = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat imageFrame = daiDataPtr->getFrame();
    Timestamp localTimestamp = timestampAtFrame(daiDataPtr->getTimestamp());
    
    depth_frame_callback_(
        VIO::make_unique<DepthFrame>(daiDataPtr->getSequenceNum(),
                                localTimestamp,
                                imageFrame));
}

}  // namespace VIO