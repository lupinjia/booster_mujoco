#ifndef DEPTH_DISPLAY_HPP
#define DEPTH_DISPLAY_HPP

#include "mujoco/simulate.h"
#include "utils/recurrent_thread.hpp"
#include <opencv2/opencv.hpp>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/idl/sensor_msgs/Image.h>
#include "param.h"
#include <random>
#include <vector>
#include <algorithm>
#include <cmath>

namespace mj = ::mujoco;
using namespace booster::robot;
#define TOPIC_DEPTHIMAGE "rt/depthimage"

class DepthDisplay
{
public:
    DepthDisplay(const mj::Simulate *sim, int width, 
                int height, int crop_left, float depth_dt_s,
                int domain_id, const std::string& interface)
        : mj_sim_(sim), width_(width), height_(height), 
        crop_left_(crop_left)
    {
        depth_dt_us_ = static_cast<int>(depth_dt_s * 1e6);
        ChannelFactory::Instance()->Init(domain_id, interface);
        initDepthImagePublisherDDS();
        printf("DepthDisplay: initialized depth image publisher on topic %s\n", TOPIC_DEPTHIMAGE);
        thread_ = std::make_shared<booster::common::RecurrentThread>(
            "DepthDisplay", 
            3, depth_dt_us_, 
            std::bind(&DepthDisplay::displayDepthImage, this));
    }

    void initDepthImagePublisherDDS()
    {
        depth_image_msg_.height() = height_;
        depth_image_msg_.width() = width_ - crop_left_;
        depth_image_msg_.encoding() = "mono8";
        depth_image_msg_.is_bigendian() = 0;
        depth_image_msg_.step() = width_ - crop_left_;
        depth_image_msg_.header().frame_id() = "camera_depth_frame";
        depth_image_msg_.data().resize((width_ - crop_left_) * height_);
        std::fill(depth_image_msg_.data().begin(), depth_image_msg_.data().end(), 0);
        depth_image_publisher.reset(new ChannelPublisher<sensor_msgs::msg::Image>(TOPIC_DEPTHIMAGE));
        depth_image_publisher->InitChannel();
    }
    // Convert of the original Python simulation helpers to C++.
    // Note: many of the original "config" fields used by the Python
    // snippets are not present in param::config; provide reasonable
    // defaults here, and prefer param::config where available.
    cv::Mat add_depth_artifacts(const cv::Mat& artifacts_buffer,
                                float artifacts_prob,
                                const std::array<float,2>& height_mean_std,
                                const std::array<float,2>& width_mean_std)
    {
        CV_Assert(artifacts_buffer.type() == CV_32F);
        int H = artifacts_buffer.rows;
        int W = artifacts_buffer.cols;

        cv::Mat out = artifacts_buffer.clone();
        std::mt19937 rng(static_cast<unsigned int>(cv::getTickCount()));
        std::uniform_real_distribution<float> uni01(0.f, 1.f);
        std::normal_distribution<float> norm01(0.f, 1.f);

        // Generate artifact seed mask
        std::vector<cv::Point> coords;
        coords.reserve(256);
        for (int r = 0; r < H; ++r)
        {
            for (int c = 0; c < W; ++c)
            {
                if (uni01(rng) < artifacts_prob)
                    coords.emplace_back(c, r);
            }
        }

        if (coords.empty())
            return out;

        for (const auto &p : coords)
        {
            float h = std::max(1.f, height_mean_std[0] + norm01(rng) * height_mean_std[1]);
            float w = std::max(1.f, width_mean_std[0] + norm01(rng) * width_mean_std[1]);
            int half_h = static_cast<int>(std::round(h / 2.f));
            int half_w = static_cast<int>(std::round(w / 2.f));

            int top = std::max(0, p.y - half_h);
            int bottom = std::min(H, p.y + half_h);
            int left = std::max(0, p.x - half_w);
            int right = std::min(W, p.x + half_w);

            if (top >= bottom || left >= right)
                continue;

            // Set region to zero (artifact hole)
            cv::Rect roi(left, top, right - left, bottom - top);
            out(roi).setTo(0.f);
        }

        return out;
    }

    // Add stereo-like artifacts to a depth image (in-place)
    void add_depth_stereo(cv::Mat& image)
    {
        CV_Assert(image.type() == CV_32F);
        int H = image.rows;
        int W = image.cols;

        // masks
        cv::Mat far_mask = image > param::config.far_clip;
        cv::Mat too_close_mask = image < param::config.near_clip;
        cv::Mat near_mask = (far_mask == 0) & (too_close_mask == 0);

        std::mt19937 rng(static_cast<unsigned int>(cv::getTickCount()));
        std::uniform_real_distribution<float> uni01(0.f, 1.f);
        std::uniform_real_distribution<float> farNoiseDist(0.f, param::config.far_noise);
        std::uniform_real_distribution<float> nearNoiseDist(0.f, param::config.near_noise);

        // add noise to far points
        for (int r = 0; r < H; ++r)
        {
            float* img_ptr = image.ptr<float>(r);
            const uchar* far_ptr = far_mask.ptr<uchar>(r);
            const uchar* near_ptr = near_mask.ptr<uchar>(r);
            for (int c = 0; c < W; ++c)
            {
                if (far_ptr[c])
                    img_ptr[c] += farNoiseDist(rng);
                else if (near_ptr[c])
                    img_ptr[c] += nearNoiseDist(rng);
            }
        }

        // compute vertical_block_mask: columns where too_close pixels > 60% of H
        cv::Mat col_count = cv::Mat::zeros(1, W, CV_32S);
        for (int c = 0; c < W; ++c)
        {
            int cnt = 0;
            for (int r = 0; r < H; ++r)
                if (too_close_mask.at<uchar>(r, c)) ++cnt;
            col_count.at<int>(0, c) = cnt;
        }
        std::vector<char> vertical_block_col(W, 0);
        for (int c = 0; c < W; ++c)
            vertical_block_col[c] = (col_count.at<int>(0, c) > static_cast<int>(H * 0.6f));

        cv::Mat full_block_mask = cv::Mat::zeros(H, W, CV_8U);
        cv::Mat half_block_mask = cv::Mat::zeros(H, W, CV_8U);
        for (int r = 0; r < H; ++r)
        for (int c = 0; c < W; ++c)
        {
            if (too_close_mask.at<uchar>(r,c))
            {
                if (vertical_block_col[c]) full_block_mask.at<uchar>(r,c) = 255;
                else half_block_mask.at<uchar>(r,c) = 255;
            }
        }

        // add artifacts where vertical pixels are all too close
        std::vector<float> values = param::config.stereo_full_block_values;
        std::shuffle(values.begin(), values.end(), rng);
        for (float pixel_value : values)
        {
            cv::Mat artifacts_buffer = cv::Mat::ones(H, W, CV_32F);
            artifacts_buffer = add_depth_artifacts(artifacts_buffer,
                                                  param::config.stereo_full_block_artifacts_prob,
                                                  param::config.stereo_full_block_height_mean_std,
                                                  param::config.stereo_full_block_width_mean_std);
            for (int r = 0; r < H; ++r)
            for (int c = 0; c < W; ++c)
            {
                if (full_block_mask.at<uchar>(r,c))
                {
                    float factor = 1.f - artifacts_buffer.at<float>(r,c);
                    image.at<float>(r,c) = factor * pixel_value;
                }
            }
        }

        // half block sparks
        std::uniform_real_distribution<float> sparkDist(0.f, 1.f);
        for (int r = 0; r < H; ++r)
        for (int c = 0; c < W; ++c)
        {
            if (half_block_mask.at<uchar>(r,c))
            {
                if (sparkDist(rng) < param::config.stereo_half_block_spark_prob)
                    image.at<float>(r,c) = param::config.stereo_half_block_value;
            }
        }
    }

    // Recognize top-down columns where all pixels above are too-far
    cv::Mat recognize_top_down_seeing_sky(const cv::Mat& too_far_mask)
    {
        CV_Assert(too_far_mask.type() == CV_8U);
        int H = too_far_mask.rows;
        int W = too_far_mask.cols;
        cv::Mat result = cv::Mat::zeros(H, W, CV_8U);

        // For each column compute cumulative sum and compare
        for (int c = 0; c < W; ++c)
        {
            int cum = 0;
            for (int r = 0; r < H; ++r)
            {
                if (too_far_mask.at<uchar>(r,c)) ++cum;
                if (cum > r) // all pixels above (0..r-1) are too_far
                    result.at<uchar>(r,c) = 255;
            }
        }

        return result;
    }

    // Add sky-like artifacts
    void add_sky_artifacts(cv::Mat& image)
    {
        CV_Assert(image.type() == CV_32F);
        int H = image.rows;
        int W = image.cols;

        // masks
        cv::Mat possible_to_sky_mask = image > param::config.sky_artifacts_far_distance;
        cv::Mat possible_mask_8u;
        possible_to_sky_mask.convertTo(possible_mask_8u, CV_8U);
        cv::Mat to_sky_mask = recognize_top_down_seeing_sky(possible_mask_8u);

        std::mt19937 rng(static_cast<unsigned int>(cv::getTickCount()));
        for (float pixel_value : param::config.sky_artifacts_values)
        {
            cv::Mat artifacts_buffer = cv::Mat::ones(H, W, CV_32F);
            artifacts_buffer = add_depth_artifacts(artifacts_buffer,
                                                  param::config.sky_artifacts_prob,
                                                  param::config.sky_artifacts_height_mean_std,
                                                  param::config.sky_artifacts_width_mean_std);

            for (int r = 0; r < H; ++r)
            for (int c = 0; c < W; ++c)
            {
                if (!to_sky_mask.at<uchar>(r,c)) continue;

                float buf = artifacts_buffer.at<float>(r,c);
                float& val = image.at<float>(r,c);
                if (std::isfinite(val))
                    val *= buf;
                else // inf
                {
                    if (buf < 1.f)
                        val = 0.f;
                }

                // add weighted pixel_value where artifact present
                val += (1.f - buf) * pixel_value;
            }
        }
    }
    void displayDepthImage()
    {
        // Assuming depth image is rendered to a framebuffer or texture
        // 32 bit float channel 1
        cv::Mat depth_image(height_, width_ - crop_left_, CV_32FC1);

        // Fill depth_image with simulated depth data
        if (mj_sim_->depth_value_ == nullptr)
        {
            // fill with zeros if mj_sim_->depth_value_ is not initialized
            depth_image.setTo(cv::Scalar(0));
        }
        else
        {
            for (int i = 0; i < height_; ++i)
            {
                for (int j = 0; j < width_ - crop_left_; ++j)
                {
                    depth_image.at<float>(i, j) = mj_sim_->depth_value_[i * (width_ - crop_left_) + j];
                }
            }
        }
        // flip around y axis
        cv::flip(depth_image, depth_image, 0);
        // add stereo artifacts
        if(param::config.add_depth_stereo_noise)
        {
            add_depth_stereo(depth_image);
        }
        // add sky  artifacts
        if(param::config.add_sky_artifacts)
        {
            add_sky_artifacts(depth_image);
        }
        // normalize into [0, 1] and pack into uint8 mono image
        for (int i = 0; i < height_; ++i)
        {
            for (int j = 0; j < width_ - crop_left_; ++j)
            {
                float normalized = (depth_image.at<float>(i, j) - param::config.near_clip)
                                   / (param::config.far_clip - param::config.near_clip);
                depth_image.at<float>(i, j) = normalized;
                depth_image_msg_.data()[i * (width_ - crop_left_) + j] =
                    static_cast<uint8_t>(std::max(0.0f, std::min(1.0f, normalized)) * 255.0f);
            }
        }
        // publish depth image message
        if (depth_image_publisher != nullptr)
        {
            depth_image_publisher->Write(&depth_image_msg_);
        }
        if (param::config.visualize_opencv)
        {
            // Convert to 8-bit for display
            cv::Mat depth_image_8u;
            depth_image.convertTo(depth_image_8u, CV_8UC1, 255.0);

            // Display the depth image using OpenCV
            cv::imshow("Depth Image", depth_image_8u);
            cv::waitKey(1); // Wait for 1 ms to allow image to be displayed
        }
    }

private:
    const mj::Simulate* mj_sim_;
    int width_;
    int height_;
    int crop_left_;
    int depth_dt_us_; // depth display interval in microseconds
    booster::common::RecurrentThreadPtr thread_;
    ChannelPublisherPtr<sensor_msgs::msg::Image> depth_image_publisher;
    sensor_msgs::msg::Image depth_image_msg_;
};

#endif // DEPTH_DISPLAY_HPP
