/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#include <fstream>

#include "op3_ball_detector/ball_detector.h"

namespace robotis_op
{

BallDetector::BallDetector()
  : nh_(ros::this_node::getName()),
    it_(this->nh_),
    enable_(true),
    params_config_(),
    init_param_(false),
    not_found_count_(0),
    switch_detection_flag_(false)
{
  has_path_ = nh_.getParam("yaml_path", param_path_);
  has_color_config_ = nh_.getParam("color_path", color_config_path_);

  if (has_path_)
    std::cout << "Path : " << param_path_ << std::endl;
  if(has_color_config_)
    std::cout << "Color config path : " << color_config_path_ << std::endl;

  //detector config struct
  DetectorConfig detect_config;

  //get user parameters from dynamic reconfigure (yaml entries)
  nh_.param<int>("gaussian_blur_size", detect_config.gaussian_blur_size, params_config_.gaussian_blur_size);
  if (detect_config.gaussian_blur_size % 2 == 0)
    detect_config.gaussian_blur_size -= 1;
  if (detect_config.gaussian_blur_size <= 0)
    detect_config.gaussian_blur_size = 1;
  nh_.param<double>("gaussian_blur_sigma", detect_config.gaussian_blur_sigma, params_config_.gaussian_blur_sigma);
  nh_.param<double>("canny_edge_th", detect_config.canny_edge_th, params_config_.canny_edge_th);
  nh_.param<double>("hough_accum_resolution", detect_config.hough_accum_resolution,
                    params_config_.hough_accum_resolution);
  nh_.param<double>("min_circle_dist", detect_config.min_circle_dist, params_config_.min_circle_dist);
  nh_.param<double>("hough_accum_th", detect_config.hough_accum_th, params_config_.hough_accum_th);
  nh_.param<int>("min_radius", detect_config.min_radius, params_config_.min_radius);
  nh_.param<int>("max_radius", detect_config.max_radius, params_config_.max_radius);
  nh_.param<int>("filter_h_min", detect_config.filter_threshold.h_min, params_config_.filter_threshold.h_min);
  nh_.param<int>("filter_h_max", detect_config.filter_threshold.h_max, params_config_.filter_threshold.h_max);
  nh_.param<int>("filter_s_min", detect_config.filter_threshold.s_min, params_config_.filter_threshold.s_min);
  nh_.param<int>("filter_s_max", detect_config.filter_threshold.s_max, params_config_.filter_threshold.s_max);
  nh_.param<int>("filter_v_min", detect_config.filter_threshold.v_min, params_config_.filter_threshold.v_min);
  nh_.param<int>("filter_v_max", detect_config.filter_threshold.v_max, params_config_.filter_threshold.v_max);
  nh_.param<bool>("use_second_filter", detect_config.use_second_filter, params_config_.use_second_filter);
  nh_.param<int>("filter2_h_min", detect_config.filter2_threshold.h_min, params_config_.filter2_threshold.h_min);
  nh_.param<int>("filter2_h_max", detect_config.filter2_threshold.h_max, params_config_.filter2_threshold.h_max);
  nh_.param<int>("filter2_s_min", detect_config.filter2_threshold.s_min, params_config_.filter2_threshold.s_min);
  nh_.param<int>("filter2_s_max", detect_config.filter2_threshold.s_max, params_config_.filter2_threshold.s_max);
  nh_.param<int>("filter2_v_min", detect_config.filter2_threshold.v_min, params_config_.filter2_threshold.v_min);
  nh_.param<int>("filter2_v_max", detect_config.filter2_threshold.v_max, params_config_.filter2_threshold.v_max);
  nh_.param<int>("ellipse_size", detect_config.ellipse_size, params_config_.ellipse_size);
  nh_.param<bool>("filter_debug", detect_config.debug, params_config_.debug);

  //sets publishers
  image_pub_ = it_.advertise("image_out", 100);
  circles_pub_ = nh_.advertise<op3_ball_detector::CircleSetStamped>("circle_set", 100);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 100);

  //sets subscribers
  enable_sub_ = nh_.subscribe("enable", 1, &BallDetector::enableCallback, this);
  image_sub_ = it_.subscribe("image_in", 1, &BallDetector::imageCallback, this);
  camera_info_sub_ = nh_.subscribe("cameraInfo_in", 100, &BallDetector::cameraInfoCallback, this);

  //initializes newImageFlag
  new_image_flag_ = false;

  // dynamic_reconfigure
  callback_fnc_ = boost::bind(&BallDetector::dynParamCallback, this, _1, _2);
  param_server_.setCallback(callback_fnc_);

  // color configure
  color_config_path_ = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/red_ball_config.yaml";
  h_range_ = 140;
  s_range_ = 40;
  v_range_ = 100;

  for(int i = 0; i < NUM_INTERVALS - 1; i++)
  {
    light_weights_.push_back(1);
  }

  // web setting
  param_pub_ = nh_.advertise<op3_ball_detector::BallDetectorParams>("current_params", 1);
  param_command_sub_ = nh_.subscribe("param_command", 1, &BallDetector::paramCommandCallback, this);
  set_param_client_ = nh_.advertiseService("set_param", &BallDetector::setParamCallback, this);
  get_param_client_ = nh_.advertiseService("get_param", &BallDetector::getParamCallback, this);
  save_image_client_ = nh_.advertiseService("save_image", &BallDetector::saveImageCallback, this);
  switch_detection_client_ = nh_.advertiseService("switch_detection", &BallDetector::switchDetectionCallback, this);
  default_setting_path_ = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/ball_detector_params_default.yaml";

  //sets config and prints it
  params_config_ = detect_config;
  init_param_ = true;
  printConfig();
  process();
}

BallDetector::~BallDetector()
{

}

bool BallDetector::newImage()
{
  if (new_image_flag_)
  {
    new_image_flag_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

void BallDetector::process()
{
  int light_val = 0;
  if (enable_ == false)
    return;

  if (switch_detection_flag_ == true)
  {
    light_val = applyDetectionSettings();
    std::cout << "weights:  (";
    for(auto i = light_weights_.begin(); i != light_weights_.end(); i++)
    {
      if(i != light_weights_.begin())
        std::cout << " ,";
      std::cout << *i;
    }
    std::cout << ")" << std::endl;
  }

  if (cv_img_ptr_sub_ != NULL)
  {
    cv::Mat img_hsv, img_filtered;

    // set input image
    setInputImage(cv_img_ptr_sub_->image, img_hsv);

    // image filtering
    filterImage(img_hsv, img_filtered);

    //detect circles
    houghDetection2(img_filtered);
  
    if(switch_detection_flag_ == true){
      // was the detection valid?
      if (goodDetectionMode()) {
        std::cout << "Good detection mode for " << light_val << std::endl;
        params_color_.adjustWeightsWithLightVal(light_val, true, light_weights_);
      } else {
        params_color_.adjustWeightsWithLightVal(light_val, false, light_weights_);
      }
      params_color_.updateDistribution(light_range_, light_weights_);
    }
  }
}

void BallDetector::publishImage()
{
  if (enable_ == false)
    return;

  //image_raw topic
  cv_img_pub_.header.seq++;
  cv_img_pub_.header.stamp = sub_time_;
  cv_img_pub_.header.frame_id = image_frame_id_;
  switch (img_encoding_)
  {
  case IMG_RGB8:
    cv_img_pub_.encoding = sensor_msgs::image_encodings::RGB8;
    break;
  case IMG_MONO:
    cv_img_pub_.encoding = sensor_msgs::image_encodings::MONO8;
    break;
  default:
    cv_img_pub_.encoding = sensor_msgs::image_encodings::MONO8;
    break;
  }
  getOutputImage(cv_img_pub_.image);
  image_pub_.publish(cv_img_pub_.toImageMsg());
  camera_info_pub_.publish(camera_info_msg_);
}

void BallDetector::publishCircles()
{
  if (enable_ == false)
    return;

  if (circles_.size() == 0)
    return;

  //clears and resize the message
  circles_msg_.circles.clear();
  circles_msg_.circles.resize(circles_.size());

  //fill header
  circles_msg_.header.seq++;
  circles_msg_.header.stamp = sub_time_;
  circles_msg_.header.frame_id = "detector";

  // fill circle data
  // top(-1), bottom(+1)
  // left(-1), right(+1)
  for (int idx = 0; idx < circles_.size(); idx++)
  {
    circles_msg_.circles[idx].x = circles_[idx][0] / out_image_.cols * 2 - 1;    // x (-1 ~ 1)
    circles_msg_.circles[idx].y = circles_[idx][1] / out_image_.rows * 2 - 1;    // y (-1 ~ 1)
    circles_msg_.circles[idx].z = circles_[idx][2];    // radius
  }

  //publish message
  circles_pub_.publish(circles_msg_);
}

void BallDetector::enableCallback(const std_msgs::Bool::ConstPtr &msg)
{
  enable_ = msg->data;
}

void BallDetector::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
  if (enable_ == false)
    return;

  try
  {
    if (msg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
      this->img_encoding_ = IMG_MONO;
    if (msg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0)
      this->img_encoding_ = IMG_RGB8;
    this->cv_img_ptr_sub_ = cv_bridge::toCvCopy(msg, msg->encoding);
  } catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //indicates a new image is available
  this->sub_time_ = msg->header.stamp;
  this->image_frame_id_ = msg->header.frame_id;
  this->new_image_flag_ = true;
  return;
}

void BallDetector::dynParamCallback(op3_ball_detector::DetectorParamsConfig &config, uint32_t level)
{
  params_config_.gaussian_blur_size = config.gaussian_blur_size;
  params_config_.gaussian_blur_sigma = config.gaussian_blur_sigma;
  params_config_.canny_edge_th = config.canny_edge_th;
  params_config_.hough_accum_resolution = config.hough_accum_resolution;
  params_config_.min_circle_dist = config.min_circle_dist;
  params_config_.hough_accum_th = config.hough_accum_th;
  params_config_.min_radius = config.min_radius;
  params_config_.max_radius = config.max_radius;
  params_config_.filter_threshold.h_min = config.filter_h_min;
  params_config_.filter_threshold.h_max = config.filter_h_max;
  params_config_.filter_threshold.s_min = config.filter_s_min;
  params_config_.filter_threshold.s_max = config.filter_s_max;
  params_config_.filter_threshold.v_min = config.filter_v_min;
  params_config_.filter_threshold.v_max = config.filter_v_max;
  params_config_.use_second_filter = config.use_second_filter;
  params_config_.filter2_threshold.h_min = config.filter2_h_min;
  params_config_.filter2_threshold.h_max = config.filter2_h_max;
  params_config_.filter2_threshold.s_min = config.filter2_s_min;
  params_config_.filter2_threshold.s_max = config.filter2_s_max;
  params_config_.filter2_threshold.v_min = config.filter2_v_min;
  params_config_.filter2_threshold.v_max = config.filter2_v_max;
  params_config_.ellipse_size = config.ellipse_size;
  params_config_.debug = config.debug_image;

  // gaussian_blur has to be odd number.
  if (params_config_.gaussian_blur_size % 2 == 0)
    params_config_.gaussian_blur_size -= 1;
  if (params_config_.gaussian_blur_size <= 0)
    params_config_.gaussian_blur_size = 1;

  printConfig();
  saveConfig();
}

void BallDetector::cameraInfoCallback(const sensor_msgs::CameraInfo & msg)
{
  if (enable_ == false)
    return;

  camera_info_msg_ = msg;
}

void BallDetector::paramCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "debug")
  {
    params_config_.debug = true;
    saveConfig();
  }
  else if(msg->data == "normal")
  {
    params_config_.debug = false;
    saveConfig();
  }
  else if(msg->data == "reset")
  {
    // load default parameters and apply
    resetParameter();
  }
}

bool BallDetector::setParamCallback(op3_ball_detector::SetParameters::Request &req, op3_ball_detector::SetParameters::Response &res)
{
  params_config_.gaussian_blur_size = req.params.gaussian_blur_size;
  params_config_.gaussian_blur_sigma = req.params.gaussian_blur_sigma;
  params_config_.canny_edge_th = req.params.canny_edge_th;
  params_config_.hough_accum_resolution = req.params.hough_accum_resolution;
  params_config_.min_circle_dist = req.params.min_circle_dist;
  params_config_.hough_accum_th = req.params.hough_accum_th;
  params_config_.min_radius = req.params.min_radius;
  params_config_.max_radius = req.params.max_radius;
  params_config_.filter_threshold.h_min = req.params.filter_h_min;
  params_config_.filter_threshold.h_max = req.params.filter_h_max;
  params_config_.filter_threshold.s_min = req.params.filter_s_min;
  params_config_.filter_threshold.s_max = req.params.filter_s_max;
  params_config_.filter_threshold.v_min = req.params.filter_v_min;
  params_config_.filter_threshold.v_max = req.params.filter_v_max;
  params_config_.ellipse_size = req.params.ellipse_size;

  saveConfig();

  res.returns = req.params;

  return true;
}

bool BallDetector:: getParamCallback(op3_ball_detector::GetParameters::Request &req, op3_ball_detector::GetParameters::Response &res)
{
  res.returns.gaussian_blur_size = params_config_.gaussian_blur_size;
  res.returns.gaussian_blur_sigma = params_config_.gaussian_blur_sigma;
  res.returns.canny_edge_th = params_config_.canny_edge_th;
  res.returns.hough_accum_resolution = params_config_.hough_accum_resolution;
  res.returns.min_circle_dist = params_config_.min_circle_dist;
  res.returns.hough_accum_th = params_config_.hough_accum_th;
  res.returns.min_radius = params_config_.min_radius;
  res.returns.max_radius = params_config_.max_radius;
  res.returns.filter_h_min = params_config_.filter_threshold.h_min;
  res.returns.filter_h_max = params_config_.filter_threshold.h_max;
  res.returns.filter_s_min = params_config_.filter_threshold.s_min;
  res.returns.filter_s_max = params_config_.filter_threshold.s_max;
  res.returns.filter_v_min = params_config_.filter_threshold.v_min;
  res.returns.filter_v_max = params_config_.filter_threshold.v_max;
  res.returns.ellipse_size = params_config_.ellipse_size;

  return true;
}

// Callback for image saving
bool BallDetector::saveImageCallback(op3_ball_detector::SaveImage::Request &req, op3_ball_detector::SaveImage::Response &res)
{
  std::string filename = "/home/robotis/" + req.params.name + ".png";
  if (cv_img_ptr_sub_ != NULL) {
    cv::Mat out_image;
    cv::cvtColor(cv_img_ptr_sub_->image, out_image, cv::COLOR_RGB2BGR);
    if (cv::imwrite(filename, out_image) == false) {
      res.returns.name = "Failed for some reason";
    }
    else {
      res.returns.name = "Saved";
    }
  }
  else {
    res.returns.name = "Not Saved";
  }

  return true;
}

bool BallDetector::switchDetectionCallback(op3_ball_detector::SwitchDetection::Request &req, op3_ball_detector::SwitchDetection::Response &res)
{
  // Not set, turn on improved detection
  if (switch_detection_flag_ == false)
  {
    switch_detection_flag_ = true;
    loadDetectionSettings();
    res.returns = "Improved detection loaded.";
  }
  else // otherwise, turn it off
  {
    switch_detection_flag_ = false;
    // define unloading
    res.returns = "Improved detection unloaded.";
  }
  return true;
}

bool BallDetector::loadDetectionSettings()
{
  std::cout << "Loading detection settings..." << std::endl << std::endl;
  try
  {
    YAML::Node config = YAML::LoadFile(color_config_path_.c_str());

    params_color_.x_min = config["x_min"].as<int>();
    params_color_.x_max = config["x_max"].as<int>();
    params_color_.light_slope = config["light_slope"].as<double>();
    params_color_.light_constant = config["light_constant"].as<double>();
    has_color_config_ = true;

    params_color_.range = params_color_.x_max - params_color_.x_min;
    double subrange = params_color_.range / NUM_INTERVALS;
    light_range_.clear();
    for(double i = params_color_.x_min; i < params_color_.x_max; i+=subrange)
    {
      light_range_.push_back(i);
    }

    params_color_.updateDistribution(light_range_, light_weights_);

  }
   catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Failed to Get detection settings : " << color_config_path_);
    return false;
  }

  return true;
}

void BallDetector::testDistributionPercent(int light_val, int range, int x_min, int x_max)
{
  last_vals_.push_back(int((light_val - params_color_.x_min) / (params_color_.range / NUM_INTERVALS)));
  ++counter_[int((light_val - params_color_.x_min) / (params_color_.range / NUM_INTERVALS))];
  if(last_vals_.size() > range)
  {
    int oldest = last_vals_.front();
    last_vals_.pop_front();
    --counter_[oldest];
  }
  double percent_in_range = 0;
  double total = 0;
  double sum_in_range = 0;
  for(std::map<int, int>::iterator i = counter_.begin(); i != counter_.end(); i++)
  {
    total += i->second;
  }
  int lower = int((x_min - params_color_.x_min) / (params_color_.range / NUM_INTERVALS));
  int upper = int((x_max - params_color_.x_min) / (params_color_.range / NUM_INTERVALS));
  double subrange = params_color_.range / NUM_INTERVALS;
  for(int i = lower; i <= upper; i++)
  {
    sum_in_range += counter_[i];
  }
  if(sum_in_range > 0)
    percent_in_range = (sum_in_range / total) * 100;
  else
    percent_in_range = 0.0f;

  std::cout << "IN RANGE:   " << percent_in_range << "%" << std::endl;
}

int BallDetector::applyDetectionSettings()
{
  if(!has_color_config_)
    return -1;
  int h, s, v, g, b;


  int light_val = params_color_.sampleLightVal();
  int R = params_color_.getMedianRVal(light_val);
  g = 0;
  b = 0;

  double avgH = (params_config_.filter_threshold.h_min - params_config_.filter_threshold.h_max ) / 2;
  double avgS = (params_config_.filter_threshold.s_max - params_config_.filter_threshold.s_min ) / 2;
  double avgV = (params_config_.filter_threshold.v_max - params_config_.filter_threshold.v_min ) / 2;

  convertRGBtoHSV(R, g, b, h, s, v);

  std::cout << "Updating HSV to (" << h << ", " << s << ", " << v << ")" << std::endl;

  updateHSV(h, s, v);
  publishParam();
  return light_val;
}

void BallDetector::updateHSV(int h, int s, int v)
{
  params_config_.filter_threshold.h_min = h + h_range_ / 2;  // min is the larger number for some reason
  params_config_.filter_threshold.h_max = h - h_range_ / 2;
  params_config_.filter_threshold.s_max = s + s_range_ / 2;
  params_config_.filter_threshold.s_min = s - s_range_ / 2;
  params_config_.filter_threshold.v_max = v + v_range_ / 2;
  params_config_.filter_threshold.v_min = v - v_range_ / 2;

  params_config_.filter_threshold.h_min = std::min(params_config_.filter_threshold.h_min, 360);
  params_config_.filter_threshold.h_max = std::max(params_config_.filter_threshold.h_max, 0);
  params_config_.filter_threshold.s_min = std::max(params_config_.filter_threshold.s_min, 0);
  params_config_.filter_threshold.s_max = std::min(params_config_.filter_threshold.s_max, 255);
  params_config_.filter_threshold.v_min = std::max(params_config_.filter_threshold.v_min, 0);
  params_config_.filter_threshold.v_max = std::min(params_config_.filter_threshold.v_max, 255);

}

void BallDetector::convertHSVtoRGB(double h, double s, double v, int &rOut, int &gOut, int &bOut)
{
  double scaledS = s / 255;
  double scaledV = v / 255;

  double C = scaledV * scaledS;
  double X = C * (1 - std::abs(fmod((h / 60) , 2) - 1));
  double m = scaledV - C;

  double r, g, b;

  if(h < 60){
    r = C;
    g = X;
    b = 0;
  } else if(h < 120){
    r = X;
    g = C;
    b = 0;
  } else if(h < 180){
    r = 0;
    g = C;
    b = X;
  } else if(h < 240){
    r = 0;
    g = X;
    b = C;
  } else if(h < 300){
    r = X;
    g = 0;
    b = C;
  } else {
    r = C;
    g = 0;
    b = X;
  }

  rOut = (r + m) * 255;
  gOut = (g + m) * 255;
  bOut = (b + m) * 255;
}

void BallDetector::convertRGBtoHSV(int r, int g, int b, int &hOut, int &sOut, int &vOut)
{
  double rScaled = (double)r / 255;
  double gScaled = (double)g / 255;
  double bScaled = (double)b / 255;

  double Cmax = std::max(std::max(rScaled, gScaled), bScaled);
  double Cmin = std::min(std::min(rScaled, gScaled), bScaled);
  double delta = Cmax - Cmin;

  double val = ((rScaled - gScaled) / delta) + 4.0;

  if(delta == 0){
    hOut = 0;
  } else if(Cmax == rScaled){
    hOut = 60 * (fmod(((gScaled - bScaled) / delta) , 6));
  } else if(Cmax == gScaled){
    hOut = 60 * (((bScaled - rScaled) / delta) + 2);
  } else if(Cmax == bScaled){
    hOut = 60 * (((rScaled - gScaled) / delta) + 4);
  }

  sOut = Cmax ? (delta / Cmax) * 255 : 0;

  vOut = Cmax * 255;
}

bool BallDetector::goodDetectionMode()
{
  if (circles_.size() > 1) 
    return false; // If there is more than one circle, this can't be a good detection mode
  else if (circles_.empty())    
    return false; // If there are no circles after detection, it's not 
  else 
    return true;  // Otherwise, exactly one circle indicates probably a pretty good detection mode 
}

void BallDetector::resetParameter()
{

  YAML::Node doc;

  try
  {
    // load yaml
    doc = YAML::LoadFile(default_setting_path_.c_str());

    // parse
    params_config_.gaussian_blur_size = doc["gaussian_blur_size"].as<int>();
    params_config_.gaussian_blur_sigma = doc["gaussian_blur_sigma"].as<double>();
    params_config_.canny_edge_th = doc["canny_edge_th"].as<double>();
    params_config_.hough_accum_resolution = doc["hough_accum_resolution"].as<double>();
    params_config_.min_circle_dist = doc["min_circle_dist"].as<double>();
    params_config_.hough_accum_th = doc["hough_accum_th"].as<double>();
    params_config_.min_radius = doc["min_radius"].as<int>();
    params_config_.max_radius = doc["max_radius"].as<int>();
    params_config_.filter_threshold.h_min = doc["filter_h_min"].as<int>();
    params_config_.filter_threshold.h_max = doc["filter_h_max"].as<int>();
    params_config_.filter_threshold.s_min = doc["filter_s_min"].as<int>();
    params_config_.filter_threshold.s_max = doc["filter_s_max"].as<int>();
    params_config_.filter_threshold.v_min = doc["filter_v_min"].as<int>();
    params_config_.filter_threshold.v_max = doc["filter_v_max"].as<int>();
    params_config_.use_second_filter = doc["use_second_filter"].as<bool>();
    params_config_.filter2_threshold.h_min = doc["filter2_h_min"].as<int>();
    params_config_.filter2_threshold.h_max = doc["filter2_h_max"].as<int>();
    params_config_.filter2_threshold.s_min = doc["filter2_s_min"].as<int>();
    params_config_.filter2_threshold.s_max = doc["filter2_s_max"].as<int>();
    params_config_.filter2_threshold.v_min = doc["filter2_v_min"].as<int>();
    params_config_.filter2_threshold.v_max = doc["filter2_v_max"].as<int>();
    params_config_.ellipse_size = doc["ellipse_size"].as<int>();
    params_config_.debug = doc["filter_debug"].as<bool>();

    // gaussian_blur has to be odd number.
    if (params_config_.gaussian_blur_size % 2 == 0)
      params_config_.gaussian_blur_size -= 1;
    if (params_config_.gaussian_blur_size <= 0)
      params_config_.gaussian_blur_size = 1;

    printConfig();
    saveConfig();

    publishParam();
  } catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Failed to Get default parameters : " << default_setting_path_);
    return;
  }
}

void BallDetector::publishParam()
{
  op3_ball_detector::BallDetectorParams params;

  params.gaussian_blur_size = params_config_.gaussian_blur_size;
  params.gaussian_blur_sigma = params_config_.gaussian_blur_sigma;
  params.canny_edge_th = params_config_.canny_edge_th;
  params.hough_accum_resolution = params_config_.hough_accum_resolution;
  params.min_circle_dist = params_config_.min_circle_dist;
  params.hough_accum_th = params_config_.hough_accum_th;
  params.min_radius = params_config_.min_radius;
  params.max_radius = params_config_.max_radius;
  params.filter_h_min = params_config_.filter_threshold.h_min;
  params.filter_h_max = params_config_.filter_threshold.h_max;
  params.filter_s_min = params_config_.filter_threshold.s_min;
  params.filter_s_max = params_config_.filter_threshold.s_max;
  params.filter_v_min = params_config_.filter_threshold.v_min;
  params.filter_v_max = params_config_.filter_threshold.v_max;
  params.ellipse_size = params_config_.ellipse_size;

  param_pub_.publish(params);
}

void BallDetector::printConfig()
{
  if (init_param_ == false)
    return;

  std::cout << "Detetctor Configuration:" << std::endl << "    gaussian_blur_size: "
            << params_config_.gaussian_blur_size << std::endl << "    gaussian_blur_sigma: "
            << params_config_.gaussian_blur_sigma << std::endl << "    canny_edge_th: " << params_config_.canny_edge_th
            << std::endl << "    hough_accum_resolution: " << params_config_.hough_accum_resolution << std::endl
            << "    min_circle_dist: " << params_config_.min_circle_dist << std::endl << "    hough_accum_th: "
            << params_config_.hough_accum_th << std::endl << "    min_radius: " << params_config_.min_radius
            << std::endl << "    max_radius: " << params_config_.max_radius << std::endl << "    filter_h_min: "
            << params_config_.filter_threshold.h_min << std::endl << "    filter_h_max: "
            << params_config_.filter_threshold.h_max << std::endl << "    filter_s_min: "
            << params_config_.filter_threshold.s_min << std::endl << "    filter_s_max: "
            << params_config_.filter_threshold.s_max << std::endl << "    filter_v_min: "
            << params_config_.filter_threshold.v_min << std::endl << "    filter_v_max: "
            << params_config_.filter_threshold.v_max << std::endl << "    use_second_filter: "
            << params_config_.use_second_filter << std::endl << "    filter2_h_min: "
            << params_config_.filter2_threshold.h_min << std::endl << "    filter2_h_max: "
            << params_config_.filter2_threshold.h_max << std::endl << "    filter2_s_min: "
            << params_config_.filter2_threshold.s_min << std::endl << "    filter2_s_max: "
            << params_config_.filter2_threshold.s_max << std::endl << "    filter2_v_min: "
            << params_config_.filter2_threshold.v_min << std::endl << "    filter2_v_max: "
            << params_config_.filter2_threshold.v_max << std::endl << "    ellipse_size: "
            << params_config_.ellipse_size << std::endl << "    filter_image_to_debug: " << params_config_.debug
            << std::endl << std::endl;

  std::cout << "Ball color Configuration:" << std::endl << "    x_min: "
            << params_color_.x_min << std::endl << "    x_max: "
            << params_color_.x_max << std::endl << "    light_slope: "
            << params_color_.light_slope << std::endl << "    light_constant: "
            << params_color_.light_constant << std::endl << std::endl;
}

void BallDetector::saveConfig()
{
  if (has_path_ == false)
    return;

  YAML::Emitter yaml_out;

  yaml_out << YAML::BeginMap;
  yaml_out << YAML::Key << "gaussian_blur_size" << YAML::Value << params_config_.gaussian_blur_size;
  yaml_out << YAML::Key << "gaussian_blur_sigma" << YAML::Value << params_config_.gaussian_blur_sigma;
  yaml_out << YAML::Key << "canny_edge_th" << YAML::Value << params_config_.canny_edge_th;
  yaml_out << YAML::Key << "hough_accum_resolution" << YAML::Value << params_config_.hough_accum_resolution;
  yaml_out << YAML::Key << "min_circle_dist" << YAML::Value << params_config_.min_circle_dist;
  yaml_out << YAML::Key << "hough_accum_th" << YAML::Value << params_config_.hough_accum_th;
  yaml_out << YAML::Key << "min_radius" << YAML::Value << params_config_.min_radius;
  yaml_out << YAML::Key << "max_radius" << YAML::Value << params_config_.max_radius;
  yaml_out << YAML::Key << "filter_h_min" << YAML::Value << params_config_.filter_threshold.h_min;
  yaml_out << YAML::Key << "filter_h_max" << YAML::Value << params_config_.filter_threshold.h_max;
  yaml_out << YAML::Key << "filter_s_min" << YAML::Value << params_config_.filter_threshold.s_min;
  yaml_out << YAML::Key << "filter_s_max" << YAML::Value << params_config_.filter_threshold.s_max;
  yaml_out << YAML::Key << "filter_v_min" << YAML::Value << params_config_.filter_threshold.v_min;
  yaml_out << YAML::Key << "filter_v_max" << YAML::Value << params_config_.filter_threshold.v_max;
  yaml_out << YAML::Key << "use_second_filter" << YAML::Value << params_config_.use_second_filter;
  yaml_out << YAML::Key << "filter2_h_min" << YAML::Value << params_config_.filter2_threshold.h_min;
  yaml_out << YAML::Key << "filter2_h_max" << YAML::Value << params_config_.filter2_threshold.h_max;
  yaml_out << YAML::Key << "filter2_s_min" << YAML::Value << params_config_.filter2_threshold.s_min;
  yaml_out << YAML::Key << "filter2_s_max" << YAML::Value << params_config_.filter2_threshold.s_max;
  yaml_out << YAML::Key << "filter2_v_min" << YAML::Value << params_config_.filter2_threshold.v_min;
  yaml_out << YAML::Key << "filter2_v_max" << YAML::Value << params_config_.filter2_threshold.v_max;
  yaml_out << YAML::Key << "ellipse_size" << YAML::Value << params_config_.ellipse_size;
  yaml_out << YAML::Key << "filter_debug" << YAML::Value << params_config_.debug;
  yaml_out << YAML::EndMap;

  // output to file
  std::ofstream fout(param_path_.c_str());
  fout << yaml_out.c_str();
}

void BallDetector::setInputImage(const cv::Mat & inIm)
{
  in_image_ = inIm.clone();

  if (params_config_.debug == false)
    out_image_ = in_image_.clone();
}

void BallDetector::setInputImage(const cv::Mat & inIm, cv::Mat &in_filter_img)
{
  cv::cvtColor(inIm, in_filter_img, cv::COLOR_RGB2HSV);

  if (params_config_.debug == false)
    out_image_ = inIm.clone();
}

void BallDetector::getOutputImage(cv::Mat & outIm)
{
  this->drawOutputImage();
  outIm = out_image_.clone();
}

void BallDetector::filterImage()
{
  if (!in_image_.data)
    return;

  cv::Mat img_hsv, img_filtered;
  cv::cvtColor(in_image_, img_hsv, cv::COLOR_RGB2HSV);

  inRangeHsv(img_hsv, params_config_.filter_threshold, img_filtered);

  // mophology : open and close
  mophology(img_filtered, img_filtered, params_config_.ellipse_size);

  if (params_config_.use_second_filter == true)
  {
    // mask
    cv::Mat img_mask;

    // check hsv range
    cv::Mat img_filtered2;
    inRangeHsv(img_hsv, params_config_.filter2_threshold, img_filtered2);

    makeFilterMaskFromBall(img_filtered, img_mask);
    cv::bitwise_and(img_filtered2, img_mask, img_filtered2);

    // or
    cv::bitwise_or(img_filtered, img_filtered2, img_filtered);
  }

  mophology(img_filtered, img_filtered, params_config_.ellipse_size);

  cv::cvtColor(img_filtered, in_image_, cv::COLOR_GRAY2RGB);
}

void BallDetector::filterImage(const cv::Mat &in_filter_img, cv::Mat &out_filter_img)
{
  if (!in_filter_img.data)
    return;

  inRangeHsv(in_filter_img, params_config_.filter_threshold, out_filter_img);

  // mophology : open and close
  mophology(out_filter_img, out_filter_img, params_config_.ellipse_size);

  if (params_config_.use_second_filter == true)
  {
    // mask
    cv::Mat img_mask;

    // check hsv range
    cv::Mat img_filtered2;
    inRangeHsv(in_filter_img, params_config_.filter2_threshold, img_filtered2);

    makeFilterMaskFromBall(out_filter_img, img_mask);
    cv::bitwise_and(img_filtered2, img_mask, img_filtered2);

    // or
    cv::bitwise_or(out_filter_img, img_filtered2, out_filter_img);
  }

  mophology(out_filter_img, out_filter_img, params_config_.ellipse_size);

  //draws results to output Image
  if (params_config_.debug == true)
    cv::cvtColor(out_filter_img, out_image_, cv::COLOR_GRAY2RGB);
}

void BallDetector::makeFilterMask(const cv::Mat &source_img, cv::Mat &mask_img, int range)
{
  // source_img.
  mask_img = cv::Mat::zeros(source_img.rows, source_img.cols, CV_8UC1);

  int source_height = source_img.rows;
  int source_width = source_img.cols;

  // channel : 1
  if (source_img.channels() != 1)
    return;

  for (int i = 0; i < source_height; i++)
  {
    for (int j = 0; j < source_width; j++)
    {
      uint8_t pixel = source_img.at < uint8_t > (i, j);

      if (pixel == 0)
        continue;

      for (int mask_i = i - range; mask_i <= i + range; mask_i++)
      {
        if (mask_i < 0 || mask_i >= source_height)
          continue;

        for (int mask_j = j - range; mask_j <= j + range; mask_j++)
        {
          if (mask_j < 0 || mask_j >= source_width)
            continue;

          mask_img.at < uchar > (mask_i, mask_j, 0) = 255;
        }
      }
    }
  }
}

void BallDetector::makeFilterMaskFromBall(const cv::Mat &source_img, cv::Mat &mask_img)
{
  // source_img.
  mask_img = cv::Mat::zeros(source_img.rows, source_img.cols, CV_8UC1);

  if (circles_.size() == 0)
    return;

  // channel : 1
  if (source_img.channels() != 1)
    return;

  cv::Mat img_labels, stats, centroids;
  int numOfLables = cv::connectedComponentsWithStats(source_img, img_labels, stats, centroids, 8, CV_32S);
  for (int j = 1; j < numOfLables; j++)
  {
    int area = stats.at<int>(j, cv::CC_STAT_AREA);
    int left = stats.at<int>(j, cv::CC_STAT_LEFT);
    int top = stats.at<int>(j, cv::CC_STAT_TOP);
    int width = stats.at<int>(j, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(j, cv::CC_STAT_HEIGHT);

    int center_x = left + width * 0.5;
    int center_y = top + height * 0.5;
    int radius = (width + height) * 0.5;

    for (int mask_i = center_y - radius; mask_i <= center_y + radius; mask_i++)
    {
      if (mask_i < 0 || mask_i >= source_img.rows)
        continue;

      int mask_offset = abs(mask_i - center_y) * 0.5;

      for (int mask_j = center_x - radius + mask_offset; mask_j <= center_x + radius - mask_offset; mask_j++)
      {
        if (mask_j < 0 || mask_j >= source_img.cols)
          continue;

        mask_img.at < uchar > (mask_i, mask_j, 0) = 255;
      }
    }
  }

}

void BallDetector::inRangeHsv(const cv::Mat &input_img, const HsvFilter &filter_value, cv::Mat &output_img)
{
  // 0-360 -> 0-180
  int scaled_hue_min = static_cast<int>(filter_value.h_min * 0.5);
  int scaled_hue_max = static_cast<int>(filter_value.h_max * 0.5);

  if (scaled_hue_min <= scaled_hue_max)
  {
    cv::Scalar min_value = cv::Scalar(scaled_hue_min, filter_value.s_min, filter_value.v_min, 0);
    cv::Scalar max_value = cv::Scalar(scaled_hue_max, filter_value.s_max, filter_value.v_max, 0);
    cv::inRange(input_img, min_value, max_value, output_img);
  }
  else
  {
    cv::Mat lower_hue_range, upper_hue_range;
    cv::Scalar min_value, max_value;

    min_value = cv::Scalar(0, filter_value.s_min, filter_value.v_min, 0);
    max_value = cv::Scalar(scaled_hue_max, filter_value.s_max, filter_value.v_max, 0);
    cv::inRange(input_img, min_value, max_value, lower_hue_range);

    min_value = cv::Scalar(scaled_hue_min, filter_value.s_min, filter_value.v_min, 0);
    max_value = cv::Scalar(179, filter_value.s_max, filter_value.v_max, 0);
    cv::inRange(input_img, min_value, max_value, upper_hue_range);

    cv::bitwise_or(lower_hue_range, upper_hue_range, output_img);
  }
}

void BallDetector::mophology(const cv::Mat &intput_img, cv::Mat &output_img, int ellipse_size)
{
  cv::erode(intput_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)));
  cv::dilate(output_img, output_img,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size * 2, ellipse_size * 2)));

  cv::dilate(output_img, output_img,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)));
  cv::erode(output_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)));
}

void BallDetector::houghDetection(const unsigned int imgEncoding)
{
  cv::Mat gray_image;
  std::vector<cv::Vec3f> circles_current;
  std::vector<cv::Vec3f> prev_circles = circles_;

  //clear previous circles
  circles_.clear();

  // If input image is RGB, convert it to gray
  if (imgEncoding == IMG_RGB8)
    cv::cvtColor(in_image_, gray_image, CV_RGB2GRAY);

  //Reduce the noise so we avoid false circle detection
  cv::GaussianBlur(gray_image, gray_image,
                   cv::Size(params_config_.gaussian_blur_size, params_config_.gaussian_blur_size),
                   params_config_.gaussian_blur_sigma);

  double hough_accum_th = params_config_.hough_accum_th;


  //Apply the Hough Transform to find the circles
  cv::HoughCircles(gray_image, circles_current, CV_HOUGH_GRADIENT, params_config_.hough_accum_resolution,
                   params_config_.min_circle_dist, params_config_.canny_edge_th, hough_accum_th,
                   params_config_.min_radius, params_config_.max_radius);

  if (circles_current.size() == 0)
    not_found_count_ += 1;
  else
    not_found_count_ = 0;

  double alpha = 0.2;

  for (int ix = 0; ix < circles_current.size(); ix++)
  {
    cv::Point2d center = cv::Point(circles_current[ix][0], circles_current[ix][1]);
    double radius = circles_current[ix][2];

    for (int prev_ix = 0; prev_ix < prev_circles.size(); prev_ix++)
    {
      cv::Point2d prev_center = cv::Point(prev_circles[prev_ix][0], prev_circles[prev_ix][1]);
      double prev_radius = prev_circles[prev_ix][2];

      cv::Point2d diff = center - prev_center;
      double radius_th = std::max(radius, prev_radius) * 0.75;

      if (sqrt(diff.dot(diff)) < radius_th)
      {
        if (abs(radius - prev_radius) < radius_th)
        {
          circles_current[ix] = circles_current[ix] * alpha + prev_circles[prev_ix] * (1 - alpha);
        }

        break;
      }
    }

    circles_.push_back(circles_current[ix]);
  }
}

void BallDetector::houghDetection2(const cv::Mat &input_hough)
{
  std::vector<cv::Vec3f> circles_current;
  std::vector<cv::Vec3f> prev_circles = circles_;

  //clear previous circles
  circles_.clear();

  //Reduce the noise so we avoid false circle detection
  cv::GaussianBlur(input_hough, input_hough,
                   cv::Size(params_config_.gaussian_blur_size, params_config_.gaussian_blur_size),
                   params_config_.gaussian_blur_sigma);

  double hough_accum_th = params_config_.hough_accum_th;


  //Apply the Hough Transform to find the circles
  cv::HoughCircles(input_hough, circles_current, CV_HOUGH_GRADIENT, params_config_.hough_accum_resolution,
                   params_config_.min_circle_dist, params_config_.canny_edge_th, hough_accum_th,
                   params_config_.min_radius, params_config_.max_radius);

  if (circles_current.size() == 0)
    not_found_count_ += 1;
  else
    not_found_count_ = 0;

  double alpha = 0.2;

  for (int ix = 0; ix < circles_current.size(); ix++)
  {
    cv::Point2d center = cv::Point(circles_current[ix][0], circles_current[ix][1]);
    double radius = circles_current[ix][2];

    for (int prev_ix = 0; prev_ix < prev_circles.size(); prev_ix++)
    {
      cv::Point2d prev_center = cv::Point(prev_circles[prev_ix][0], prev_circles[prev_ix][1]);
      double prev_radius = prev_circles[prev_ix][2];

      cv::Point2d diff = center - prev_center;
      double radius_th = std::max(radius, prev_radius) * 0.75;

      if (sqrt(diff.dot(diff)) < radius_th)
      {
        if (abs(radius - prev_radius) < radius_th)
        {
          circles_current[ix] = circles_current[ix] * alpha + prev_circles[prev_ix] * (1 - alpha);
        }

        break;
      }
    }

    circles_.push_back(circles_current[ix]);
  }
}

void BallDetector::drawOutputImage()
{
  cv::Point center_position;
  int radius = 0;
  size_t ii;

  for (ii = 0; ii < circles_.size(); ii++)
  {
    {
      int this_radius = cvRound(circles_[ii][2]);
      if (this_radius > radius)
      {
        radius = this_radius;
        center_position = cv::Point(cvRound(circles_[ii][0]), cvRound(circles_[ii][1]));
      }
    }
  }
  cv::circle(out_image_, center_position, 5, cv::Scalar(0, 0, 255), -1, 8, 0);      // circle center in blue
  cv::circle(out_image_, center_position, radius, cv::Scalar(0, 0, 255), 3, 8, 0);      // circle outline in blue

}

}       // namespace robotis_op
