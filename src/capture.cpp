// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture.h"
#include <sstream>
#include <string>

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

Capture::Capture(ros::NodeHandle &node, const std::string &topic_name,
                 int32_t buffer_size, const std::string &frame_id)
    : node_(node),
      left_it_(ros::NodeHandle(node, "left_it")),
      right_it_(ros::NodeHandle(node, "right_it")),
      topic_name_(topic_name),
      buffer_size_(buffer_size),
      frame_id_(frame_id),
      left_info_manager_(ros::NodeHandle(node, "left"), "left_camera"),
      right_info_manager_(ros::NodeHandle(node, "right"), "right_camera")
{
}

void Capture::parseCustomSettings()
{
  for (int i = 0;; ++i)
  {
    int code = 0;
    double value = 0.0;
    std::stringstream stream;
    stream << "property_" << i << "_code";
    const std::string param_for_code = stream.str();
    stream.str("");
    stream << "property_" << i << "_value";
    const std::string param_for_value = stream.str();
    if (!node_.getParam(param_for_code, code) || !node_.getParam(param_for_value, value))
    {
      break;
    }
    if (!cap_.set(code, value))
    {
      ROS_ERROR_STREAM("Setting with code " << code << " and value " << value << " failed"
                                            << std::endl);
    }
  }
}

void Capture::advertise()
{
  left_pub_ = left_it_.advertiseCamera("left/" + topic_name_, buffer_size_);
  right_pub_ = right_it_.advertiseCamera("right/" + topic_name_, buffer_size_);
}

void Capture::loadCameraInfo()
{
  std::string url;
  if (node_.getParam("left/camera_info_url", url))
  {
    if (left_info_manager_.validateURL(url))
    {
      left_info_manager_.loadCameraInfo(url);
    }
  }

  if (node_.getParam("right/camera_info_url", url))
  {
    if (right_info_manager_.validateURL(url))
    {
      right_info_manager_.loadCameraInfo(url);
    }
  }

  rescale_camera_info_ = node_.param<bool>("rescale_camera_info", false);
}

void Capture::rescaleCameraInfo(int width, int height, sensor_msgs::CameraInfo &info)
{
  double width_coeff = static_cast<double>(width) / info.width;
  double height_coeff = static_cast<double>(height) / info.height;
  info.width = width;
  info.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info.K[0] *= width_coeff;
  info.K[2] *= width_coeff;
  info.K[4] *= height_coeff;
  info.K[5] *= height_coeff;

  info.P[0] *= width_coeff;
  info.P[2] *= width_coeff;
  info.P[5] *= height_coeff;
  info.P[6] *= height_coeff;
}

void Capture::open(int32_t device_id)
{
  cap_.open(device_id);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "device_id" << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }

  advertise();
  loadCameraInfo();
}

void Capture::open(const std::string &device_path)
{
  cap_.open(device_path, cv::CAP_V4L);
  if (!cap_.isOpened())
  {
    throw DeviceError("device_path " + device_path + " cannot be opened");
  }

  advertise();
  loadCameraInfo();
}

void Capture::open()
{
  open(0);
}

void Capture::openFile(const std::string &file_path)
{
  cap_.open(file_path);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "file " << file_path << " cannot be opened";
    throw DeviceError(stream.str());
  }

  advertise();
  loadCameraInfo();
}

bool Capture::capture()
{
  if (cap_.read(base_frame_))
  {
    ros::Time now = ros::Time::now();
    left_bridge_.encoding         = enc::BGR8;
    left_bridge_.header.stamp     = now;
    left_bridge_.header.frame_id  = frame_id_;
    left_bridge_.image            = base_frame_( cv::Rect(0, 
                                                          base_frame_.rows, 
                                                          base_frame_.cols/2, 
                                                          base_frame_.rows) );

    right_bridge_.encoding        = enc::BGR8;
    right_bridge_.header.stamp    = now;
    right_bridge_.header.frame_id = frame_id_;
    right_bridge_.image           = base_frame_( cv::Rect(base_frame_.cols/2, 
                                                          base_frame_.rows, 
                                                          base_frame_.cols/2, 
                                                          base_frame_.rows) );

    /* Left resize */
    left_info_ = left_info_manager_.getCameraInfo();
    if (left_info_.height == 0 && left_info_.width == 0)
    {
      left_info_.height = left_bridge_.image.rows;
      left_info_.width = left_bridge_.image.cols;
    }
    else if (left_info_.height != left_bridge_.image.rows || left_info_.width != left_bridge_.image.cols)
    {
      if (rescale_camera_info_)
      {
        int old_width = left_info_.width;
        int old_height = left_info_.height;

        rescaleCameraInfo(left_bridge_.image.cols, left_bridge_.image.rows, left_info_);

        ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                      old_width, old_height, left_bridge_.image.cols, left_bridge_.image.rows);
      }
      else
      {
        ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      left_info_.width, left_info_.height, left_bridge_.image.cols, left_bridge_.image.rows);
      }
    }

    /* Right resize */
    right_info_ = right_info_manager_.getCameraInfo();
    if (right_info_.height == 0 && right_info_.width == 0)
    {
      right_info_.height = right_bridge_.image.rows;
      right_info_.width = right_bridge_.image.cols;
    }
    else if (right_info_.height != right_bridge_.image.rows || right_info_.width != right_bridge_.image.cols)
    {
      if (rescale_camera_info_)
      {
        int old_width = right_info_.width;
        int old_height = right_info_.height;

        rescaleCameraInfo(right_bridge_.image.cols, right_bridge_.image.rows, right_info_);

        ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                      old_width, old_height, right_bridge_.image.cols, right_bridge_.image.rows);
      }
      else
      {
        ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      right_info_.width, right_info_.height, right_bridge_.image.cols, right_bridge_.image.rows);
      }
    }

    /* Info fill */
    left_info_.header.stamp = now;
    left_info_.header.frame_id = frame_id_;
    
    right_info_.header.stamp = now;
    right_info_.header.frame_id = frame_id_;

    return true;
  }
  return false;
}

void Capture::publish()
{
  left_pub_.publish(*getImageMsgPtr(left_bridge_), left_info_);
  right_pub_.publish(*getImageMsgPtr(right_bridge_), right_info_);
}

bool Capture::setPropertyFromParam(int property_id, const std::string &param_name)
{
  if (cap_.isOpened())
  {
    double value = 0.0;
    if (node_.getParam(param_name, value))
    {
      ROS_INFO("setting property %s = %lf", param_name.c_str(), value);
      return cap_.set(property_id, value);
    }
  }
  return true;
}

} // namespace cv_camera
