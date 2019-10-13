// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_CAPTURE_H
#define CV_CAMERA_CAPTURE_H

#include "cv_camera/exception.h"
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.h>

/**
 * @brief namespace of this package
 */
namespace cv_camera
{

/**
 * @brief captures by cv::VideoCapture and publishes to ROS topic.
 *
 */
class Capture
{
public:
  /**
   * @brief costruct with ros node and topic settings
   *
   * @param node ROS node handle for advertise topic.
   * @param topic_name name of topic to publish (this may be image_raw).
   * @param buffer_size size of publisher buffer.
   * @param frame_id frame_id of publishing messages.
   */
  Capture(ros::NodeHandle &node,
          const std::string &topic_name,
          int32_t buffer_size,
          const std::string &frame_id);

  /**
   * @brief Open capture device with device ID.
   *
   * @param device_id id of camera device (number from 0)
   * @throw cv_camera::DeviceError device open failed
   *
   */
  void open(int32_t device_id);

  /**
   * @brief TODO <<<
   */
  void parseCustomSettings();
  void advertise();

  /**
   * @brief Open capture device with device name.
   *
   * @param device_path path of the camera device
   * @throw cv_camera::DeviceError device open failed
   */
  void open(const std::string &device_path);

  /**
   * @brief Load camera info from file.
   *
   * This loads the camera info from the file specified in the camera_info_url parameter.
   */
  void loadCameraInfo();

  /**
   * @brief Open default camera device.
   *
   * This opens with device 0.
   *
   * @throw cv_camera::DeviceError device open failed
   */
  void open();

  /**
   * @brief open video file instead of capture device.
   */
  void openFile(const std::string &file_path);

  /**
   * @brief capture an image and store.
   *
   * to publish the captured image, call publish();
   * @return true if success to capture, false if not captured.
   */
  bool capture();

  /**
   * @brief Publish the image that is already captured by capture().
   *
   */
  void publish();

  /**
   * @brief accessor of CameraInfo.
   *
   * you have to call capture() before call this.
   *
   * @return CameraInfo
   */
  // inline const sensor_msgs::CameraInfo &getInfo() const
  // {
  //   return info_;
  // }

  /**
   * @brief accessor of cv::Mat
   *
   * you have to call capture() before call this.
   *
   * @return captured cv::Mat
   */
  inline const cv::Mat &getCvImage(cv_bridge::CvImage &bridge) const
  {
    return bridge.image;
  }

  /**
   * @brief accessor of ROS Image message.
   *
   * you have to call capture() before call this.
   *
   * @return message pointer.
   */
  inline const sensor_msgs::ImagePtr getImageMsgPtr(cv_bridge::CvImage &bridge) const
  {
    return bridge.toImageMsg();
  }

  /**
   * @brief try capture image width
   * @return true if success
   */
  inline bool setWidth(int32_t width)
  {
    return cap_.set(CV_CAP_PROP_FRAME_WIDTH, width*2 /* As it gives double frame */);
  }

  /**
   * @brief try capture image height
   * @return true if success
   */
  inline bool setHeight(int32_t height)
  {
    return cap_.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  }

  /**
   * @brief set CV_PROP_*
   * @return true if success
   */
  bool setPropertyFromParam(int property_id, const std::string &param_name);

private:
  /**
   * @brief rescale camera calibration to another resolution
   */
  void rescaleCameraInfo(int width, int height, sensor_msgs::CameraInfo &info);

  /**
   * @brief node handle for advertise.
   */
  ros::NodeHandle node_;

  /**
   * @brief ROS image transport utility.
   */
  image_transport::ImageTransport left_it_;
  image_transport::ImageTransport right_it_;

  /**
   * @brief name of topic without namespace (usually "image_raw").
   */
  std::string topic_name_;

  /**
   * @brief header.frame_id for publishing images.
   */
  std::string frame_id_;
  /**
   * @brief size of publisher buffer
   */
  int32_t buffer_size_;

  /**
   * @brief image publisher created by image_transport::ImageTransport.
   */
  image_transport::CameraPublisher left_pub_;
  image_transport::CameraPublisher right_pub_;

  /**
   * @brief capture device.
   */
  cv::VideoCapture cap_;

  /**
   * @brief full frame from camera
   */
  cv::Mat base_frame_;

  std::string filter_type_;
  int         filter_kernel_sz_;

  /**
   * @brief this stores last captured image.
   */
  cv_bridge::CvImage left_bridge_;
  cv_bridge::CvImage right_bridge_;

  /**
   * @brief this stores last captured image info.
   *
   * currently this has image size (width/height) only.
   */
  sensor_msgs::CameraInfo left_info_;
  sensor_msgs::CameraInfo right_info_;

  /**
   * @brief camera info manager
   */
  camera_info_manager::CameraInfoManager left_info_manager_;
  camera_info_manager::CameraInfoManager right_info_manager_;

  /**
   * @brief rescale_camera_info param value
   */
  bool rescale_camera_info_;
};

} // namespace cv_camera

#endif // CV_CAMERA_CAPTURE_H
