#include <sstream>
#include <string>

#include "MVCameraCapture.h"


namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

MVCameraCapture::MVCameraCapture(ros::NodeHandle &node, const std::string &topic_name,
                 int32_t buffer_size, const std::string &frame_id)
    : node_(node),
      it_(node_),
      topic_name_(topic_name),
      buffer_size_(buffer_size),
      frame_id_(frame_id),
      info_manager_(node_, frame_id)
{
}

void MVCameraCapture::loadCameraInfo()
{
  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }

  rescale_camera_info_ = node_.param<bool>("rescale_camera_info", false);

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

void MVCameraCapture::rescaleCameraInfo(int width, int height)
{
  double width_coeff = width / info_.width;
  double height_coeff = height / info_.height;
  info_.width = width;
  info_.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info_.K[0] *= width_coeff;
  info_.K[2] *= width_coeff;
  info_.K[4] *= height_coeff;
  info_.K[5] *= height_coeff;

  info_.P[0] *= width_coeff;
  info_.P[2] *= width_coeff;
  info_.P[5] *= height_coeff;
  info_.P[6] *= height_coeff;
}

void MVCameraCapture::open(int32_t device_id)
{
  int                     iCameraCounts = 1;
  int                     iStatus=-1;
  tSdkCameraDevInfo       tCameraEnumList;
  tSdkCameraCapbility     tCapability;      //设备描述信息
  int                     channel=3;

  // 0 for English, 1 for Chinese
  int status = CameraSdkInit(1);
  std::cout << "Camera SDK init status : " << status << std::endl;


  status = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
  if(iCameraCounts==0)
  {
    std::cout << "No camera found" << std::endl;
    std::cout << "Status = " << status << std::endl;

    std::stringstream stream;
    stream << "No camera found";
    throw DeviceError(stream.str());
  }

  iStatus = CameraInit(&tCameraEnumList,-1,-1, &hCamera_);
  std::cout << "hCamera = " << hCamera_ << std::endl;

  if(iStatus!=CAMERA_STATUS_SUCCESS)
  {
    std::stringstream stream;
    stream << "device_id" << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }

  CameraGetCapability(hCamera_,&tCapability);
  std::cout << "buffer size" <<  tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3 << std::endl;
  rgbBuffer_ = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);


  tSdkImageResolution psCurVideoSize;

  CameraGetImageResolution(hCamera_, &psCurVideoSize);
  psCurVideoSize.iWidthZoomHd = 1024;
  psCurVideoSize.iHeightZoomHd = 1024;
 CameraSetImageResolution(hCamera_, &psCurVideoSize);

  CameraPlay(hCamera_);

  if(tCapability.sIspCapacity.bMonoSensor)
  {
    channel=1;
    CameraSetIspOutFormat(hCamera_,CAMERA_MEDIA_TYPE_MONO8);
  }

  else
  {
    channel=3;
    CameraSetIspOutFormat(hCamera_,CAMERA_MEDIA_TYPE_BGR8);
  }

  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);

  //loadCameraInfo();
}

void MVCameraCapture::open()
{
  open(0);
  // CameraSetAeState(hCamera_,FALSE);
}

bool MVCameraCapture::capture()
{
  uchar * pbyBuffer =0;
  if(CameraGetImageBuffer(hCamera_, &frameInfo_,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
  {
    CameraImageProcess(hCamera_, pbyBuffer, rgbBuffer_, &frameInfo_);
    //char *buffer = (char *)malloc(2048 * 2048 * 3);
    //memcpy(rgbBuffer_, buffer, 2048 * 2048 * 3);
    bridge_.image = cv::Mat(frameInfo_.iHeight, frameInfo_.iWidth, CV_8UC3, rgbBuffer_, cv::Mat::AUTO_STEP);

     // std::cout << bridge_.image.rows << std::endl;
    cv::resize(bridge_.image, bridge_.image, cv::Size(1024, 1024));

    //bridge_.image = image;

    //cv::imshow("iamge", image);
    //cv::waitKey(30);

    ros::Time now = ros::Time::now();
    bridge_.encoding = enc::BGR8;
    bridge_.header.stamp = now;
    bridge_.header.frame_id = frame_id_;

    info_ = info_manager_.getCameraInfo();
    if (info_.height == 0 && info_.width == 0)
    {
      info_.height = bridge_.image.rows;
      info_.width = bridge_.image.cols;
    }
    else if (info_.height != bridge_.image.rows || info_.width != bridge_.image.cols)
    {
      if (rescale_camera_info_)
      {
        int old_width = info_.width;
        int old_height = info_.height;
        rescaleCameraInfo(bridge_.image.cols, bridge_.image.rows);
        ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                      old_width, old_height, bridge_.image.cols, bridge_.image.rows);
      }
      else
      {
        ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      info_.width, info_.height, bridge_.image.cols, bridge_.image.rows);
      }
    }
    info_.header.stamp = now;
    info_.header.frame_id = frame_id_;


    CameraReleaseImageBuffer(hCamera_, pbyBuffer);
    return true;
  }
  return false;
}

void MVCameraCapture::publish()
{
  pub_.publish(*getImageMsgPtr(), info_);
}

void MVCameraCapture::setExposure(double value)
{
  if (autoExposureOn_)
    CameraSetAeTarget(hCamera_, value);
  else
    CameraSetExposureTime(hCamera_, value);

}

void MVCameraCapture::setGain(double value)
{
  //CameraSetGain(hCamera_, value, value, value);
  if (!autoGainOn_)
    CameraSetAnalogGain(hCamera_, value);
}

void MVCameraCapture::setAutoExposureMode(bool value)
{
  autoExposureOn_ = value;
  CameraSetAeState(hCamera_, autoExposureOn_);
}

void MVCameraCapture::setAutoGainMode(bool value)
{
  autoGainOn_ = value;
}

void MVCameraCapture::setColorGain(cv::Vec3b colorGain)
{
  CameraSetGain(hCamera_, colorGain[0], colorGain[1], colorGain[2]);
}

void MVCameraCapture::triggerWhiteBalance()
{
  CameraSetOnceWB(hCamera_);
}

bool MVCameraCapture::setPropertyFromParam(int property_id, const std::string &param_name)
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
