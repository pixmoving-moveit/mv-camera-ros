// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "driver.h"
#include <string>



namespace
{
const double DEFAULT_RATE = 30.0;
const int32_t PUBLISHER_BUFFER_SIZE = 1;
}


boost::shared_ptr<cv_camera::MVCameraCapture> sCamera;


namespace cv_camera
{

void configure_callback(mv_camera::CameraConfig &config, uint32_t level);


Driver::Driver(ros::NodeHandle &private_node, ros::NodeHandle &camera_node)
    : private_node_(private_node),
      camera_node_(camera_node)
{
}

void Driver::setup()
{
  double hz(DEFAULT_RATE);
  int32_t device_id(0);
  std::string device_path("");
  std::string frame_id("camera");
  std::string file_path("");

  private_node_.getParam("device_id", device_id);
  private_node_.getParam("frame_id", frame_id);
  private_node_.getParam("rate", hz);

  int32_t image_width(640);
  int32_t image_height(480);

  camera_.reset(new MVCameraCapture(camera_node_,
                            "image_raw",
                            PUBLISHER_BUFFER_SIZE,
                            frame_id));

  sCamera = camera_;
  std::cout << "openin the camera device id \n";
  camera_->open(device_id);
  if (private_node_.getParam("image_width", image_width))
  {
    if (!camera_->setWidth(image_width))
    {
      ROS_WARN("fail to set image_width");
    }
  }
  if (private_node_.getParam("image_height", image_height))
  {
    if (!camera_->setHeight(image_height))
    {
      ROS_WARN("fail to set image_height");
    }
  }

  dynamic_reconfigure::Server<mv_camera::CameraConfig>::CallbackType f;
  f = boost::bind(&configure_callback, _1, _2);
  server_.setCallback(f);

  /*

  camera_->setPropertyFromParam(CV_CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
  camera_->setPropertyFromParam(CV_CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
  camera_->setPropertyFromParam(CV_CAP_PROP_FPS, "cv_cap_prop_fps");
  camera_->setPropertyFromParam(CV_CAP_PROP_FOURCC, "cv_cap_prop_fourcc");
  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_COUNT, "cv_cap_prop_frame_count");
  camera_->setPropertyFromParam(CV_CAP_PROP_FORMAT, "cv_cap_prop_format");
  camera_->setPropertyFromParam(CV_CAP_PROP_MODE, "cv_cap_prop_mode");
  camera_->setPropertyFromParam(CV_CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
  camera_->setPropertyFromParam(CV_CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
  camera_->setPropertyFromParam(CV_CAP_PROP_SATURATION, "cv_cap_prop_saturation");
  camera_->setPropertyFromParam(CV_CAP_PROP_HUE, "cv_cap_prop_hue");
  camera_->setPropertyFromParam(CV_CAP_PROP_GAIN, "cv_cap_prop_gain");
  camera_->setPropertyFromParam(CV_CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
  camera_->setPropertyFromParam(CV_CAP_PROP_CONVERT_RGB, "cv_cap_prop_convert_rgb");

  camera_->setPropertyFromParam(CV_CAP_PROP_RECTIFICATION, "cv_cap_prop_rectification");
  camera_->setPropertyFromParam(CV_CAP_PROP_ISO_SPEED, "cv_cap_prop_iso_speed");
#ifdef CV_CAP_PROP_WHITE_BALANCE_U
  camera_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_U, "cv_cap_prop_white_balance_u");
#endif // CV_CAP_PROP_WHITE_BALANCE_U
#ifdef CV_CAP_PROP_WHITE_BALANCE_V
  camera_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_V, "cv_cap_prop_white_balance_v");
#endif // CV_CAP_PROP_WHITE_BALANCE_V
#ifdef CV_CAP_PROP_BUFFERSIZE
  camera_->setPropertyFromParam(CV_CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");
#endif // CV_CAP_PROP_BUFFERSIZE
*/
  rate_.reset(new ros::Rate(hz));
}

void configure_callback(mv_camera::CameraConfig &config, uint32_t level)
{
    ROS_INFO("Reconfiguring...");


    double exposure = config.Exposure_time;
    double gain = config.Gain;

    cv::Vec3b colorGain(config.Red_gain, config.Green_gain, config.Blue_gain);

    sCamera->setExposure(exposure);
    sCamera->setGain(gain);

    if (config.Trigger_WB)
      sCamera->triggerWhiteBalance();
    else
    {
      sCamera->setColorGain(colorGain);
    }

    bool autoExposureMode = config.Exposure_auto != "Manual";
    bool autoGainMode = config.Gain_auto != "Manual";

    sCamera->setAutoExposureMode(autoExposureMode);

    std::cout << "AutoExposure: " << autoExposureMode << std::endl;
    std::cout << "AutoGain: " << autoGainMode << std::endl;

    /*
    // Set relative intensity of LEDs.
    ROS_DEBUG("INTENSITY: %d", config.intensity);
    LeddarSetProperty(handler, PID_LED_INTENSITY, 0, config.intensity);

    // Set automatic LED intensity.
    ROS_DEBUG("AUTO INTENSITY: %s", config.auto_intensity ? "true" : "false");
    LeddarSetProperty(handler, PID_AUTOMATIC_LED_INTENSITY, 0,
                      config.auto_intensity);

    // Set number of accumulations to perform.
    ROS_DEBUG("ACCUMULATIONS: %d", config.accumulations);
    LeddarSetProperty(handler, PID_ACCUMULATION_EXPONENT, 0,
                      config.accumulations);

    // Set number of oversamplings to perform between base samples.
    ROS_DEBUG("OVERSAMPLING: %d", config.oversampling);
    LeddarSetProperty(handler, PID_OVERSAMPLING_EXPONENT, 0,
                      config.oversampling);

    // Set number of base samples acquired.
    ROS_DEBUG("BASE SAMPLES: %d", config.base_point_count);
    LeddarSetProperty(handler, PID_BASE_POINT_COUNT, 0,
                      config.base_point_count);

    // Set offset to increase detection threshold.
    ROS_DEBUG("THRESHOLD OFFSET: %d", config.threshold_offset);
    LeddarSetProperty(handler, PID_THRESHOLD_OFFSET, 0,
                      config.threshold_offset);

    // Set detection of 2 objects close to each other.
    ROS_DEBUG("DEMERGING: %s", config.object_demerging ? "true" : "false");
    LeddarSetProperty(handler, PID_OBJECT_DEMERGING, 0,
                      config.object_demerging);

    // Write changes to Leddar.
    LeddarWriteConfiguration(handler);
    */
}



void Driver::proceed()
{
  if (camera_->capture())
  {
    camera_->publish();
  }
  rate_->sleep();
}

Driver::~Driver()
{
}

} // namespace cv_camera
