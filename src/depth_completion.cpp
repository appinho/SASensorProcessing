#include <stereo_image/depth_completion.h>
#include <Eigen/Dense>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace stereo_image
{
DepthCompletion::DepthCompletion(ros::NodeHandle nh) : 
  nh_(nh),
  exact_sync_(ExactPolicy(10),
  sub_pointcloud_, sub_left_color_camera_info_, sub_left_color_image_)
{
  // Set up a dynamic reconfigure server.
  // Do this before parameter server, else some of the parameter server values can be overwritten.
  dynamic_reconfigure::Server<stereo_image::DepthCompletionParamsConfig>::CallbackType cb;
  cb = boost::bind(&DepthCompletion::configCallback, this, _1, _2);
  dr_srv_.setCallback(cb);

  // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
  // of the node can be run simultaneously while using different parameters.
  ros::NodeHandle pnh("~");

  sub_pointcloud_.subscribe(nh_, "/kitti/velo/pointcloud", 1);
  sub_left_color_camera_info_.subscribe(nh_, "/kitti/camera_color_left/camera_info", 1);
  sub_left_color_image_.subscribe(nh_, "/kitti/camera_color_left/image_raw", 1);
  exact_sync_.registerCallback(boost::bind(&DepthCompletion::callback,
                                              this, _1, _2, _3));

  pub_depth_image_ = nh_.advertise<Image>("/kitti/depth_image", 1);
  pub_completed_image_ = nh_.advertise<Image>("/kitti/completed_image", 1);
  pub_completed_pointcloud_ = nh_.advertise<PointCloud2>("/kitti/completed_pointcloud", 1);

}

bool DepthCompletion::inImage(const CameraInfoConstPtr& cam_info, const int u, const int v)
{
  return(u >= 0 && u < cam_info->height && v >= 0 && v < cam_info->width);
}

void DepthCompletion::depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols)
  {
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);
  }
  //The following doesn't work if there are NaNs
  double minVal, maxVal; 
  minMaxLoc(float_img, &minVal, &maxVal);
  ROS_INFO("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  cv::convertScaleAbs(float_img, mono8_img, 2.55, 0.0);
}

void DepthCompletion::callback(const PointCloud2ConstPtr& pc_msg,
                               const CameraInfoConstPtr& l_info_msg,
                               const ImageConstPtr& l_image_msg)
{
  // 1. To cam frame
  std::string target_frame = "camera_color_left";
  PointCloud2 pc_cam;
  bool transformed = pcl_ros::transformPointCloud(target_frame, *pc_msg, pc_cam, listener_);
  
  // 2. To rect cam frame
  PointCloud2 pc_rect_cam;
  Eigen::Matrix4f R = Eigen::Matrix4f();
  R << l_info_msg->R[0], l_info_msg->R[1], l_info_msg->R[2], 0,
       l_info_msg->R[3], l_info_msg->R[4], l_info_msg->R[5], 0,
       l_info_msg->R[6], l_info_msg->R[7], l_info_msg->R[8], 0,
                      0,                0,                0, 1;
  pcl_ros::transformPointCloud(R, pc_cam, pc_rect_cam);

  // 3. To image frame
  PointCloud2 pc_img;
  Eigen::Matrix4f P = Eigen::Matrix4f();
  P << l_info_msg->P[0], l_info_msg->P[1], l_info_msg->P[2], l_info_msg->P[3],
       l_info_msg->P[4], l_info_msg->P[5], l_info_msg->P[6], l_info_msg->P[7], 
       l_info_msg->P[8], l_info_msg->P[9], l_info_msg->P[10], l_info_msg->P[11],
                      0,                0,                 0,                 0;
  pcl_ros::transformPointCloud(P, pc_rect_cam, pc_img);

  // Init images
  cv::Mat depth_image = cv::Mat::zeros(l_info_msg->width, l_info_msg->height, CV_32FC1);
  cv::Mat inv_depth_image = cv::Mat::zeros(l_info_msg->width, l_info_msg->height, CV_32FC1);
  cv::Mat l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::BGR8)->image;

  PointCloud2Iterator<float> iter_cam_z(pc_rect_cam, "z");
  PointCloud2Iterator<float> iter_img_x(pc_img, "x");
  PointCloud2Iterator<float> iter_img_y(pc_img, "y");
  PointCloud2Iterator<float> iter_img_z(pc_img, "z");

  for (; iter_cam_z != iter_cam_z.end();
       ++iter_cam_z, ++iter_img_x, ++iter_img_y, ++iter_img_z)
  {

    const float& cam_z = *iter_cam_z;
    if (cam_z <= 0) continue;

    const float& img_x = *iter_img_x;
    const float& img_y = *iter_img_y;
    const float& img_z = *iter_img_z;

    if (img_z == 0) continue;
    const int u = img_x / img_z;
    const int v = img_y / img_z; 


    if (inImage(l_info_msg, u, v))
    {
      if (depth_image.at<float>(v, u) == 0  ||
          depth_image.at<float>(v, u) > cam_z)
      {
        // Fill images
        float factor = std::min(1.0, double(cam_z) / 40);
        int r = int(factor * 255);
        int g = 255;
        int b = int((1 - factor) * 255);
        for(int i = -2; i <=2; i++)
        {
          int r = 2 - std::abs(i);
          for(int j = r; j <= r; j++)
          {
            int x = u + i;
            int y = v + j;
            if (inImage(l_info_msg, x, y))
            {
              depth_image.at<float>(y, x) = cam_z;
              inv_depth_image.at<float>(y, x) = 100 - cam_z;
              l_image.at<cv::Vec3b>(y, x) = cv::Vec3b(r, g, b);
            }
          }
        }
      }
    }
  }

  cv::Mat kernel = cv::Mat(5, 5, CV_8UC1, 1);
  /// Apply the dilation operation
  cv::Mat dilated_depth_image;
  cv::dilate(inv_depth_image, dilated_depth_image, kernel);

  cv::Mat display_depth_image;
  depthToCV8UC1(dilated_depth_image, display_depth_image);

  cv_bridge::CvImage cv_bridge_depth_image;
  cv_bridge_depth_image.image = display_depth_image;
  cv_bridge_depth_image.encoding = "mono8";
  cv_bridge_depth_image.header.stamp = l_info_msg->header.stamp;
  pub_depth_image_.publish(cv_bridge_depth_image.toImageMsg());
}

void DepthCompletion::configCallback(stereo_image::DepthCompletionParamsConfig &config, uint32_t level)
{

}


void DepthCompletion::processDepthCompletion()
{

}

}
