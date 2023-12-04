#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Converting the image to grayscale to simplify the circles detection
    cv::Mat gray_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

    // Circles detecting by using Hough transform
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray_image, circles, cv::HOUGH_GRADIENT, 1,
                      gray_image.rows / 8,  // min distance between detected centers
                      100, 30, 5, 100);    // Hough transform parameters 

    // Draw the detected circle on the original image
    for (size_t i = 0; i < circles.size(); i++)
    {
      cv::Vec3i circle = circles[i];
      cv::Point center(circle[0], circle[1]);
      int radius = circle[2];

      // Draw the circle
      cv::circle(cv_ptr->image, center, radius, cv::Scalar(0, 0, 255), 3); // color = red, thickness = 3
    }

    // Update the GUI window with the modified image
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Publish the image with the circles
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
