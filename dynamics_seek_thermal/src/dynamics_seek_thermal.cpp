#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "/home/pi/catkin_ws/src/Treiber_Seek/libseek-thermal/src/seek.h"
#include "/home/pi/catkin_ws/src/Treiber_Seek/libseek-thermal/src/SeekCam.h"

#include <iostream>
#include <string>
#include <signal.h>
#include <memory>

//#include <seek/seek.h>
#define FREQ 10 
#include <opencv2/core.hpp>
using namespace LibSeek;

/*enum
{
    COLORMAP_AUTUMN = 0,
    COLORMAP_BONE = 1,
    COLORMAP_JET = 2,
    COLORMAP_WINTER = 3,
    COLORMAP_RAINBOW = 4,
    COLORMAP_OCEAN = 5,
    COLORMAP_SUMMER = 6,
    COLORMAP_SPRING = 7,
    COLORMAP_COOL = 8,
    COLORMAP_HSV = 9,
    COLORMAP_PINK = 10,
    COLORMAP_HOT = 11
}*/

int main(int argc, char** argv)
{
    // Init ROS publisher
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    cv::Mat blur, smooth;

    // Create an image message and advertise it to the ROS network
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_l_grey = it.advertise("seek/left/image_grey", 1);
    //image_transport::Publisher pub_r_grey = it.advertise("seek/right/image_grey", 1);
    image_transport::Publisher pub_l_rainbow = it.advertise("seek/left/image_rainbow", 1);
    //image_transport::Publisher pub_r_rainbow = it.advertise("seek/right/image_rainbow", 1);

    // Create a SeekThermalPro camera
    LibSeek::SeekThermalPro seek_l("/home/pi/catkin_ws/src/dynamics_seek_thermal/config/seekpro_ffc_left.png", 0);
    if(!seek_l.open()) {
        ROS_ERROR("Unable to open SEEK Compact Pro camera");
        return -1;
    }

    std::cout << "chip ID of left camera: ";
    for (auto const& i: seek_l.chip_id()) {
		std::cout << i << " ";
	}
    std::cout << std::endl;

    // Create a SeekThermalPro camera
    /*LibSeek::SeekThermalPro seek_r("/home/rrt/dynamics_ws/src/sensors/dynamics_seek_thermal/config/seekpro_ffc_right.png", 1);
    if(!seek_r.open()) {
        ROS_ERROR("Unable to open SEEK Compact Pro camera");
        return -1;
    }

    std::cout << "chip ID of right camera: ";
    for (auto const& i: seek_r.chip_id()) {
		std::cout << i << " ";
	}
    std::cout << std::endl;*/

    // Setup seek camera
    LibSeek::SeekCam* seek;

    // ROS helper class to run the loop at a given frequency (Hz)
    ros::Rate loop_rate(FREQ);
    while (nh.ok()) {        
        // Read an image from the SEEK Compact PRO camera
        cv::Mat frameA, grey_frameA, rainbow_frameA, frameB, grey_frameB, rainbow_frameB; // = cv::imread("/home/dylan/catkin_ws/src/libseek_thermal_driver/lena.jpeg", CV_LOAD_IMAGE_COLOR);
        
        if(!seek_l.read(frameA)) {
            ROS_ERROR("SEEK Compact Pro camera cannot be read");
            return -2;
        }
        cv::normalize(frameA, grey_frameA, 0, 65535, cv::NORM_MINMAX);
        cv::GaussianBlur(grey_frameA, grey_frameA, cv::Size(7,7), 0);
        cv::addWeighted(grey_frameA, 1.5, grey_frameA, -0.5, 0, grey_frameA);
	    cv::flip(grey_frameA, grey_frameA, -1);

        /*if(!seek_r.read(frameB)) {
            ROS_ERROR("SEEK Compact Pro camera cannot be read");
            return -2;
        }
        cv::normalize(frameB, grey_frameB, 0, 65535, cv::NORM_MINMAX);
        cv::GaussianBlur(grey_frameB, grey_frameB, cv::Size(7,7), 0);
        cv::addWeighted(grey_frameB, 1.5, grey_frameB, -0.5, 0, grey_frameB);
	    cv::flip(grey_frameB, grey_frameB, -1);*/

        // Convert the image to a readable format for OpenCV
        grey_frameA.convertTo(grey_frameA, CV_8UC1, 1.0/256.0);
        cv::cvtColor(grey_frameA, grey_frameA, cv::COLOR_GRAY2BGR);
	    cv::applyColorMap(grey_frameA, rainbow_frameA, 2);
        sensor_msgs::ImagePtr msg_l_grey = cv_bridge::CvImage(std_msgs::Header(), "bgr8", grey_frameA).toImageMsg();
        sensor_msgs::ImagePtr msg_l_rainbow = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rainbow_frameA).toImageMsg();

        // Convert the image to a readable format for OpenCV
        /*grey_frameB.convertTo(grey_frameB, CV_8UC1, 1.0/256.0);
        cv::cvtColor(grey_frameB, grey_frameB, cv::COLOR_GRAY2BGR);
	    cv::applyColorMap(grey_frameB, rainbow_frameB, 2);
        sensor_msgs::ImagePtr msg_r_grey = cv_bridge::CvImage(std_msgs::Header(), "bgr8", grey_frameB).toImageMsg();
        sensor_msgs::ImagePtr msg_r_rainbow = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rainbow_frameB).toImageMsg();*/

        // Publish message to the ROS network
        pub_l_grey.publish(msg_l_grey);
        //pub_r_grey.publish(msg_r_grey);
        pub_l_rainbow.publish(msg_l_rainbow);
        //pub_r_rainbow.publish(msg_r_rainbow);

        // Process callbacks from ROS, see https://answers.ros.org/question/11887/significance-of-rosspinonce/
        ros::spinOnce();

        // Sleep to comply with the given loop_rate
        loop_rate.sleep();
    }
}
