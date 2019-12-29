#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include<ros/package.h>
#include "detectlane.h"
#include "carcontrol.h"

bool STREAM = true;

DetectLane *detect;
CarControl *car;
int skipFrame = 1;
Point p_center;
int _turn;
vector<int> flag1;
bool flag2 = false;
int decision = 0 ;
double theta;
Mat view;
string path = ros::package::getPath("team507");
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        view = cv_ptr->image.clone();
        detect->update(cv_ptr->image);
        p_center = detect->getPoint();
        circle(view,p_center, 5, Scalar(255, 255, 0), 3);
        imshow("View",view);
        car->driverCar(p_center,55,decision,flag2);
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    // cv::namedWindow("View");
    // cv::namedWindow("Threshold");
    // cv::namedWindow("Binary");
    // cv::namedWindow("Lane");
    detect = new DetectLane();
    car = new CarControl();
    if (STREAM) {
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("HTH/camera/rgb", 1, imageCallback);

        ros::spin();
    } 
    cv::destroyAllWindows();
}
