#include "carcontrol.h"
#include <ros/ros.h>
#include "std_msgs/Int32.h"



//===================================================================
bool STREAM = true;
//===================================================================
int ketQua;
void BBCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ketQua = msg->data;
}

CarControl::CarControl()
{
    carPos.x = 120;
    carPos.y = 300;   
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("HTH/set_angle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("HTH/set_speed",10);
}

CarControl::~CarControl() {}

float CarControl::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y; 
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity)
{
    int i = left.size() - 11;
    float error = preError;
    //=================================================================
    ros::NodeHandle nh;
    ros::Subscriber sub2 = nh.subscribe("bienBao", 10, BBCallback); 
    ROS_INFO("I heard: %d", ketQua);
    //=================================================================
    // left[i] = 26389976 || right = 26389704

    while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
        i--;
        while(left[i] == DetectLane::null || right[i] == DetectLane::null)
        {
            i--;
        }
       	if (i < 0) return;
    }
    
     
    if (ketQua == 1)
    {
        velocity = 0;
        error = -90;
        // while(left[i] == DetectLane::null || right[i] == DetectLane::null)
        // {
        //     velocity = 0;
        //     error = -90;
        //     break;
        // }
    }
    else if(ketQua == 2)
    {
        velocity = 0;
	    error = 90;
    }
    else if(left[i] != DetectLane::null && right[i] !=  DetectLane::null)
    {
        error = errorAngle((left[i] + right[i]) / 2);
    }
    else if (left[i] != DetectLane::null)
    {
        //error = errorAngle(left[i] + Point(laneWidth / 2, 0));
        
        error = -4;
    }
    else 
    {
        //error = errorAngle(right[i] - Point(laneWidth / 2, 0));
        error = 4;
    }

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
 
    ros::spin();
}
