#include "carcontrol.h"
#include <ros/ros.h>
#include "std_msgs/Int32.h"

int ketQua;
void BBCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ketQua = msg->data;
}

CarControl::CarControl()
{
    carPos.x = 160;
    carPos.y = 240;
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

void CarControl::driverCar(const Point &PC, float velocity,int flag,bool flag2)
{
    float error;
    Point prevP;
    error = errorAngle(PC);

    ros::NodeHandle nh;
    ros::Subscriber sub2 = nh.subscribe("bienBao", 10, BBCallback); 
    // if(flag_p == true)
    // {
    //     error = errorAngle(prevP);
    //     flag_p = false;
    // }
    // else prevP = PC;
    if (ketQua == 1)
    {
        velocity = 35;
        error = -85;
    }
    else if(ketQua == 2)
    {
        velocity = 35;
        error = 85;
    }
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);
    // if(flag2 == true)
    // {
    //     // if(flag == 1)
    //     // {
    //     //     //error = -15.0;
    //     //     error = -10.0;
    //     //     velocity = 40;
    //     //     }
    //     // else if(flag == 2)
    //     // {
    //     //     error = 15.0;
    //     //     //error = 10.0;
    //     //     velocity = 40;
            
    //     // }
    //     flag_p = true;
    //     std_msgs::Float32 angle;
    //     std_msgs::Float32 speed;

    //     angle.data = error;
    //     speed.data = velocity;

    //     steer_publisher.publish(angle);
    //     speed_publisher.publish(speed);
    //     sleep(1);
    // }
    // else{
    
    //     std_msgs::Float32 angle;
    //     std_msgs::Float32 speed;
    //     if(error > 12 || error < -12)
    //         velocity-=10;
    //     angle.data = error;
    //     speed.data = velocity;

    //     steer_publisher.publish(angle);
    //     speed_publisher.publish(speed);
    // }
	ros::spin();
} 
