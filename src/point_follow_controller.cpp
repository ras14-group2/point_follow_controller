#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>

#include <cmath>

class PointFollowController{

private:
    geometry_msgs::Point targetPoint;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub, distPub;

    double D;

public:
    PointFollowController(){

        //TODO: change namespace??
        nh = ros::NodeHandle();

        sub = nh.subscribe("/hand_detector/target", 1, &PointFollowController::pointMsgCallback, this);
        pub = nh.advertise<geometry_msgs::Twist>("motor_controller/twist", 1);
        
        distPub = nh.advertise<std_msgs::Float64>("/psdistance", 1);

        D = 0.5; //distance offset

        return;
    }

    void pointMsgCallback(const geometry_msgs::Point::ConstPtr& msg){
        targetPoint = *msg;

        return;
    }

    void follow(){

        double distance = sqrt(pow(targetPoint.x, 2) + pow(targetPoint.z, 2));

        geometry_msgs::Twist motorMsg;

        if(distance <= 0){
            motorMsg.linear.x = 0.0;
//            ROS_INFO("distance <= 0");
        }
        else if(fabs(distance-D) < 0.05){
            motorMsg.linear.x = 0;
//            ROS_INFO("abs(distance-D) is: %f", abs(distance-D));
//            ROS_INFO("distance: %f", distance);
//            ROS_INFO("D is %f", D);
        }
        else if(distance > D+0.15){
            motorMsg.linear.x = 0.4;
        }
        else if(distance > D+0.05){
            motorMsg.linear.x = 0.2;
        } else if (distance < D-0.15) {
        	motorMsg.linear.x = -0.4;
        } else if (distance < D-0.05) {
        	motorMsg.linear.x = -0.2;
        }
        
        double angle = atan(targetPoint.x / targetPoint.z);
        
        if(angle > 0.05){
        	motorMsg.angular.z = -0.09;
        	if (angle > 0.25) {
        		motorMsg.angular.z = -0.2;
        	}
        }
        else if(angle < -0.05){
        	motorMsg.angular.z = 0.09;
        	if (angle < -0.25) {
        		motorMsg.angular.z = 0.2;
        	}
        }
        else{
        	motorMsg.angular.z = 0.0;
        }

        pub.publish(motorMsg);
        
        std_msgs::Float64 distmsg;
        distmsg.data = distance;
        distPub.publish(distmsg);
        
//        ROS_INFO("linear speed is: %f", motorMsg.linear.x);
//        ROS_INFO("distance is: %f", distance);

        return;
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "point_follow_controller");

    PointFollowController pfController;

    ros::Rate loop_rate(10);

    while(ros::ok()){

        pfController.follow();

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
