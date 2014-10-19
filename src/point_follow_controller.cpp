#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <cmath>

class PointFollowController{

private:
    geometry_msgs::Point targetPoint;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    double linearScaler;
    double angularScaler;

    double distanceOffset;

public:
    PointFollowController(){

        //TODO: change namespace??
        nh = ros::NodeHandle();

        sub = nh.subscribe("/hand_detector/target", 1, &PointFollowController::pointMsgCallback, this);
        pub = nh.advertise<geometry_msgs::Twist>("motor_controller/twist", 1);

        linearScaler = 1.0;
        angularScaler = 0.0;

        distanceOffset = 0.4;

        return;
    }

    void pointMsgCallback(const geometry_msgs::Point::ConstPtr& msg){
        targetPoint = *msg;

        return;
    }

    void follow(){

        double distance = sqrt(pow(targetPoint.x, 2) + pow(targetPoint.z, 2));

        geometry_msgs::Twist motorMsg;


        if(distance > 0){
            motorMsg.linear.x = linearScaler * (distance - distanceOffset);
        }
        else{
            motorMsg.linear.x = 0.0;
        }
        motorMsg.angular.z = angularScaler * targetPoint.x;

        pub.publish(motorMsg);

        return;
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "point_follow_controller");

    PointFollowController pfController;

    ros::Rate loop_rate(5);

    while(ros::ok()){

        pfController.follow();

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
