#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>

#include <cmath>


#define D 0.5


class PointFollowController{

private:
    geometry_msgs::Point targetPoint;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub, distPub;

//    double linearScaler;
//    double angularScaler;

//    double distanceOffset;

public:
    PointFollowController(){

        //TODO: change namespace??
        nh = ros::NodeHandle();

        sub = nh.subscribe("/hand_detector/target", 1, &PointFollowController::pointMsgCallback, this);
        pub = nh.advertise<geometry_msgs::Twist>("motor_controller/twist", 1);
        
        distPub = nh.advertise<std_msgs::Float64>("/psdistance", 1);

        //linearScaler = 1.0;
        //angularScaler = 0.0;

        //distanceOffset = 0.4;

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
            std::cerr << "first if" << std::endl;
        }
        else if(fabs(distance-D) < 0.05){
            motorMsg.linear.x = 0;
            std::cerr << "second if" << std::endl;
            std::cerr << "abs(distance-D) is: " << abs(distance-D) << std::endl;
            std::cerr << "distance " << distance << std::endl;
            std::cerr << "D is: " << D << std::endl;
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

        //motorMsg.angular.z = - std::max(-0.3, std::min(0.3, std::atan(targetPoint.x / targetPoint.z)));
        
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
        
        /*
        if(targetPoint.x > 0.05){
        	motorMsg.angular.z = -0.09;
        }
        else if(targetPoint.x < -0.05){
        	motorMsg.angular.z = 0.09;
        	if (targetPoint.x < -0.2) {
        		motorMsg
        	}
        }
        else{
        	motorMsg.angular.z = 0.0;
        }
        */
        
        

        pub.publish(motorMsg);
        
        std_msgs::Float64 distmsg;
        distmsg.data = distance;
        distPub.publish(distmsg);
        
        
        std::cerr << "linear speed is: " << motorMsg.linear.x << std::endl;
        std::cerr << "distance is: " << distance << std::endl;

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
