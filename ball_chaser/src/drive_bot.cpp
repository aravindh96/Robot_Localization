#include "ros/ros.h"
#include "ball_chaser/DriveToPosition.h"
#include "geometry_msgs/Twist.h"


class CommandBot
{
    private:
        //Creating variables for NodeHandle, Publisher and ServiceServer types
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::ServiceServer serv_;
    public:
        CommandBot()
        {
            //Setting the ServiceServer and Publisher to respective service and topic
            serv_ = n_.advertiseService("/ball_chaser/command_bot", &CommandBot::Handle_Command_Bot , this);
            pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
        }
        
        bool Handle_Command_Bot(ball_chaser::DriveToPosition::Request& req, ball_chaser::DriveToPosition::Response& res)
        {
            //Creating a geometry_msgs/Twist variable and setting to the reqested values from service call
            geometry_msgs::Twist cmd_msg;
            cmd_msg.linear.x = req.linear_x;
            cmd_msg.angular.z = req.angular_z;
            
            //Publishing the command msg to /cmd_vel topic
            pub_.publish(cmd_msg);

            return true;

        }
};


int main(int argc, char** argv)
{
    //Initializing the node
    ros::init(argc,argv,"drive_bot");

    //Create CommandBot object
    CommandBot obj1;

    //Handle Communications
    ros::spin();

    return 0;
}