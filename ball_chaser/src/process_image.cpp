#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "ball_chaser/DriveToPosition.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include <string>

class Process_Img_Client_Call
{
    private:
    //Creating variables of NodeHandle, Subscriber and Client
    ros::NodeHandle n_;
    ros::Subscriber sub_img_;
    ros::ServiceClient client_;

    public :
    Process_Img_Client_Call()
    {
        //Setting Subscriber and Client to respecitive topic and service calls
        sub_img_ = n_.subscribe("/camera/rgb/image_raw", 10 , &Process_Img_Client_Call::Process_Img_Callback,this);
        client_ = n_.serviceClient<ball_chaser::DriveToPosition>("/ball_chaser/command_bot"); 
    }

    void Process_Img_Callback(const sensor_msgs::Image img)
    {
       
            int min_col = img.step;
            int max_col = 0;
            int mid_point =0;
            int i=0;
            ball_chaser::DriveToPosition cmd_pos;
            cmd_pos.request.linear_x = 0;
            cmd_pos.request.angular_z = 0;

            //Finding the column limits of the white ball
            for(int i=0; i<img.height*img.step; i++)
            {
                if(img.data[i] == 255)
                {
                    min_col = std::min(min_col, i%(int)img.step);
                    max_col = std::max(max_col, i%(int)img.step);
            
                }
            }

            mid_point = (min_col + max_col)/2;

            if(mid_point == img.step/2) //White Ball not in the scene
            {
                cmd_pos.request.angular_z = 1;
            }
            else
            {
                 
                if((max_col-min_col) > img.step/2)//White ball is too close to move
                {
                    cmd_pos.request.angular_z = 0;
                    cmd_pos.request.linear_x = 0;
                }
                else if(mid_point<img.step/3) //White ball in left side of scene
                    cmd_pos.request.angular_z = 1;
                
                else if(mid_point<(0.67*img.step))  //White ball in center of scene
                    cmd_pos.request.linear_x = 1.5;
                
                else  //White ball in right side of scene
                    cmd_pos.request.angular_z = -1;

            }
           
            //Calling the service /ball_chaser/command_bot
            if(!client_.call(cmd_pos))
            {
                ROS_WARN("Service Call Failed");
            }

    }
};


int main(int argc, char** argv)
{
    //Initialize the node
    ros::init(argc,argv,"process_image");

    //Create Process_img object
    Process_Img_Client_Call obj1;

    ros::Duration(1).sleep();
    
    //Handle communication
    ros::spin();

    

    return 0;

}