#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <vector>

float t = 0;
float n = 0;
float dx = 0;
float dy = 0;
float dz = 0;
float dpsi = 0;
float x = -5;
float y = 0;
float z= 0;
float psi = (M_PI/2);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reference");
    ros::NodeHandle nh;
    ros::Publisher ref_pos = nh.advertise<geometry_msgs::Vector3>("ref_pos", 100);
    ros::Publisher ref_vel = nh.advertise<geometry_msgs::Vector3>("vel_pos", 100);
    ros::Publisher ref_yaw = nh.advertise<geometry_msgs::Vector3>("yaw_pos", 100);

    ros::Rate loop_rate(100);

    geometry_msgs::Vector3 val_pos;
    geometry_msgs::Vector3 val_vel;
    geometry_msgs::Vector3 val_yaw;

    while (ros::ok())
    {
        t = n*0.01;
        if (t > 0 && t<5){
            dx = 0;
            dz = -0.5;
            dy = 0;
            dpsi = -0;
        }
        if (t>=5){ // Aqui se agrega el limite de 65s
            dx = 0.5*sin(0.1*(t-5));
            dy = 0.5*cos(0.1*(t-5));
            dz = 0;
            dpsi = -0.1;
        }
        

        x = x+ dx*0.01;
        y = y+ dy*0.01;
        z = z+ dz*0.01;

        psi = psi + dpsi*0.01;

        val_pos.x = x;
        val_pos.y = y;
        val_pos.z = z;

        val_vel.x = dx;
        val_vel.y = dy;
        val_vel.z = dz;

        val_yaw.x = psi;
        val_yaw.y = dpsi;

        ref_pos.publish(val_pos);
        ref_vel.publish(val_vel);
        ref_yaw.publish(val_yaw);
        ros::spinOnce();

        loop_rate.sleep();
        n++;
        
    }

    return 0;
}