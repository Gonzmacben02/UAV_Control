#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "gazebo_msgs/ModelState.h"
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <iostream>

float x = 0.0;
float y = 0.0;
float z = 0.0;
float x_des = 2.0;
float y_des = -3.0;
float z_des = 1;
float x_dot_des = 2.0;
float y_dot_des = -3.0;
float z_dot_des = 1;
float x_dot = 0.0;
float y_dot = 0.0;
float z_dot = 0.0;
float m = 2;
float g = 9.81;
float th = 0.0;
float phi_roll = 0.0;
float theta_pitch = 0.0;
float psi_yaw = 0.0;
float phi_roll_des = 0.0;
float theta_pitch_des = 0.0;
float psi_yaw_des = 0.0;
float psi_yaw_dot_des = 0.0;
float ex = 0.0;
float ey = 0.0;
float ez = 0.0;
float ex_dot = 0.0;
float ey_dot = 0.0;
float ez_dot = 0.0;
float contador = 0.0;

float kpx = 0.0;
float kpy = 0.0;
float kpz = 0.0;
float kdx = 0.6;
float kdy = 0.6;
float kdz = 1.0;

float kpx_dot = 0.0;
float kpy_dot = 0.0;
float kpz_dot = 0.0;
float kdx_dot = 0.0;
float kdy_dot = 0.0;
float kdz_dot = 0.0;

float Uvx, Uvy, Uvz;
float step = 0.01; // step = 1/100


Eigen::Vector3f pos_dyn;
Eigen::Vector3f vel_dyn;
Eigen::Vector3f pos_des;
Eigen::Vector3f vel_des;
Eigen::Vector3f ang_dyn;
Eigen::Vector3f yaw_des;
Eigen::Vector3f error;
Eigen::Vector3f error_dot;

Eigen::Vector3f pos_ctrl;
Eigen::Vector3f kp;
Eigen::Vector3f kd;
Eigen::Vector3f kp_dot;
Eigen::Vector3f kd_dot;
Eigen::Vector3f PD;



void posDynCallback(const geometry_msgs::Vector3::ConstPtr& posD)
{
    x = posD->x;
    y = posD->y;
    z = posD->z;
    
}

void velDynCallback(const geometry_msgs::Vector3::ConstPtr& velD)
{
    x_dot = velD->x;
    y_dot = velD->y;
    z_dot = velD->z;

}

void refPosCallback(const geometry_msgs::Vector3::ConstPtr& refP)
{
    x_des = refP->x;
    y_des = refP->y;
    z_des = refP->z;
}

void refVelCallback(const geometry_msgs::Vector3::ConstPtr& refV)
{
    x_dot_des = refV->x;
    y_dot_des = refV->y;
    z_dot_des = refV->z;
}

void angDynCallback(const geometry_msgs::Vector3::ConstPtr& angD)
{
    phi_roll = angD->x;
    theta_pitch = angD->y;
    psi_yaw = angD->z;
}

void refYawCallback(const geometry_msgs::Vector3::ConstPtr& refY)
{
    psi_yaw_des = refY->x;
    psi_yaw_dot_des = refY->y;
}

int main(int argc, char **argv)
{
    // Inicializar ROS y la creación del nodo
    ros::init(argc, argv, "position_control"); // ros::init(argc, argv, "$<NOMBRE DEL ARCHIVO CPP>")
    ros::NodeHandle nh; //ros::NodeHandle $NOMBRE_DE_LA_VARIABLE_QUE_REPRESENTA_AL_NODO_EN_EL_CÓDIGO>
    
    ros::Subscriber dynamics_pos_sub = nh.subscribe("dynamics_pos", 100, &posDynCallback); 
    ros::Subscriber dynamics_vel_sub = nh.subscribe("dynamics_vel", 100, &velDynCallback);
    ros::Subscriber dynamics_ang_sub = nh.subscribe("dynamics_ang", 100, &angDynCallback);
    ros::Subscriber ref_pos_sub = nh.subscribe("ref_pos", 100, &refPosCallback);
    ros::Subscriber ref_vel_sub = nh.subscribe("ref_vel", 100, &refVelCallback);
    ros::Subscriber ref_yaw_sub = nh.subscribe("ref_yaw", 100, &refYawCallback);


    ros::Publisher posControl_pub = nh.advertise<geometry_msgs::Vector3>("posControl", 100);
    ros::Publisher posControl_errorL_pub = nh.advertise<geometry_msgs::Vector3>("posControl_errorL", 100);
    // Frecuencia a la que correrá el nodo en Hz
    ros::Rate loop_rate(100);    // ros::Rate <$NOMBRE DE LA VARIABLE>($<Hz>)

    pos_dyn << x, y, z;
    vel_dyn << x_dot, y_dot, z_dot;
    ang_dyn << phi_roll, theta_pitch, psi_yaw;
    pos_des << x_des, y_des, z_des;
    vel_des << x_dot_des, y_dot_des, z_dot_des;
    error << ex, ey, ez;
    error_dot << ex_dot, ey_dot, ez_dot;
    pos_ctrl << phi_roll_des, theta_pitch_des, th;
    yaw_des << psi_yaw_des, psi_yaw_dot_des, 0;
    PD << Uvx,Uvy,Uvz;
    kp << kpx, kpy, kpz;
    kd << kdx, kdy, kdz;
    kp_dot << kpx_dot, kpy_dot, kpz_dot;
    kd_dot << kdx_dot, kdy_dot, kdz_dot;
    while(ros::ok())
    {
        
        x = x+step*x_dot;
        y = y+step*y_dot;
        z = z+step*z_dot;
        ex = x-x_des;
        ey = y-y_des;
        ez = z-z_des;
        ex_dot = x_dot-x_dot_des;
        ey_dot = y_dot-y_dot_des;
        ez_dot = z_dot-z_dot_des;
        Uvx = -kpx*ex-kdy*ex_dot;
        Uvy = -kpy*ey-kdy*ey_dot;
        Uvz = -(kpz*ez)-(kdz*ez_dot);
        th = (m/(cos(phi_roll)*cos(theta_pitch)))*(0-g+Uvz);
        phi_roll_des = asin((m/th)*((sin(psi_yaw_des)*(Uvx))-(cos(psi_yaw_des)*(Uvy))));
        theta_pitch_des = asin((((m/th)*Uvx)-sin(psi_yaw_des)*sin(phi_roll_des))/((cos(psi_yaw_des))*cos(phi_roll_des)));
        
        geometry_msgs::Vector3 posControl_var;
        posControl_var.x = phi_roll_des;
        posControl_var.y = theta_pitch_des;
        posControl_var.z = th;

        posControl_pub.publish(posControl_var);

        geometry_msgs::Vector3 posControl_errorL_var;
        posControl_errorL_var.x = ex;
        posControl_errorL_var.y = ey;
        posControl_errorL_var.z = ez;

        posControl_errorL_pub.publish(posControl_errorL_var);

        
        ROS_INFO("DRON EN MOVIMIENTO");

        std::cout << "Roll deseado " << phi_roll_des << std::endl;
        std::cout << "Pitch deseado " << theta_pitch_des << std::endl;
        std::cout << "Uvz " << Uvz  << std::endl;
        std::cout << "Thrust " << th << "\n" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

