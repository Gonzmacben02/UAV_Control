#include <ros/ros.h> 
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

float t_roll = 0; //Torques
float t_pitch = 0;
float t_yaw = 0;
float phi = 0;
float theta = 0;
float psi = 0;
//float phi_punto;
//float theta_punto;
//float psi_punto;
float Jxx = 0.0411;
float Jyy = 0.0478;
float Jzz = 0.0599;
float wx = 0;
float wy = 0;
float wz = 0;
float masa = 2;
float g = 9.81;
float step = 1.0/100.0;
float Th = masa * g; //Thrust

// Vector3f w_I;
// w_I << cos(psi)*cos(theta)*phi_punto - sin(psi)*theta_punto, sin(psi)*cos(theta)*phi_punto + cos(psi)*theta_punto, psi_punto - sin(theta)*phi_punto;

Eigen::Matrix3f J;

Eigen::Vector3f omega;

Eigen::Vector3f posicion_linear;

Eigen::Matrix3f sk;

Eigen::Vector3f vel_linear_loc; //Se;or internet

Eigen::Vector3f vel_linear_in;

Eigen::Vector3f e3;

// Eigen::Vector3f posicion_punto;
// posicion_punto << 0, 0, 0;

Eigen::Vector3f w;

Eigen::Vector3f w_punto;

Eigen::Vector3f torques;

Eigen::Vector3f fuerza;

Eigen::Vector3f acel_linear_loc;

Eigen::Matrix3f rotacional;

        
void posControlCallBack(const geometry_msgs::Vector3::ConstPtr& thrust){
    Th = thrust->z;
}

void angControlCallBack(const geometry_msgs::Vector3::ConstPtr& trq){
    t_roll = trq->x;
    t_pitch = trq->y;
    t_yaw = trq->z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamics");
    ros::NodeHandle nh;

    ros::Subscriber posControl_sub = nh.subscribe("posControl", 100, &posControlCallBack);
    ros::Subscriber angControl_sub = nh.subscribe("attitudeControl", 100, &angControlCallBack);

    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Vector3>("dynamics_pos",100);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Vector3>("dynamics_vel",100);
    ros::Publisher ang_pub = nh.advertise<geometry_msgs::Vector3>("dynamics_ang",100);
    ros::Publisher velang_pub = nh.advertise<geometry_msgs::Vector3>("dynamics_velang",100);


    ros::Rate loop_rate(100);

    geometry_msgs::Vector3 pos_var;
    geometry_msgs::Vector3 vel_var;
    geometry_msgs::Vector3 ang_var;
    geometry_msgs::Vector3 velang_var;

    J << Jxx, 0, 0, 
        0, Jyy, 0, 
        0, 0, Jzz;

    omega << 0, 0, 0;

    posicion_linear << -5, 0, 0;

    sk << 0, -wz, wy, 
        wz, 0, -wx,
        -wy, wx, 0;

    vel_linear_loc << 0, 0, 0;

    vel_linear_in << 0, 0, 0;

    vel_linear_in << 0, 0, 0;

    e3 << 0, 0, 1;

    w << wx, wy, wz;

    w_punto << 0, 0, 0;

    torques << t_roll, t_pitch, t_yaw;

    fuerza << 0, 0, 0;

    acel_linear_loc << 0, 0, 0;

    rotacional << cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta),
                cos(theta)*sin(psi), cos(psi)*cos(phi) + sin(psi)*sin(phi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi),
                -sin(theta), cos(phi), cos(phi)*cos(theta);

    while(ros::ok()){
        //Angular
        w_punto = J.inverse()*(torques - sk * J * w);

        w(0) = w(0) + step * w_punto(0);
        w(1) = w(1) + step * w_punto(1);
        w(2) = w(2) + step * w_punto(2);

        phi = phi + step * w(0);
        theta = theta + step * w(1);
        psi = psi + step * w(2);

        //Lineal
        fuerza = Th * e3 + rotacional.transpose() * (masa * g * e3);
        
        acel_linear_loc = fuerza/masa - sk * vel_linear_loc;

        vel_linear_loc(0) = vel_linear_loc(0) + step * acel_linear_loc(0);
        vel_linear_loc(1) = vel_linear_loc(1) + step * acel_linear_loc(1);
        vel_linear_loc(2) = vel_linear_loc(2) + step * acel_linear_loc(2);

        vel_linear_in = rotacional.inverse() * vel_linear_loc;

        posicion_linear(0) = posicion_linear(0) + step * vel_linear_in(0);
        posicion_linear(1) = posicion_linear(1) + step * vel_linear_in(1);
        posicion_linear(2) = posicion_linear(2) + step * vel_linear_in(2);

        pos_var.x = posicion_linear(0);
        pos_var.y = posicion_linear(1);
        pos_var.z = posicion_linear(2);

        vel_var.x = vel_linear_in(0);
        vel_var.y = vel_linear_in(1);
        vel_var.z = vel_linear_in(2);

        ang_var.x = phi;
        ang_var.y = theta;
        ang_var.z = psi;

        velang_var.x = w(0);
        velang_var.y = w(1);
        velang_var.z = w(2);

        pos_pub.publish(pos_var);
        vel_pub.publish(vel_var);
        ang_pub.publish(ang_var);
        velang_pub.publish(velang_var);

        ROS_INFO("Funciona la dinamica (se supone)");

        ros::spinOnce();

        loop_rate.sleep(); 
        
    }
    return 0;
}  