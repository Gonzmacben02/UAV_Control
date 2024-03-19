#include <ros/ros.h> 
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

float masa = 2;
float g = 9.81;
float step = 0.01;
float Th = -(masa * g); //Thrust
float Jxx = 0.0411;
float Jyy = 0.0478;
float Jzz = 0.0599;

Eigen::Vector3f w;
Eigen::Vector3f w_punto;
Eigen::Vector3f att;
Eigen::Vector3f att_punto;
Eigen::Vector3f fuerza(0,0,0);
Eigen::Vector3f acel_linear_loc;
Eigen::Vector3f vel_linear_loc;
Eigen::Vector3f vel_linear_in;
Eigen::Vector3f posicion_linear;
Eigen::Vector3f torques;

Eigen::Vector3f e3(0,0,1);
Eigen::Matrix3f J;

Eigen::Matrix3f asimetrica(){
    Eigen::Matrix3f sk;
    sk << 0, -w(2), w(1), 
        w(2), 0, -w(0),
        -w(1), w(0), 0;
    return sk;
}

Eigen::Matrix3f rotacional(){
    Eigen::Matrix3f r;
    r << cos(att(2))*cos(att(1)), cos(att(2))*sin(att(0))*sin(att(1)) - cos(att(0))*sin(att(2)), sin(att(2))*sin(att(0)) + cos(att(2))*cos(att(0))*sin(att(1)),
        cos(att(1))*sin(att(2)), cos(att(2))*cos(att(0)) + sin(att(2))*sin(att(0))*sin(att(1)), cos(att(0))*sin(att(2))*sin(att(1)) - cos(att(2))*sin(att(0)),
        -sin(att(1)), cos(att(1))*sin(att(0)), cos(att(0))*cos(att(1));
    return r;
}

Eigen::Matrix3f rDos(){
    Eigen::Matrix3f r2;
    r2 << 1, sin(att(0))*tan(att(1)), cos(att(0))*tan(att(1)),
            0, cos(att(0)), -sin(att(0)), 
            0, sin(att(0))/cos(att(1)), cos(att(0))/cos(att(1));
    return r2;
}

        
void posControlCallBack(const geometry_msgs::Vector3::ConstPtr& thrust){
    Th = thrust->z;
}

void angControlCallBack(const geometry_msgs::Vector3::ConstPtr& trq){
    torques(0) = trq->x;
    torques(1) = trq->y;
    torques(2) = trq->z;
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

    posicion_linear << -5, 0, 0;
    w << 0, 0, 0;
    w_punto << 0, 0, 0;
    att << 0, 0, 0;
    att_punto << 0, 0, 0;

    vel_linear_loc << 0, 0, 0;
    vel_linear_in << 0, 0, 0;
    vel_linear_in << 0, 0, 0;

    acel_linear_loc << 0, 0, 0;

    pos_var.x = posicion_linear(0);
    pos_var.y = posicion_linear(1);
    pos_var.z = posicion_linear(2);

    vel_var.x = vel_linear_in(0);
    vel_var.y = vel_linear_in(1);
    vel_var.z = vel_linear_in(2);

    ang_var.x = att(0);
    ang_var.y = att(1);
    ang_var.z = att(2);

    velang_var.x = att_punto(0);
    velang_var.y = att_punto(1);
    velang_var.z = att_punto(2);

    pos_pub.publish(pos_var);
    vel_pub.publish(vel_var);
    ang_pub.publish(ang_var);
    velang_pub.publish(velang_var);

    while(ros::ok()){

        //Angular
        w_punto = J.inverse()*(torques - (asimetrica() * J * w));

        w(0) = w(0) + step * w_punto(0);
        w(1) = w(1) + step * w_punto(1);
        w(2) = w(2) + step * w_punto(2);

        att_punto = rDos() * w;

        att(0) = att(0) + step * att_punto(0);
        att(1) = att(1) + step * att_punto(1);
        att(2) = att(2) + step * att_punto(2);

        //Lineal
        fuerza = (Th * e3) + (rotacional().transpose() * (masa * g * e3));
        
        acel_linear_loc = fuerza/masa - asimetrica() * vel_linear_loc;

        vel_linear_loc(0) = vel_linear_loc(0) + step * acel_linear_loc(0);
        vel_linear_loc(1) = vel_linear_loc(1) + step * acel_linear_loc(1);
        vel_linear_loc(2) = vel_linear_loc(2) + step * acel_linear_loc(2);

        vel_linear_in = rotacional() * vel_linear_loc;

        posicion_linear(0) = posicion_linear(0) + step * vel_linear_in(0);
        posicion_linear(1) = posicion_linear(1) + step * vel_linear_in(1);
        posicion_linear(2) = posicion_linear(2) + step * vel_linear_in(2);

        pos_var.x = posicion_linear(0);
        pos_var.y = posicion_linear(1);
        pos_var.z = posicion_linear(2);

        vel_var.x = vel_linear_in(0);
        vel_var.y = vel_linear_in(1);
        vel_var.z = vel_linear_in(2);

        ang_var.x = att(0);
        ang_var.y = att(1);
        ang_var.z = att(2);

        velang_var.x = att_punto(0);
        velang_var.y = att_punto(1);
        velang_var.z = att_punto(2);

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