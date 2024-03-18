#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>


//Declaración de Funciones
void error_function();
void ang_control_function();
void tor_function();

//Definición de las variables locales generales
float step = 0.01; //Step prueba

double roll = 0;
double pitch = 0;
double yaw = 0;

double roll_des = 0;
double pitch_des = 0;
double yaw_des = 0;

double roll_vel = 0;
double pitch_vel = 0;
double yaw_vel = 0;

double roll_des_vel = 0; //0
double pitch_des_vel = 0; //0
double yaw_des_vel = 0;

double err_roll = 0;
double err_pitch = 0;
double err_yaw = 0;
double err_roll_derivada = 0;
double err_pitch_derivada = 0;
double err_yaw_derivada = 0;

double roll_ang = 0;
double pitch_ang = 0;
double yaw_ang = 0;

double u_roll = 0;
double u_pitch = 0;
double u_yaw = 0;

double kp_roll = 2;
double kp_pitch = 4;
double kp_yaw = 2;
double kd_roll = 3;
double kd_pitch = 2;
double kd_yaw = 4;

const double Jxx = 0.0418;
const double Jyy = 0.0478;
const double Jzz = 0.0599;

//Published Variables
double tor_roll = 0;
double tor_pitch = 0;
double tor_yaw = 0;


//Callback Functions
void posCallback(const geometry_msgs::Vector3::ConstPtr& c) //Viene del Nodo Pos
{
    roll_des = c->x;
    pitch_des = c->y;
}

void yawCallback(const geometry_msgs::Vector3::ConstPtr& c) //Viene del Nodo Ref
{
    yaw_des = c->x;
    yaw_des_vel = c->y;
}

void angCallback(const geometry_msgs::Vector3::ConstPtr& c) //Viene del Nodo UAV
{
    roll = c->x;
    pitch = c->y;
    yaw = c->y;
}

void velangCallback(const geometry_msgs::Vector3::ConstPtr& c) //Viene del Nodo UAV 
{
    roll_vel = c->x;
    pitch_vel = c->y;
    yaw_vel = c->y;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "attitude_control"); // ros::init(argc, argv, "$<NOMBRE DEL ARCHIVO CPP>")
    ros::NodeHandle nh; //ros::NodeHandle $NOMBRE_DE_LA_VARIABLE_QUE_REPRESENTA_AL_NODO_EN_EL_CÓDIGO>
    
    // Declaración de los publishers
    //ros::Publisher <$PUBLISHER_NAME> = nh.advertise<$TIPO_DE_MENSAJE>("<$TÓPICO>", 
    //<$NÚMERO_DE_DATOS_A_GUARDAR_EN_CASO_DE_TERMINAR_EL_NODO>)
    ros::Publisher attitude_torq_pub = nh.advertise<geometry_msgs::Vector3>("attitudeControl", 100);
    ros::Subscriber pose_sub = nh.subscribe("posControl", 100, &posCallback);
    ros::Subscriber yaw_sub = nh.subscribe("ref_yaw", 100, &yawCallback);
    ros::Subscriber ang_sub = nh.subscribe("dynamics_ang", 100, &angCallback);
    ros::Subscriber velang_sub = nh.subscribe("dynamics_velang", 100, &velangCallback);

    // Frecuencia a la que correrá el nodo en Hz
    ros::Rate loop_rate(100);    // ros::Rate <$NOMBRE DE LA VARIABLE>($<Hz>)

    // Definición de las variables que identifican al mensaje de ROS
    //El mensaje Vector3 es un vector (3,1) que permite enviar varias variables en un mismo mensaje 
    geometry_msgs::Vector3 attitude_torq_var;
    
    while(ros::ok())
    {
        //Llamada a las funciones
        error_function();
        ang_control_function();
        tor_function();

        //Guardar valores calculados en variables para ser publicados
        attitude_torq_var.x = tor_roll;
        attitude_torq_var.y = tor_pitch;
        attitude_torq_var.z = tor_yaw;

        //Publicar valores
        attitude_torq_pub.publish(attitude_torq_var);

        ros::spinOnce();
        loop_rate.sleep();
         
    }

    return 0;
}

//Definición de Funciones
void error_function()
{
    err_roll = roll_des - roll;
    err_pitch = pitch_des - pitch;
    err_yaw = yaw_des - yaw;

    err_roll_derivada = roll_des_vel - roll_vel;
    err_pitch_derivada = pitch_des_vel - pitch_vel;
    err_yaw_derivada = yaw_des_vel - yaw_vel;
}

void ang_control_function()
{
    u_roll = kp_roll*err_roll + kd_roll*err_roll_derivada;
    u_pitch = kp_pitch*err_pitch + kd_pitch*err_pitch_derivada;
    u_yaw = kp_yaw*err_yaw + kd_yaw*err_yaw_derivada;
}

void tor_function()
{
    tor_roll = Jxx*(((Jzz-Jyy)/Jxx)*(pitch_vel*yaw_vel)+u_roll);
    tor_pitch = Jxx*(((Jzz-Jyy)/Jxx)*(roll_vel*yaw_vel)+u_pitch);
    tor_yaw = Jxx*(((Jzz-Jyy)/Jxx)*(roll_vel*pitch_vel)+u_yaw);
}

