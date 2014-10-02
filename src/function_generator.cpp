
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "string"
#include "math.h"
#include <time.h>

// Namespaces
using namespace std;

#define FREQ 200

class StopWatch
{
public:
	StopWatch();
	double _start_time;
	double elapsed_time(void);
	void Reset(void);

private:

};
StopWatch::StopWatch()
{
	_start_time = ros::Time::now().toNSec();
}
double StopWatch::elapsed_time(void)
{
	double t;
	t = (ros::Time::now().toNSec()-_start_time)/1000000000.0;
	return t;
}
void StopWatch::Reset(void)
{
	_start_time = ros::Time::now().toNSec();
}


class SignalGenerator
{
public:
	SignalGenerator(string type,double amplitude,double frequency);

	string signal;
	double ampl;
	double freq;

	double output(double t);

private:

};
SignalGenerator::SignalGenerator(string type,double amplitude,double frequency)
{
	ampl = amplitude;
	freq = frequency;
	signal = type;
}
double SignalGenerator::output(double t)
{
	double value;
	if(signal=="Sinus" || signal=="sinus")
	{
		value = ampl*cos(2*M_PI*freq*t);
	}
	else if(signal == "Constant" || signal=="constant")
	{
		value = ampl;
	}
	else if (signal == "Block" || signal=="block")
	{
		if (sin(2*M_PI*freq*t)>=0)
		{value = ampl;}
		else
		{value=-ampl;}
	}
	else
	{
		printf("Wrong signal name!!\n");
		value=0;
	}

	return value;
}


class Output
{

public:
	sensor_msgs::JointState joint_state;
	void OutputCallback(sensor_msgs::JointState output_);
};

void Output::OutputCallback(sensor_msgs::JointState output_)
{
	joint_state = output_;
}



int main(int argc, char **argv)
{
	// ROS initialization

	Output output;

    ros::init(argc, argv, "function_generator");
    ros::NodeHandle n;
    ros::Publisher signal_pub = n.advertise<std_msgs::Float64MultiArray>("davinci_si/input",1);
    ros::Publisher setpoint_pub = n.advertise<sensor_msgs::JointState>("davinci_joystick/joint_states",1);
    ros::Publisher s_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_roll_controller/command",1);

    ros::Subscriber postion_sub = n.subscribe("/davinci/joint_states", 1, &Output::OutputCallback, &output);
    ros::Rate rate(FREQ);

    sensor_msgs::JointState setpoint;
    std_msgs::Float64MultiArray msg;
    msg.data.resize(2);

    std_msgs::Float64 Iset;

    StopWatch stopwatch;

    char str[10];
    double f,a;
    printf("Enter signal: ");
    scanf("%s",str);
    printf("Frequency: ");
    scanf("%lf",&f);
    printf("Amplitude: ");
    scanf("%lf",&a);

    SignalGenerator input_signal(str,a,f);

    double signal =0;
    double t;

    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
    	t = stopwatch.elapsed_time();
    	signal = input_signal.output(t);

    	msg.data[0] = t;
    	msg.data[1] = signal;
    	signal_pub.publish(msg);

	Iset.data =signal;
	s_pub.publish(Iset);

    	setpoint.header.stamp = ros::Time::now();
    	setpoint.name.resize(4);
    	setpoint.position.resize(4);
    	setpoint.name[0] ="pitch";
    	setpoint.position[0] = signal;
    	setpoint.name[1] ="yaw";
    	setpoint.position[1] = signal;
    	setpoint.name[2] ="roll";
    	setpoint.position[2] = signal;
    	setpoint.name[3]="pinch";
    	setpoint.position[3]=signal;
        //send the joint state
    	setpoint_pub.publish(setpoint);


    	ros::spinOnce();
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
    }
	Iset.data=0;
	s_pub.publish(Iset);
    return 0;
}

