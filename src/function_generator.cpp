
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "string"
#include "math.h"
#include <time.h>

// Namespaces
using namespace std;

#define FREQ 200.0

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
	SignalGenerator(string type, double a, double f, double off, double d);

	string signal;
	double ampl;
	double freq;
	double offset;
	double dead;

	double output(double t);

private:
	bool switch_;
	double value_;
	double x_;
	int cnt;
};
SignalGenerator::SignalGenerator(string type, double a, double f, double off, double d)
{
	signal = type;
	ampl = a;
	freq = f;
	offset = off;
	dead = d;
	switch_ = true;
	value_ = 0;
	cnt = 0;
	x_ = 1;
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
		if (fmod(cnt,FREQ/2/freq) <1.0)
		{x_ = x_ * -1;}
		
		value = ampl*x_;
		cnt++;
	}
	else if (signal =="2f")
	{

		if (fmod(cnt,FREQ*5/freq)<1.0)
		{
			switch_= !switch_;
		}

		if (switch_)
		{
			if (fmod(cnt,FREQ/2/freq) <1.0)
			{x_ = x_ * -1;}
			value = ampl*x_;
		}
		else
		{
			if (fmod(cnt,FREQ/4/freq) <1.0)
			{x_ = x_ * -1;}
			value = ampl*x_;
		}

		cnt++;
	}

	else if (signal == "ramp" || signal =="Ramp")
	{
		value = fmod(t,freq)*ampl;
	}

	else if (signal == "stair")
	{

		if (fmod(cnt,FREQ/freq) < 1.0)
		{		
			value_ += ampl;
		}
		if (fabs(value_) > 2.0)
		{
			ampl = -ampl;
			value_ += 2*ampl;
		}
		value = value_;
		cnt++;
	}

	else
	{
		printf("Wrong signal name!!\n");
		value=0;
	}

	value += offset;
	if(value>=0.0)
	{
		value += dead;
	}
	else
	{
		value -= dead;
	}

	
	return value;
}


class Output
{

public:
	Output();
	sensor_msgs::JointState joint_state;
	void OutputCallback(sensor_msgs::JointState output_);
};

Output::Output()
{
	joint_state.name.resize(7);
	joint_state.position.resize(7);
	joint_state.velocity.resize(7);
	joint_state.effort.resize(7);
}
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
    ros::Subscriber postion_sub = n.subscribe("/davinci/joint_states", 1, &Output::OutputCallback, &output);
    ros::Publisher signal_pub = n.advertise<std_msgs::Float64MultiArray>("davinci_si/input",1);
    ros::Publisher setpoint_pub = n.advertise<sensor_msgs::JointState>("davinci_joystick/joint_states",1);
    ros::Publisher roll_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_roll_controller/command",1);
    ros::Publisher pitch_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_pitch_controller/command",1);
    ros::Publisher jawl_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_jaw_left_controller/command",1);
    ros::Publisher jawr_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_jaw_right_controller/command",1);


    ros::Rate rate(FREQ);

    sensor_msgs::JointState setpoint;
    std_msgs::Float64MultiArray msg;
    msg.data.resize(4);

    std_msgs::Float64 I_roll;
    std_msgs::Float64 I_pitch;
    std_msgs::Float64 I_jawl;
    std_msgs::Float64 I_jawr;

    I_roll.data =0;
    I_pitch.data =0;
    I_jawl.data =0;
    I_jawr.data =0;

    StopWatch stopwatch;

    char str[10];
    double f,a,o,d;
    printf("Signal :");
    scanf("%s",str);
    printf("amplitude frequency offset dead_zone: ");
    scanf("%lf %lf %lf %lf",&a,&f,&o,&d);

    SignalGenerator input_signal(str, a, f, o, d);
    

    double value =0;
    double t;
	
	double idead =d;
    stopwatch.Reset();

    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
    	t = stopwatch.elapsed_time();
    	value = input_signal.output(t);

	
	I_roll.data = value;

    	/*msg.data[0] = t;
    	msg.data[1] = value;
	msg.data[2] = output.joint_state.position[4];
	msg.data[3] = output.joint_state.effort[4];
    	signal_pub.publish(msg);*/

		

    	roll_pub.publish(I_roll);
    	//pitch_pub.publish(I_pitch);
    	//jawl_pub.publish(I_jawl);
    	//jawr_pub.publish(I_jawr);

    	/*setpoint.header.stamp = ros::Time::now();
    	setpoint.name.resize(4);
    	setpoint.position.resize(4);
    	setpoint.effort.resize(4);
    	setpoint.name[0] ="pitch";
    	setpoint.position[0] = value;
    	setpoint.name[1] ="yaw";
    	setpoint.position[1] = value;
    	setpoint.name[2] ="roll";
    	setpoint.position[2] = value;
    	setpoint.effort[2]=output.joint_state.position[2];
    	setpoint.name[3]="pinch";
    	setpoint.position[3]=value;
        //send the joint state
    	setpoint_pub.publish(setpoint);*/


    	ros::spinOnce();
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
    }

    I_roll.data =0;
    I_pitch.data =0;
    I_jawl.data =0;
    I_jawr.data =0;

    	roll_pub.publish(I_roll);
    	pitch_pub.publish(I_pitch);
    	jawl_pub.publish(I_jawl);
    	jawr_pub.publish(I_jawr);

    return 0;
}

