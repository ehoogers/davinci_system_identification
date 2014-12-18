
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "string"
#include "math.h"
#include <time.h>

// Namespaces
using namespace std;

#define FREQ 100.0

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
	SignalGenerator(string type, double a, double f, double off);

	string signal;
	double ampl;
	double freq;
	double offset;

	double output(double t);

private:
	bool switch_;
	double value_;
	double x_;
	int cnt;
};
SignalGenerator::SignalGenerator(string type, double a, double f, double off)
{
	signal = type;
	ampl = a;
	freq = f;
	offset = off;
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
		value = ampl*sin(2*M_PI*freq*t);
	}

	else if(signal=="Multisine" || signal=="ms")
	{
		double y1 = ampl*sin(2*M_PI*freq*t);
		double y2 = ampl*sin(2*M_PI*freq*t*2)/2;
		double y3 = ampl*sin(2*M_PI*freq*t*5)/5;
		double y4 = ampl*sin(2*M_PI*freq*t*8)/8;
		double y5 = ampl*sin(2*M_PI*freq*t*10)/10;
		value = y1 + y2 + y3 + y4 + y5;
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

		if (cnt == 60*FREQ)
		{
			cnt = 0;
		}
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
		
		if (cnt == 60*FREQ)
		{
			cnt = 0;
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
		if (fabs(value_) > 1.3)
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
    ros::Publisher slide_pub = n.advertise<std_msgs::Float64>("/davinci/p4_instrument_slide_controller/command",1);
    ros::Publisher hand_roll_pub = n.advertise<std_msgs::Float64>("/davinci/p4_hand_roll_controller/command",1);
    ros::Publisher hand_pitch_pub = n.advertise<std_msgs::Float64>("/davinci/p4_hand_pitch_controller/command",1);
    ros::Publisher joystick_sp_pub = n.advertise<std_msgs::Float64MultiArray>("davinci_joystick/I_sp",1);

    ros::Rate rate(FREQ);

    sensor_msgs::JointState setpoint;
    std_msgs::Float64MultiArray msg;
    msg.data.resize(5);

    std_msgs::Float64 I_roll;
    std_msgs::Float64 I_pitch;
    std_msgs::Float64 I_jawl;
    std_msgs::Float64 I_jawr;
    std_msgs::Float64 I_slide;
    std_msgs::Float64 I_hand_roll;
    std_msgs::Float64 I_hand_pitch;


    std_msgs::Float64MultiArray joystick_sp;
    joystick_sp.data.resize(4);

joystick_sp.data[0] = 0;
joystick_sp.data[1] = 0;
joystick_sp.data[2] = 0;
joystick_sp.data[3] = 0;

    I_roll.data =0;
    I_pitch.data =0;
    I_jawl.data =0;
    I_jawr.data =0;
    I_slide.data =0;
    I_hand_roll.data =0;
    I_hand_pitch.data =0;

    StopWatch stopwatch;

    char str[10];
    double f,a,o;
    printf("Signal :");
    scanf("%s",str);
    printf("amplitude frequency offset: ");
    scanf("%lf %lf %lf",&a,&f,&o);

    SignalGenerator input_signal(str, a, f, o);
    

    double value =0;
    double t;
    stopwatch.Reset();

    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
    	t = stopwatch.elapsed_time();
    	value = input_signal.output(t);

	joystick_sp.data[0] = 0; // value;
	joystick_sp.data[1] = 0; // value;
	joystick_sp.data[2] = 0; // value;
	joystick_sp.data[3] = 0; //value;

	
	I_jawl.data =0; // value;
	I_jawr.data = 0; //value;
	I_roll.data = value;
	I_pitch.data = 0; //value;
	I_slide.data = 0; //value;
	I_hand_roll.data = 0; //value;
	I_hand_pitch.data = 0; //value;

    	msg.data[0] = t;
    	msg.data[1] = value;
	msg.data[2] = output.joint_state.position[5];
	msg.data[3] = output.joint_state.velocity[5];
	msg.data[4] = output.joint_state.effort[5];
    	signal_pub.publish(msg);
		

    	roll_pub.publish(I_roll);
    	//pitch_pub.publish(I_pitch);
    	//jawl_pub.publish(I_jawl);
    	//jawr_pub.publish(I_jawr);
	//slide_pub.publish(I_slide);
	//hand_roll_pub.publish(I_hand_roll);
	//hand_pitch_pub.publish(I_hand_pitch);
	//joystick_sp_pub.publish(joystick_sp);


    	ros::spinOnce();
        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
    }

    I_roll.data =0;
    I_pitch.data =0;
    I_jawl.data =0;
    I_jawr.data =0;
    I_hand_roll.data =0;
    I_hand_pitch.data =0;

    	roll_pub.publish(I_roll);
    	pitch_pub.publish(I_pitch);
    	jawl_pub.publish(I_jawl);
    	jawr_pub.publish(I_jawr);
	slide_pub.publish(I_slide);
	hand_roll_pub.publish(I_slide);
	hand_pitch_pub.publish(I_slide);
    return 0;
}

