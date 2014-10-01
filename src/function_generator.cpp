
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "string"
#include "math.h"
#include <time.h>


// Namespaces
using namespace std;

#define FREQ 1000

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


int main(int argc, char **argv)
{
	// ROS initialization
    ros::init(argc, argv, "function_generator");
    ros::NodeHandle n;
    ros::Publisher signal_pub = n.advertise<std_msgs::Float64MultiArray>("davinci_si/input",1);
    ros::Rate rate(FREQ);

    std_msgs::Float64MultiArray msg;
    msg.data.resize(2);

    StopWatch stopwatch;

    char str[10];
    double f,a;

    printf("Enter signal: ");
    scanf("%s",str);
    printf("Frequency: ");
    scanf("%lf",&f);
    printf("Amplitude :");
    scanf("%lf",&a);

    SignalGenerator input_signal(str,a,f);
    double value;
    double t;

    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
    	t = stopwatch.elapsed_time();
    	value = input_signal.output(stopwatch.elapsed_time());

    	msg.data[0] = t;
    	msg.data[1] = value;
    	signal_pub.publish(msg);

        rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
    }

    return 0;
}

