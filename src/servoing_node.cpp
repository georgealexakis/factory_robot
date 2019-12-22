/*
 *
 * Project Name: Factory Robot
 * Description: This is an implementation of simple visual servoing with the use of visp package in robot.
 * 
 */

#include "stdio.h"
#include "stdlib.h"
#include "sstream"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

class ServoingRobot
{
private:
	ros::Publisher movementPublisher;   // Sends Twist messages
	ros::Publisher joystickPublisher;   // Sends commands to the connected applications
	ros::Subscriber commandSubscriber;  // Receives the message from QR codes
	ros::Subscriber positionSubscriber; // Starts visual servoing
	ros::Subscriber statusSubscriber;   // Checks if exist QR code tag
	bool servoing;						// Enables servoing if a QR code tag is detected
	bool servoingQR;					// Enables moving if specific QR code tag is detected

public:
	ServoingRobot()
	{
		ros::NodeHandle n;

		// Initializes publishers and subscribers
		movementPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
		joystickPublisher = n.advertise<std_msgs::String>("/joystick/diagnostics", 10);
		commandSubscriber = n.subscribe("/visp_auto_tracker/code_message", 10, &ServoingRobot::commandCallback, this);
		statusSubscriber = n.subscribe("/visp_auto_tracker/status", 10, &ServoingRobot::statusCallback, this);
		positionSubscriber = n.subscribe("/visp_auto_tracker/object_position", 10, &ServoingRobot::positionCallback, this);

		// Initializes global variables
		servoing = false;
		servoingQR = false;

		// Displays ready message
		ROS_INFO("\n******** Servoing Robot Node ***********\n");
	}

	// Callback for QR tag detection (avoids conflict with other QR code tags)
	void commandCallback(const std_msgs::String::ConstPtr &msg)
	{
		if (msg->data == "qr5")
		{
			servoingQR = true;
			diagnostics("servoing", -1);
		}
		else
		{
			servoingQR = false;
		}
	}

	// Enables visual servoing using /visp_auto_tracker/object_position topic
	void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		geometry_msgs::Twist twistMsg;
		twistMsg.linear.y = 0.0;
		twistMsg.linear.z = 0.0;
		twistMsg.angular.x = 0.0;
		twistMsg.angular.y = 0.0;
		if (servoing && servoingQR)
		{
			float distance = msg->pose.position.z;
			float position = msg->pose.position.x;

			// Sets a rest threshold to avoid moving all the time
			// Converts the distance values (not real distance) of the tag from the camera to linear velocity
			if (distance < 0.18)
				twistMsg.linear.x = -0.2;
			else if (distance > 0.23)
				twistMsg.linear.x = distance;
			else
				twistMsg.linear.x = 0.0;

			// Sets a rest threshold to avoid rotating all the time
			// Converts the distance values from center (not real distance) of the tag from the camera to angular velocity
			if (position < -0.05)
				twistMsg.angular.z = -position * 10;
			else if (position > 0.05)
				twistMsg.angular.z = -position * 10;
			else
				twistMsg.angular.z = 0.0;
			movementPublisher.publish(twistMsg);
		}
	}

	// Checks if any QR code exists and start servoing
	void statusCallback(const std_msgs::Int8::ConstPtr &msg)
	{
		int status = msg->data;
		if (status != 1)
		{
			servoing = true;
		}
		else
		{
			servoing = false;
		}
	}

	// Sends diagnostics messages to the Applications
	void diagnostics(string msg, int num)
	{
		stringstream ss;
		std_msgs::String joyDiagnostics;
		if (num != -1)
			ss << msg << num;
		else
			ss << msg;
		joyDiagnostics.data = ss.str();
		joystickPublisher.publish(joyDiagnostics);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servoing_node");
	ServoingRobot robot;
	ros::spin();
	return 0;
}
