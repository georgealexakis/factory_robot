/*
 *
 * Project Name: Factory Robot
 * Description: This is an implementation of an autonomous navigated robot, that also can avoid obstacles, through QR code tag detection.
 * 
 */

#include "stdio.h"
#include "stdlib.h"
#include "sstream"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "kobuki_msgs/Led.h"
#include "kobuki_msgs/Sound.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class FactoryRobot
{
private:
	ros::Publisher movementPublisher;		// Sends Twist messages
	ros::Publisher led1Publisher;			// Sends led messages
	ros::Publisher led2Publisher;			// Sends led messages
	ros::Publisher soundPublisher;			// Sends sound messages
	ros::Publisher joystickPublisher;		// Sends command to the connected applications
	ros::Subscriber commandSubscriber;		// Receives the message from QR codes
	ros::Subscriber joystickSubscriber;		// Receives messages from the connected applications
	bool exCommand;							// Allows execution of only one command
	int moveCallbackCounter;				// Goal attemps counter
	static const double PI = 3.14159265359; // Global PI variable
	int redefineCounter;					// Redefines counter to avoids multiple redefine procedures
	vector<double> coordinatesP1;			// Coordinates for position 1
	vector<double> coordinatesP2;			// Coordinates for position 2
	vector<double> coordinatesP3;			// Coordinates for position 3
	vector<double> coordinatesP4;			// Coordinates for position 4
	string tagP1;							// QR code tag data for position 1
	string tagP2;							// QR code tag data for position 2
	string tagP3;							// QR code tag data for position 3
	string tagP4;							// QR code tag data for position 4

public:
	FactoryRobot()
	{
		ros::NodeHandle n;

		// Initializes publishers and subscribers
		movementPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
		led1Publisher = n.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 10);
		led2Publisher = n.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 10);
		soundPublisher = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 10);
		joystickPublisher = n.advertise<std_msgs::String>("/joystick/diagnostics", 10);
		commandSubscriber = n.subscribe("/visp_auto_tracker/code_message", 10, &FactoryRobot::commandCallback, this);
		joystickSubscriber = n.subscribe("/joystick/command", 10, &FactoryRobot::joystickCallback, this);

		// Initializes global variables
		exCommand = false;
		moveCallbackCounter = 0;
		redefineCounter = 1;

		// Get coordinates from parameters file
		n.getParam("/position1", tagP1);
		n.getParam("/position2", tagP2);
		n.getParam("/position3", tagP3);
		n.getParam("/position4", tagP4);
		n.getParam("/coordinates1", coordinatesP1);
		n.getParam("/coordinates2", coordinatesP2);
		n.getParam("/coordinates3", coordinatesP3);
		n.getParam("/coordinates4", coordinatesP4);

		// Displays ready message
		ROS_INFO("\n******** Factory Robot Node ***********\n");
	}

	// Callback for QR tag detection
	void commandCallback(const std_msgs::String::ConstPtr &msg)
	{
		executeCommand(msg);
	}

	void executeCommand(const std_msgs::String::ConstPtr &msg)
	{
		// Command chooser
		if (!exCommand)
		{
			if (msg->data == tagP1)
			{
				setGoal(coordinatesP1.at(0), coordinatesP1.at(1), coordinatesP1.at(2), coordinatesP1.at(3)); // 1st position (initial position)
			}
			else if (msg->data == tagP2)
			{
				setGoal(coordinatesP2.at(0), coordinatesP2.at(1), coordinatesP2.at(2), coordinatesP2.at(3)); // 2nd position
			}
			else if (msg->data == tagP3)
			{
				setGoal(coordinatesP3.at(0), coordinatesP3.at(1), coordinatesP3.at(2), coordinatesP3.at(3)); // 3rd position
			}
			else if (msg->data == tagP4)
			{
				setGoal(coordinatesP4.at(0), coordinatesP4.at(1), coordinatesP4.at(2), coordinatesP4.at(3)); // 4th position
			}
		}
	}

	// Callback from joystick
	void joystickCallback(const std_msgs::String::ConstPtr &msg)
	{
		executeCommand(msg);
	}

	// Generates sound for different purposes
	void generateSound(int sound)
	{
		kobuki_msgs::Sound soundMsg;
		soundMsg.value = sound;
		soundPublisher.publish(soundMsg);
	}

	// Changes led states of turtlebot
	void changeLedState(int led, int state)
	{
		kobuki_msgs::Led ledMsg;
		ledMsg.value = state;
		if (led == 1)
		{
			led1Publisher.publish(ledMsg);
		}
		else
		{
			led2Publisher.publish(ledMsg);
		}
	}

	// Feedback after success goal
	void goalFeedback(int sound, int led1, int led2)
	{
		generateSound(sound);
		changeLedState(1, led1);
		changeLedState(2, led2);
	}

	// Sends diagnostics messages to Web Application
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

	// Sets a simple goal for the robot in the map
	void setGoal(float x, float y, float z, float w)
	{
		goalFeedback(5, 2, 2);
		exCommand = true;
		redefineCounter = 0;

		MoveBaseClient ac("move_base", true);
		while (!ac.waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("\n******** Waiting Action Server ***********\n");
		}
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = x;
		goal.target_pose.pose.position.y = y;
		goal.target_pose.pose.orientation.z = z;
		goal.target_pose.pose.orientation.w = w;

		ac.sendGoal(goal);
		ac.waitForResult();

		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			// Goal reached and then starts to search the QR tag
			goalFeedback(6, 3, 3);
			diagnostics("goal reached, c: ", moveCallbackCounter);
			moveCallbackCounter = 0;
			exCommand = false; // Allows QR detection
			redefinePosition();
			redefineCounter++; // Prevents multiple redefine procedures
		}
		else if (moveCallbackCounter > 0)
		{
			// Aborts all attempts and stops the movement
			goalFeedback(6, 3, 3);
			diagnostics("all aborted, c: ", moveCallbackCounter);
			moveCallbackCounter = 0;
			exCommand = false; // Allows QR detection
		}
		else
		{
			// Aborts goal and goes to initial position
			goalFeedback(6, 3, 3);
			diagnostics("goal aborted, c: ", moveCallbackCounter);
			moveCallbackCounter++;
			exCommand = false;																			 // Allows QR detection
			setGoal(coordinatesP1.at(0), coordinatesP1.at(1), coordinatesP1.at(2), coordinatesP1.at(3)); // Initial position
		}
	}

	// Tries to focus exactly to QR code tag, if it is not successed it will abort
	void redefinePosition()
	{
		goalFeedback(6, 0, 0);
		ros::Rate loop_rate(0.5);
		loop_rate.sleep();
		rotateRobot(1, 90, true);
		rotateRobot(1, 90, false);
		rotateRobot(1, 90, false);
		rotateRobot(1, 90, true);
		moveRobot(0.1, 0.1, false);
		rotateRobot(1, 90, true);
		rotateRobot(1, 90, false);
		rotateRobot(1, 90, false);
		rotateRobot(1, 90, true);
		diagnostics("redefine finish ", redefineCounter);
		if (redefineCounter == 0)
			goalFeedback(6, 3, 3);
	}

	// Tranforms angle from degrees to radians
	double computeRadians(double degrees)
	{
		return degrees * 2 * PI / 360.0;
	}

	// Rotates robot to specific angle
	void rotateRobot(double speed, double angle, bool clockwise)
	{
		if (redefineCounter == 0)
		{
			ros::Rate rate(0.5);
			geometry_msgs::Twist twistMsg;
			twistMsg.linear.x = 0.0;
			twistMsg.linear.y = 0.0;
			twistMsg.linear.z = 0.0;
			twistMsg.angular.x = 0.0;
			twistMsg.angular.y = 0.0;
			if (clockwise)
				twistMsg.angular.z = -abs(speed);
			else
				twistMsg.angular.z = abs(speed);

			double radiansAngle = computeRadians(angle);
			double currentAngle = 0.0;
			double t0 = ros::Time::now().toSec();
			ros::Rate loop_rate(100);
			do
			{
				if (redefineCounter == 0)
					movementPublisher.publish(twistMsg);
				double t1 = ros::Time::now().toSec();
				currentAngle = speed * (t1 - t0);
				ros::spinOnce();
				loop_rate.sleep();
			} while (currentAngle < radiansAngle && !exCommand);
			twistMsg.angular.z = 0.0;
			if (redefineCounter == 0)
				movementPublisher.publish(twistMsg);
			rate.sleep();
		}
	}

	// Moves robot to specific distance
	void moveRobot(double speed, double distance, bool direction)
	{
		if (redefineCounter == 0)
		{
			ros::Rate rate(0.5);
			geometry_msgs::Twist twistMsg;
			if (direction)
				twistMsg.linear.x = abs(speed);
			else
				twistMsg.linear.x = -abs(speed);
			twistMsg.linear.y = 0.0;
			twistMsg.linear.z = 0.0;
			twistMsg.angular.x = 0.0;
			twistMsg.angular.y = 0.0;
			twistMsg.angular.z = 0.0;

			double t0 = ros::Time::now().toSec();
			double currentDistance = 0.0;
			ros::Rate loop_rate(100);
			do
			{
				if (redefineCounter == 0)
					movementPublisher.publish(twistMsg);
				double t1 = ros::Time::now().toSec();
				currentDistance = speed * (t1 - t0);
				ros::spinOnce();
				loop_rate.sleep();
			} while (currentDistance < distance && !exCommand);
			twistMsg.linear.x = 0.0;
			if (redefineCounter == 0)
				movementPublisher.publish(twistMsg);
			rate.sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_node");
	FactoryRobot robot;
	ros::spin();
	return 0;
}
