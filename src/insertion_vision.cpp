#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/JointCommand.h>
#include <cmath>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>

static const std::string OPENCV_WINDOW = "Camera";
static const std::string OPENCV_WINDOW_K = "Keypoints";

typedef moveit::planning_interface::MoveGroup::Plan MoveGroupPlan;

const int JOINTS_RIGHT_ARM_INDEX_START = 8;
const int JOINTS_RIGHT_ARM_INDEX_END = 15;

std::string camera_topic;

bool readyToFindWire = false;

// Global variables to contain USB pose information
geometry_msgs::Pose usbPose;
int usbFound = 0; // reset this to zero prior to gripper close

MoveGroupPlan planStampNow(MoveGroupPlan p)
{
	MoveGroupPlan plan;
	plan = p;
	plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
	return plan;
}

MoveGroupPlan createReversedPlan(MoveGroupPlan p)
{
	trajectory_msgs::JointTrajectory trajectory;
	trajectory.header = p.trajectory_.joint_trajectory.header;
	trajectory.joint_names = p.trajectory_.joint_trajectory.joint_names;
	trajectory.points.resize(p.trajectory_.joint_trajectory.points.size());

	for (int i=0; i < trajectory.points.size(); i++)
	{
		trajectory.points[i] = p.trajectory_.joint_trajectory.points[trajectory.points.size() - 1 - i];
		trajectory.points[i].time_from_start = p.trajectory_.joint_trajectory.points[i].time_from_start;
	}

	MoveGroupPlan q;
	q.trajectory_.joint_trajectory = trajectory;
	return q;
}

MoveGroupPlan concatenatePlans(MoveGroupPlan p1, MoveGroupPlan p2)
{
	// concatenate trajectories
	trajectory_msgs::JointTrajectory trajectory;
	trajectory = p1.trajectory_.joint_trajectory;
	trajectory.points.insert(trajectory.points.end(), p2.trajectory_.joint_trajectory.points.begin() + 1, p2.trajectory_.joint_trajectory.points.end());

	// adjust time for trajectory points from 2nd plan
	int size = p1.trajectory_.joint_trajectory.points.size();
	ros::Duration t1 = trajectory.points[size - 1].time_from_start;

	for (int i = 0; i < p2.trajectory_.joint_trajectory.points.size(); i++)
	{
		ros::Duration t2 = p2.trajectory_.joint_trajectory.points[i].time_from_start;
		trajectory.points[size + i].time_from_start = t1 + t2;
	}

	MoveGroupPlan p3;
	p3.trajectory_.joint_trajectory = trajectory;
	return p3;
}

MoveGroupPlan createPlan(std::vector<double> joints1, std::vector<double> joints2, 
		int n, double dt, std::vector<std::string> joint_names)
{			
	trajectory_msgs::JointTrajectory trajectory;
	trajectory.points.resize(n);
	trajectory.joint_names.resize(joints1.size());

	for (int i = 0; i < joints1.size(); i++)
		trajectory.joint_names[i] = joint_names[i];

	for (int i=0; i < n; i++)
	{
		trajectory_msgs::JointTrajectoryPoint p;
		p.velocities.resize(joints1.size());
		p.positions.resize(joints1.size());

		for (int j=0; j < p.positions.size(); j++)
		{
			p.positions[j] = joints1[j] + ((joints2[j] - joints1[j]) / n) * (i + 1);
			p.time_from_start = ros::Duration(dt * (i + 1));
			p.velocities[j] = ((joints2[j] - joints1[j]) / (double) n) / dt;
		}

		trajectory.points[i] = p;
		ROS_INFO_STREAM("trajectory point "<<i<<": \n"<<trajectory.points[i]);
	}

	trajectory.header.stamp = ros::Time::now(); // + ros::Duration(1.0);
	MoveGroupPlan plan;
	plan.trajectory_.joint_trajectory = trajectory;	
	return plan;
}

void scaleTrajectorySpeed(trajectory_msgs::JointTrajectory &trajectory, double factor)
{
	for (int i=0; i < trajectory.points.size(); i++)
	{
		trajectory_msgs::JointTrajectoryPoint p;
		p.positions = trajectory.points[i].positions;
		p.time_from_start = trajectory.points[i].time_from_start * (1.0 / factor);
		p.velocities = trajectory.points[i].velocities;
		p.accelerations = trajectory.points[i].accelerations;

		for (int j=0; j < trajectory.joint_names.size(); j++)
		{
			p.velocities[j] = p.velocities[j] * factor;
			p.accelerations[j] = p.accelerations[j] * (factor * factor);
		}

		trajectory.points[i] = p;
	}	
}

geometry_msgs::Point createPoint(double x, double y, double z)
{
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	p.z = z;	
	return p;
}

geometry_msgs::Quaternion createQuaternion(double x, double y, double z, double w)
{
	geometry_msgs::Quaternion q;
	q.x = x;
	q.y = y;
	q.z = z;
	q.w = w;	
	return q;
}

std_msgs::Header createHeader(ros::Time time, std::string frame)
{
	std_msgs::Header h;
	h.stamp = time;
	h.frame_id = frame;
	return h;
}

geometry_msgs::Pose convertToPose(tf::StampedTransform transform)
{
	tf::Quaternion q = transform.getRotation();
	tf::Point p = transform.getOrigin();

	geometry_msgs::Pose pose;
	pose.position = createPoint(p.x(), p.y(), p.z());
	pose.orientation = createQuaternion(q.x(), q.y(), q.z(), q.w());
	return pose;
}

Eigen::Matrix4d createHomogeneousTransformMatrix(tf::StampedTransform transform)
{
	tf::Point p = transform.getOrigin();
	tf::Quaternion q = transform.getRotation();
	tf::Matrix3x3 R1(q);
	Eigen::Matrix3d R2;
	tf::matrixTFToEigen(R1, R2);
	ROS_INFO_STREAM("R2:\n"<<R2);

	Eigen::Matrix4d T;
	T.block(0, 0, 3, 3) << R2;
	T.block(0, 3, 3, 1) << p.x(), p.y(), p.z();
	T.row(3) << 0, 0, 0, 1;
	return T;
}

std::vector<double> extractMoveItJointValues(sensor_msgs::JointState state, std::vector<std::string> names)
														{
	std::vector<double> joint_values(names.size());

	for (int i=0; i < names.size(); i++)
	{
		for (int j=0; j < state.name.size(); j++)
		{
			if (names[i] == state.name[j])
			{
				ROS_INFO_STREAM("joint found - name(moveit): "<<names[i]<<", name(ik solver): "<<state.name[j]<<", index(moveit): "<<i<<", index(ik solver): "<<j);
				joint_values[i] = state.position[j];
				break;
			}
		}
	}

	return joint_values;
														}

std::vector<double> extractRightArmJointValues(sensor_msgs::JointState state)
														{
	std::vector<double> joint_values(state.position.begin() + JOINTS_RIGHT_ARM_INDEX_START, 
			state.position.begin() + JOINTS_RIGHT_ARM_INDEX_END);

	return joint_values;
														}

void setJointValues(ros::Publisher pub, int mode, std::vector<double> values, std::vector<std::string> joint_names)
{
	baxter_core_msgs::JointCommand msg;
	msg.mode = mode;
	msg.command.resize(values.size());
	msg.names.resize(joint_names.size());

	for (int j=0; j < msg.command.size(); j++)
		msg.command[j] = values[j];

	msg.names = joint_names;
	pub.publish(msg);
}


void move(ros::Publisher pub, move_group_interface::MoveGroup group, std::vector<double> joints_target, std::vector<std::string> joint_names, double speed, double thres_in)
{
	std::vector<double> joints_curr;
	std::vector<double> joint_error;
	std::vector<double> joint_vel_com;

	double error_magnitude;
	double alpha = 1; // step size
	double joint_vel_com_mag;

	double max_vel_mag = 0.08;
	if (speed < max_vel_mag)
		max_vel_mag = speed;

	double thres = 0.01;
	if (thres_in < thres)
		thres = thres_in;

	for (int i=0; i< 100000; i++)
	{
		//~ std::cout << "hi 1" << std::endl;
		joints_curr = group.getCurrentJointValues();
		joint_error = joints_curr;
		joint_vel_com = joints_curr;

		//~ std::cout << "hi 2" << std::endl;
		// calculate joint error, error magnitude, comm vel
		error_magnitude = 0;
		for (int j=0; j< joint_error.size(); j++) {
			joint_error[j] = joints_target[j] - joints_curr[j];
			joint_vel_com[j] = joint_error[j] * alpha;
			error_magnitude += joint_error[j]*joint_error[j];
		}
		//~ std::cout << "hi 3" << std::endl;
		error_magnitude = sqrt(error_magnitude);
		joint_vel_com_mag = error_magnitude*alpha;

		//~ std::cout << "hi 4" << std::endl;
		// if comm vel exceeds max, scale it back to max
		if (joint_vel_com_mag > max_vel_mag)
			for (int j=0; j< joint_vel_com.size(); j++)
				joint_vel_com[j] = max_vel_mag * joint_vel_com[j] / joint_vel_com_mag;

		//~ std::cout << "hi 5" << std::endl;
		setJointValues(pub, baxter_core_msgs::JointCommand::VELOCITY_MODE, joint_vel_com, joint_names);
		std::cout << "iter: " << i << " error_magnitude: " << error_magnitude << std::endl;
		//~ std::cout << "hi 6" << std::endl;

		if (error_magnitude < thres)
			break;

		//~ std::cout << "hi 7" << std::endl;
		usleep(1000);
	}

	// need to reset to position mode when we're done or the robot will disable itself...
	setJointValues(pub, baxter_core_msgs::JointCommand::POSITION_MODE, joints_target, joint_names);

}

void usbCallback(const geometry_msgs::Pose &pose)
{		
	ROS_INFO("received usb cable pose: (x, y, theta) = (%.2f, %.2f, %.2f)", 
			pose.position.x, pose.position.y, 
			pose.orientation.w);
	usbPose = pose;
	usbFound = 1;	
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Draw an example circle on the video stream
	//if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	//	cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

	// Update GUI Window
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);

	if(readyToFindWire == true)
	{
		// detect blobs
		cv::SimpleBlobDetector::Params params;
		params.minThreshold = 40;
		params.maxThreshold = 60;
		params.thresholdStep = 50;

		params.minArea = 100;
		params.minConvexity = 0.3;
		params.minInertiaRatio = 0.01;

		params.maxArea = 8000;
		params.maxConvexity = 10;

		params.filterByColor = true;
		params.filterByCircularity = false;

		cv::line( cv_ptr->image , cv::Point(0, cv_ptr->image.rows-1), cv::Point( cv_ptr->image.cols-1, cv_ptr->image.rows-1 ), cv::Scalar::all(255) );

		cv::SimpleBlobDetector blobDetector( params );
		blobDetector.create("SimpleBlob");


		std::vector<cv::KeyPoint> keyPoints;
		std::vector< std::vector <cv::Point> > contours;
		std::vector< std::vector <cv::Point> > approxContours;
		cv::Mat out;

		blobDetector.detect( cv_ptr->image, keyPoints );
		//blobDetector.detectEx( cv_ptr->image, keyPoints, contours );
		cv::drawKeypoints( cv_ptr->image, keyPoints, out, CV_RGB(0,255,0), cv::DrawMatchesFlags::DEFAULT);
		approxContours.resize( contours.size() );

		for( int i = 0; i < contours.size(); ++i )
		{
			approxPolyDP( cv::Mat(contours[i]), approxContours[i], 4, 1 );
			drawContours( out, contours, i, CV_RGB(rand()&255, rand()&255, rand()&255) );
			cv::drawContours( out, approxContours, i, CV_RGB(rand()&255, rand()&255, rand()&255) );
		}
		if (keyPoints.size() > 0)
		{
			std::cout << "Keypoints " << keyPoints.size() << std::endl;
			cv::imshow(OPENCV_WINDOW_K, out);
			//cv::waitKey(0);
		}
	}

	// Output modified video stream
	// image_pub_.publish(cv_ptr->toImageMsg());
}


int main(int argc, char **argv)
{	
	// unit test for extractPose() and createHomogeneousTransformMatrix()
	//~ tf::StampedTransform t;
	//~ tf::Vector3 origin(2, 3, 4);
	//~ tf::Quaternion quaternion(0, 1, 0, 1);
	//~ t.setOrigin(origin);
	//~ t.setRotation(quaternion);
	//~ geometry_msgs::Pose p = convertToPose(t);
	//~ ROS_INFO_STREAM("pose:\n" << p);
	//~ Eigen::Matrix4d T = createHomogeneousTransformMatrix(t);
	//~ ROS_INFO_STREAM("homogeneous transformation matrix:\n" << T);
	//~ return 1;

	ros::init(argc, argv, "insertion");
	ros::NodeHandle node("~");

	ros::AsyncSpinner spinner(1);
	spinner.start();
	/*
	ros::Publisher pub = node.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 10);

	ros::Subscriber sub = node.subscribe("/usb_pose", 10, usbCallback);
	ros::Rate rate(1.0);

	ros::Publisher gripper_pub = node.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 10);
	 */



	// create OpenCV window
	cv::namedWindow(OPENCV_WINDOW);
	cv::namedWindow(OPENCV_WINDOW_K, CV_GUI_NORMAL);

	node.param<std::string>("camera_topic", camera_topic, "/cameras/right_hand_camera/image");

	ROS_INFO("Listening to: %s", camera_topic.c_str());
	// subscribe to the camera topic
	image_transport::ImageTransport it_(node);
	image_transport::Subscriber image_sub_ = it_.subscribe(camera_topic, 1, 
			imageCallback);

	// move arm top in front of wires

	// once arm is in position, ready to detect blobs
	readyToFindWire = true;


	/*
	// create messages to control the gripper
	baxter_core_msgs::EndEffectorCommand ee_msg_close;
	ee_msg_close.id = 65664;
	ee_msg_close.command = "grip";
	ee_msg_close.args = "{}";

	baxter_core_msgs::EndEffectorCommand ee_msg_open;
	ee_msg_open.id = 65664;
	ee_msg_open.command = "release";
	ee_msg_open.args = "{}";

	move_group_interface::MoveGroup group("right_arm");
	move_group_interface::MoveGroup group1("right_arm");
	move_group_interface::MoveGroup group2("right_arm");
	move_group_interface::MoveGroup group3("right_arm");
	move_group_interface::MoveGroup group4("right_arm");
	move_group_interface::MoveGroup group5("right_arm");
	move_group_interface::MoveGroup group6("right_arm");
	move_group_interface::MoveGroup group7("right_arm");

	// get current pose and joints
	geometry_msgs::PoseStamped curr_pose = group.getCurrentPose();
	ROS_INFO_STREAM("current end effector pose: "<<curr_pose);
	std::vector<double> joint_values = group.getCurrentJointValues();
	std::vector<std::string> joint_names = group.getJoints();
	for (int i = 0; i < joint_values.size(); i++)
		std::cout<<joint_names[i]<<": "<<joint_values.at(i)<<std::endl;

	// setup IK solver
	ros::ServiceClient service_client = node.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
	while(!service_client.exists())
	{
		ROS_INFO("Waiting for service");
		sleep(1.0);
	}
	moveit_msgs::GetPositionIK::Request service_request;
	moveit_msgs::GetPositionIK::Response service_response; 

	ROS_INFO_STREAM("Press Enter when ready ");
	char c = std::cin.get();

	curr_pose = group.getCurrentPose();
	ROS_INFO_STREAM("current end effector pose: "<<curr_pose);

	// hardcode fixed target poses
	geometry_msgs::PoseStamped p1, p2, p3, p4, p5, p6, p7, p8, p9;
	p1.header = createHeader(ros::Time(0), "/base");
	p1.pose.position = createPoint(0.76, -0.445, -0.02);
	p1.pose.orientation = createQuaternion(0,0.707,0,0.707);
	p2.header = createHeader(ros::Time(0), "/base");
	p2.pose.position = createPoint(0.815, -0.445, -0.025);
	p2.pose.orientation = createQuaternion(0,0.707,0,0.707);
	p3.header = createHeader(ros::Time(0), "/base");
	p3.pose.position = createPoint(0.80, -0.445, -0.025);
	p3.pose.orientation = createQuaternion(0,0.707,0,0.707);
	p4.header = createHeader(ros::Time(0), "/base");
	p4.pose.position = createPoint(0.83, -0.445, -0.025);
	p4.pose.orientation = createQuaternion(0,0.707,0,0.707);
	p5.header = createHeader(ros::Time(0), "/base");
	p5.pose.position = createPoint(0.835, -0.445, -0.025);
	p5.pose.orientation = createQuaternion(0,0.707,0,0.707);
	p6.header = createHeader(ros::Time(0), "/base");
	p6.pose.position = createPoint(0.815, -0.445, -0.095);
	p6.pose.orientation = createQuaternion(0,0.707,0,0.707);
	p7.header = createHeader(ros::Time(0), "/base");
	p7.pose.position = createPoint(0.815, -0.445, -0.115);
	p7.pose.orientation = createQuaternion(0,0.707,0,0.707);
	p8.header = createHeader(ros::Time(0), "/base");
	p8.pose.position = createPoint(0.819, -0.445, -0.115);
	p8.pose.orientation = createQuaternion(0,0.707,0,0.707);
	p9.header = createHeader(ros::Time(0), "/base");
	p9.pose.position = createPoint(0.811, -0.445, -0.115);
	p9.pose.orientation = createQuaternion(0,0.707,0,0.707);
	std::vector<geometry_msgs::PoseStamped> waypoints;
	waypoints.push_back(p1);
	waypoints.push_back(p2);
	waypoints.push_back(p3);
	waypoints.push_back(p4);
	waypoints.push_back(p5);
	waypoints.push_back(p6);
	waypoints.push_back(p7);
	waypoints.push_back(p8);

	// ---------------------------------------------------------
	// hardcode nominal joint configuration
	std::vector<double> joint_values_nominal = group.getCurrentJointValues();
	std::vector<double> joint_values_pregrasp = group.getCurrentJointValues();

	// These values are for the USB-Monitor alignment
	joint_values_nominal[0] = 1.04963;
	joint_values_nominal[1] = -0.865932;
	joint_values_nominal[2] = -0.0613592;
	joint_values_nominal[3] = 2.2864;
	joint_values_nominal[4] = 1.055;
	joint_values_nominal[5] = 0.00076699;
	joint_values_nominal[6] = -0.885107;
	//~ joint_values_nominal[0] = 0.627015;
	//~ joint_values_nominal[1] = -0.575626;
	//~ joint_values_nominal[2] = 0.883189;
	//~ joint_values_nominal[3] = 1.50982;
	//~ joint_values_nominal[4] = -0.854044;
	//~ joint_values_nominal[5] = 0.967942;
	//~ joint_values_nominal[6] = 2.38419;

	// joint values for pregrasp of USB
	//~ joint_values_pregrasp[0] = 0.0318301;
	//~ joint_values_pregrasp[1] = 0.0644272;
	//~ joint_values_pregrasp[2] = 0.37851;
	//~ joint_values_pregrasp[3] = 1.42468;
	//~ joint_values_pregrasp[4] = -0.790767;
	//~ joint_values_pregrasp[5] = -1.24828;
	//~ joint_values_pregrasp[6] = -0.179859;
	joint_values_pregrasp[0] = 0.530374;
	joint_values_pregrasp[1] = -0.368539;
	joint_values_pregrasp[2] = 0.214374;
	joint_values_pregrasp[3] = 2.24345;
	joint_values_pregrasp[4] = 0.274583;
	joint_values_pregrasp[5] = -1.43005;
	joint_values_pregrasp[6] = -0.158767;

	MoveGroupPlan plan;
	//~ // ---------------------------------------------------------
	//~ // Move to nominal configuration
	//~ joint_values = group.getCurrentJointValues();
	//~ ROS_INFO_STREAM("Press Enter to move to nominal configuration");
	//~ c = std::cin.get();
	//~ plan = createPlan(joint_values, joint_values_nominal, 10, 1, joint_names);
	//~ if (c == '\n')
	//~ {
		//~ plan = planStampNow(plan);
		//~ group.execute(plan);
	//~ }
	//~ ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());

	// ---------------------------------------------------------
	// Move to pre-grasp configuration
	joint_values = group.getCurrentJointValues();
	ROS_INFO_STREAM("Press Enter to move to pregrasp configuration");
	c = std::cin.get();
	plan = createPlan(joint_values, joint_values_pregrasp, 10, 1, joint_names);
	if (c == '\n')
	{
		plan = planStampNow(plan);
		group1.execute(plan);
	}
	ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());

	std::vector<double> target_joint_values(joint_names.size());
	sensor_msgs::JointState joint_state;

	//~ // solve IK for each waypoint in list
	int service_error;
	std::vector<sensor_msgs::JointState> waypoints_joint;
	service_request.ik_request.group_name = "right_arm";
	for (int i=0; i<waypoints.size(); i++)
	{
		service_request.ik_request.pose_stamped = waypoints[i];
		service_client.call(service_request, service_response);
		ROS_INFO_STREAM("waypoints["<<0<<"]: \n"<<waypoints[i]);
		service_error = service_response.error_code.val;
		ROS_INFO_STREAM("IK solver error code: " << service_error);
		if (service_error == service_response.error_code.FAILURE)
		{
			ROS_INFO_STREAM("Exiting ...");
			return 1;
		}
		waypoints_joint.push_back(service_response.solution.joint_state);
	}

	// ---------------------------------------------------------
	// Move to waypoint 1
	joint_values = group1.getCurrentJointValues();
	joint_state = waypoints_joint[0];
	target_joint_values = extractRightArmJointValues(joint_state);
	ROS_INFO_STREAM("Press Enter to move to target #"<<1);
	c = std::cin.get();
	if (c == '\n')
	{
		move(pub, group1, target_joint_values, joint_names,0.08,0.01);
	}
	ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());

	// ---------------------------------------------------------
	// Move to waypoint 2
	joint_values = group2.getCurrentJointValues();
	joint_state = waypoints_joint[1];
	target_joint_values = extractRightArmJointValues(joint_state);
	ROS_INFO_STREAM("Moving to target #"<<2);
	move(pub, group2, target_joint_values, joint_names,0.08,0.002);
	ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());


	// ---------------------------------------------------------
	// Iteratively close gripper until we find the USB
	move_group_interface::MoveGroup * grouptemp;
	//~ move_group_interface::MoveGroup grouptemp("right_arm");
	for (int usbiter = 0; usbiter<4; usbiter++)
	{
		for (int usbiter2 = 0; usbiter2<2; usbiter2++)
		{
			grouptemp = new move_group_interface::MoveGroup("right_arm");
			joint_values = grouptemp->getCurrentJointValues();
			joint_state = waypoints_joint[1+usbiter2];
			target_joint_values = extractRightArmJointValues(joint_state);
			ROS_INFO_STREAM("Moving to target #"<<2+usbiter2);
			move(pub, *grouptemp, target_joint_values, joint_names,0.08,0.002);

			ROS_INFO_STREAM("Press Enter to close gripper"<<0);
			c = std::cin.get();
			usbFound = 0; // zero out flag
			if (c == '\n')
			{
				gripper_pub.publish(ee_msg_close);
			}

			usleep(4000000); // sleep 1 sec
			ROS_INFO_STREAM("hi 1");
			//~ delete grouptemp;
			ROS_INFO_STREAM("hi 2");


			if (usbFound > 0) {
			ROS_INFO_STREAM("hi 3");
				usbiter = 10; // cause outer loop to break
				break;
			}

			gripper_pub.publish(ee_msg_open);
		}

	}

	ROS_INFO_STREAM("hi 4");

	//~ ROS_INFO_STREAM("Close gripper. Press Enter when done"<<0);
	//~ c = std::cin.get();

	//~ std::vector<double> joint_values_preinsert = group.getCurrentJointValues();
	//~ joint_values_preinsert[0] = 0.737845;
	//~ joint_values_preinsert[1] = 0.0682621;
	//~ joint_values_preinsert[2] = 0.266913;
	//~ joint_values_preinsert[3] = 1.54012;
	//~ joint_values_preinsert[4] = -1.66552;
	//~ joint_values_preinsert[5] = -1.28624;
	//~ joint_values_preinsert[6] = -0.0705631;
//~ 
	//~ // ---------------------------------------------------------
	//~ // Move to pre-insert configuration
	//~ joint_values = group.getCurrentJointValues();
	//~ ROS_INFO_STREAM("Press Enter to move to pregrasp configuration");
	//~ c = std::cin.get();
	//~ plan = createPlan(joint_values, joint_values_preinsert, 10, 1, joint_names);
	//~ if (c == '\n')
	//~ {
		//~ plan = planStampNow(plan);
		//~ group.execute(plan);
	//~ }
	//~ ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());
	//~ 
	//~ 

	// ---------------------------------------------------------
	// Move to waypoint 5
	joint_values = group3.getCurrentJointValues();
	joint_state = waypoints_joint[4];
	target_joint_values = extractRightArmJointValues(joint_state);
	ROS_INFO_STREAM("Press Enter to move to target #"<<5);
	c = std::cin.get();
	//~ plan = createPlan(joint_values, target_joint_values, 5, 1, joint_names);
	if (c == '\n')
	{
		move(pub, group3, target_joint_values, joint_names,0.08,0.01);
	}
	ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());

	// ---------------------------------------------------------
	// Move to waypoint 6
	joint_values = group4.getCurrentJointValues();
	joint_state = waypoints_joint[5];
	target_joint_values = extractRightArmJointValues(joint_state);
	ROS_INFO_STREAM("Press Enter to move to target #"<<6);
	c = std::cin.get();
	if (c == '\n')
	{
		move(pub, group4, target_joint_values, joint_names,0.04,0.004);
	}
	ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());

	printf("usbFound: %d\n", usbFound);
	printf("usbPose: %f, %f, %f\n", usbPose.position.x, usbPose.position.y, usbPose.position.z);

	// These are the delta motions to send to the gripper
	//~ double deltaz = -0.001*(usbPose.position.x - 90)/22;
	//~ double deltax = 0.001*(usbPose.position.y + 78)/22;
	double deltaz = -0.001*(usbPose.position.x - 90)/18;
	double deltax = 0.001*(usbPose.position.y + 78)/18;
	deltax -= 0.001*(usbPose.orientation.w+1)*0.55;
	ROS_INFO_STREAM("Gripper offset from sensor: "<<deltax<<", "<<deltaz);

	waypoints[6].pose.position.x = waypoints[6].pose.position.x + deltax;
	waypoints[6].pose.position.z = waypoints[6].pose.position.z + deltaz;

	// solve IK for each waypoint in list
	waypoints_joint.clear();
	service_request.ik_request.group_name = "right_arm";
	for (int i=0; i<waypoints.size(); i++)
	{
		service_request.ik_request.pose_stamped = waypoints[i];
		service_client.call(service_request, service_response);
		ROS_INFO_STREAM("waypoints["<<0<<"]: \n"<<waypoints[i]);
		service_error = service_response.error_code.val;
		ROS_INFO_STREAM("IK solver error code: " << service_error);
		if (service_error == service_response.error_code.FAILURE)
		{
			ROS_INFO_STREAM("Exiting ...");
			return 1;
		}
		waypoints_joint.push_back(service_response.solution.joint_state);
	}

	// ---------------------------------------------------------
	// Move to waypoint 7
	joint_values = group.getCurrentJointValues();
	joint_state = waypoints_joint[6];
	target_joint_values = extractRightArmJointValues(joint_state);
	ROS_INFO_STREAM("Press Enter to move to target #"<<7);
	c = std::cin.get();
	if (c == '\n')
	{
		move(pub, group5, target_joint_values, joint_names,0.02,0.002);
	}
	ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());

	// ---------------------------------------------------------
	// Move to waypoint 8
	joint_values = group.getCurrentJointValues();
	joint_state = waypoints_joint[7];
	target_joint_values = extractRightArmJointValues(joint_state);
	ROS_INFO_STREAM("Press Enter to move to target #"<<8);
	move(pub, group5, target_joint_values, joint_names,0.02,0.004);
	ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());

	// ---------------------------------------------------------
	// Move to waypoint 9
	joint_values = group.getCurrentJointValues();
	joint_state = waypoints_joint[8];
	target_joint_values = extractRightArmJointValues(joint_state);
	ROS_INFO_STREAM("Press Enter to move to target #"<<9);
	move(pub, group6, target_joint_values, joint_names,0.02,0.004);
	ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());

	// ---------------------------------------------------------
	// Move to waypoint 7
	joint_values = group.getCurrentJointValues();
	joint_state = waypoints_joint[6];
	target_joint_values = extractRightArmJointValues(joint_state);
	ROS_INFO_STREAM("Press Enter to move to target #"<<7);
	move(pub, group7, target_joint_values, joint_names,0.02,0.004);
	ROS_INFO_STREAM("current end-effector pose: "<<group.getCurrentPose());


	ROS_INFO_STREAM("press any key to terminate");
	c = std::cin.get();

	 */
	// destroy cv window
	ROS_INFO_STREAM("press any key to terminate");
	std::cin.get();

	cv::destroyWindow(OPENCV_WINDOW);
	cv::destroyWindow(OPENCV_WINDOW_K);

	ros::shutdown();
	return 0;	
}
