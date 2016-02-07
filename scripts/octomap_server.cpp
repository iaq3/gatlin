#ifndef ROS_UI_H
#define ROS_UI_H
#include "ros/ros.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>
#include <vector>
#include <iostream>

#include <tf/transform_listener.h>

#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"

#include "octomap/octomap.h"
#include "octomap/AbstractOcTree.h"
#include "octomap/ColorOcTree.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "rgbdslam/rgbdslam_ros_ui.h"
#include "rgbdslam/rgbdslam_ros_ui_b.h"
#include "rgbdslam/rgbdslam_ros_ui_f.h"
#include "rgbdslam/rgbdslam_ros_ui_s.h"
using namespace std;
using namespace octomap;
using namespace octomap_msgs;

ros::Publisher pub;
ros::Publisher pose_pub;
ros::Publisher base_pose_pub;

ros::Subscriber sub;
ros::Subscriber ekf_sub;
ros::Subscriber sub_octomap;
ros::Subscriber sub_res;

ros::ServiceClient client;
rgbdslam::rgbdslam_ros_ui srv;

ros::ServiceClient client_save;
rgbdslam::rgbdslam_ros_ui_s srv_save;

ros::ServiceClient client_pause;
rgbdslam::rgbdslam_ros_ui_b srv_pause;

bool capture = false;
bool ready = true;

int pause() {
	srv_pause.request.command = "pause";
	srv_pause.request.value = true;

	if (client_pause.call(srv_pause)) {
		ROS_INFO("Pausing the capture of images");
	} else {
		ROS_ERROR("Failed to call service Rgbdslam pause");
		return 1;
	}
}

int start() {
	srv_pause.request.command = "pause";
	srv_pause.request.value = false;

	if (client_pause.call(srv_pause)) {
		ROS_INFO("Starting the capture of images");
	} else {
		ROS_ERROR("Failed to call service Rgbdslam start");
		return 1;
	}
}

int frame() {
	srv.request.command = "frame";

	if (client.call(srv)) {
		ROS_INFO("Capturing a frame");
	} else {
		ROS_ERROR("Failed to call service Rgbdslam frame capture");
		return 1;
	}  
}

int reset() {
	srv.request.command = "reset";

	if (client.call(srv)) {
		ROS_INFO("Resetting");
	} else {
		ROS_ERROR("Failed to call service Rgbdslam reset");
		return 1;
	}
}

int send_all() {
	srv.request.command = "send_all";

	if (client.call(srv)) {
		ROS_INFO("Sending all pointclouds");
	} else {
		ROS_ERROR("Failed to call service Rgbdslam send_all");
		return 1;
	}
}

int reload_config() {
	srv.request.command = "reload_config";

	if (client.call(srv)) {
		ROS_INFO("Reloading config");
	} else {
		ROS_ERROR("Failed to call service Rgbdslam reload_config");
		return 1;
	}
}
 

int save_trajectory() {
	srv_save.request.command = "save_trajectory";	
	srv_save.request.value = "trajectory";

	if (client_save.call(srv_save)) {
		ROS_INFO("Saving the trajectory");
	} else {
		ROS_ERROR("Failed to call service Rgbdslam save_trajectory");
		return 1;
	}
}

int save() {
	srv_save.request.command = "save_octomap";
	srv_save.request.value = "gatlin.ot";

	if (client_save.call(srv_save)) {
		ROS_INFO("Saving the octomap");
	} else {
		ROS_ERROR("Failed to call service Rgbdslam save_octomap");
		return 1;
	}
}

geometry_msgs::Pose makePoseMsg(vector<double> row) {
	geometry_msgs::Pose p;
	p.position.x = row[1];
	p.position.y = row[2];
	p.position.z = row[3];
	p.orientation.x = row[4];
	p.orientation.y = row[5];
	p.orientation.z = row[6];
	p.orientation.w = row[7];
	return p;
}

int read_trajectory(string filename) {
	
	string path = "/home/turtlebot/ros_ws/devel/lib/rgbdslam/"+filename;

	ifstream myFile(path.c_str());

	vector<vector<double> > matrix;

	for(std::string line; std::getline(myFile, line); ) {
		std::istringstream iss(line);
		vector<double> row;

		for(double d; iss >> d; ) {
			row.push_back(d);
			// cout << d << ", ";

		}
		// cout << endl;

		matrix.push_back(row);
	}

	vector<double> lastRow;
	lastRow = matrix.back();
	// cout << lastRow.size() << endl;

	// for(int i=0; i < lastRow.size(); i++){
	// 	cout << lastRow[i] << ", ";		
	// }
	// cout << endl;

	geometry_msgs::Pose p;
	p = makePoseMsg(lastRow);

	base_pose_pub.publish(p);
}

void setResolution(int res) {
	ros::NodeHandle n;	
	float r = (float) res /100.0;
	n.setParam("/rgbdslam/config/octomap_resolution", .07);
	// n.setParam("octomap_resolution", .07);
	ROS_INFO("Setting Resolution to: %f m", r);

}

std_msgs::UInt16MultiArray array;
void octomap_send_array(AbstractOcTree* tree) {
	array.data.clear();

	cout << " getTreeType: " << tree->getTreeType() << "\n";
	cout << " getResolution: " << tree->getResolution() << "\n";
	array.data.push_back( (unsigned short) (tree->getResolution()*100) );

	bool color = true;
	ColorOcTree* color_octree = dynamic_cast<ColorOcTree*>(tree);
	if(!color_octree) {
		ROS_ERROR("Failed to create color_octree structure");
		color = false;
		// return;
	}

	int count = 0;
	float min = 0;
	float r = tree->getResolution();
	double dx, dy, dz;

	if(color) {		
		// for(ColorOcTree::leaf_iterator it = color_octree->begin_leafs(), end=color_octree->end_leafs(); it!= end; ++it) {
		for(ColorOcTree::tree_iterator it = color_octree->begin_tree(), end=color_octree->end_tree(); it!= end; ++it) {
			if(min > it.getY()) {
				min = it.getY();
			}
			if (color_octree->isNodeOccupied(*it) && it.getSize()<=r*2) {
				// cout << " size: " << it.getSize() << "\n";

				ColorOcTreeNode::Color c = it->getColor();
				
				array.data.push_back( (unsigned short) (it.getX()*1000 + 30000) );
				array.data.push_back( (unsigned short) (it.getY()*1000 + 30000) );
				array.data.push_back( (unsigned short) (it.getZ()*1000 + 30000) );
				array.data.push_back( (unsigned short) c.r );
				array.data.push_back( (unsigned short) c.g );
				array.data.push_back( (unsigned short) c.b );
				array.data.push_back( (unsigned short) (it.getSize()*10000) );
				count++;
			}
		}
		// calculate size in meters
		color_octree->getMetricSize(dx, dy, dz);
	} else {
		OcTree* octree = dynamic_cast<OcTree*>(tree);
		// for(OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it) {
		for(OcTree::tree_iterator it = octree->begin_tree(), end=octree->end_tree(); it!= end; ++it) {
			if(min > it.getY()) {
				min = it.getY();
			}
			if (octree->isNodeOccupied(*it) && it.getSize()<=r*2) {
				// cout << " size: " << it.getSize() << "\n";

				ColorOcTreeNode::Color c;
				c.r = 0; c.g = 255; c.b = 0;

				array.data.push_back( (unsigned short) (it.getX()*1000 + 30000) );
				array.data.push_back( (unsigned short) (it.getZ()*-1000 + 30000) );
				array.data.push_back( (unsigned short) (it.getY()*1000 + 30000) );
				array.data.push_back( (unsigned short) c.r );
				array.data.push_back( (unsigned short) c.g );
				array.data.push_back( (unsigned short) c.b );
				array.data.push_back( (unsigned short) (it.getSize()*10000) );
				count++;
			}
		}
		// calculate size in meters
		octree->getMetricSize(dx, dy, dz);
	}
	

	
	cout << " size: " << dx << ", " << dy << ", " << dz << "\n";
	cout << " count: " << count << "\n";
	cout << " min: " << min << "\n";

	// for sending every new update automatically when ready
	if(ready) {
		pub.publish(array);
		ROS_DEBUG("Published octomap_array!");
		ready = false;
	}	
}

void octomapCb(const octomap_msgs::Octomap::ConstPtr msg) {
// void octomapCb(const octomap_msgs::OctomapConstPtr& msg) {
	// creating octree
	AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

	cout << " getTreeType: " << tree->getTreeType() << "\n";
	cout << " getResolution: " << tree->getResolution() << "\n";
	
	octomap_send_array(tree);
}

// void file_to_octomap_array(string filename) {
// 	string dir = "/home/turtlebot/ros_ws/devel/lib/rgbdslam/";

// 	AbstractOcTree* tree = AbstractOcTree::read(dir+filename);
// 	octomap_send_array(tree);
// }

void poseCb() {
	// call save_trajectory
	save_trajectory();
	// read in trajectory estimate file
	// read_trajectory("trajectory_estimate.txt");
	// publish latest pose
}

void resCb(const std_msgs::Float32::ConstPtr& msg) {
	// ROS_INFO("###########################################");
	ROS_INFO("############### Resolution! ###############");
	ROS_INFO("################## %.3f ###################", msg->data);
	// ROS_INFO("###########################################");

	// float octomap_resolution;
	// if (ros::param::get("/rgbdslam/config/octomap_resolution", octomap_resolution)) {
	// 	ROS_INFO("################## %.3f ###################", octomap_resolution);
	// }
	
	ros::param::set("/rgbdslam/config/octomap_resolution", msg->data);

	// if (ros::param::get("/rgbdslam/config/octomap_resolution", octomap_resolution)) {
	// 	ROS_INFO("################## %.3f ###################", octomap_resolution);
	// }
	
}

void markerCb() {

}

geometry_msgs::Pose tf_to_pose(tf::StampedTransform transform) {
	geometry_msgs::Pose p;
	p.position.x = transform.getOrigin().x();
	p.position.y = transform.getOrigin().y();
	p.position.z = transform.getOrigin().z();
	p.orientation.x = transform.getRotation().x();
	p.orientation.y = transform.getRotation().y();
	p.orientation.z = transform.getRotation().z();
	p.orientation.w = transform.getRotation().w();
	return p;
}

void tfCb() {
	tf::TransformListener listener;
	// start();
	// sleep(1);

	tf::StampedTransform transform;
	try{
		// ros::Time now = ros::Time::now();
		listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		
		// listener.waitForTransform("/base_link", "/camera_rgb_optical_frame", ros::Time(0), ros::Duration(6.0));
		// listener.lookupTransform("/base_link", "/camera_rgb_optical_frame", ros::Time(0), transform);

		geometry_msgs::Pose p;
		p = tf_to_pose(transform);
		cout << " position: " << p.position.x << ", " << p.position.y << ", " << p.position.z << "\n";
		cout << " orientation: " << p.orientation.x << ", " << p.orientation.y << ", " << p.orientation.z << ", " << p.orientation.w << "\n";
		base_pose_pub.publish(p);

		// double x = transform.getOrigin().x();
		// double y = transform.getOrigin().y();
		// double z = transform.getOrigin().z();
		// cout << " transform: " << x << ", " << y << ", " << z << "\n";
		sleep(1);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}


	sleep(1);
	pause();
}

void serverCb() {
	// start image capture until button is pressed
	// start image capture for 5 sec
	// start();
	// sleep(5);
	frame();
	// send the pointclouds to octomap_filter
	// send_all();

	// pause();

	// subscribe to the occupied_cells_vis_array 

	// TODO:
	// rgbdslam doesn't work well on plain walls, no features
	// look for way to run rgbdslam on desktop and combine odometry
	// add pose tf arrow marker
	// pick up something with turtlebot looking at phone


}

void cmdCb(const std_msgs::Int32::ConstPtr& msg) {

	// poseCb(msg);

	ROS_DEBUG("I heard: [%d]", msg->data);

	if(msg->data == 5) {
		//Publish array
		pub.publish(array);

		ROS_DEBUG("Published octomap_array!");

	}
	else if(msg->data == 6) {
		ROS_DEBUG("Start Capture!");
		capture = true;
	}	
	else if(msg->data == 7) {
		ROS_DEBUG("Stop Capture!");
		pause();
		capture = false;
	}
	
	else if(msg->data == 8) {
		ROS_DEBUG("Reset Map!");
		reset();
	}
	
	else if(msg->data == 1) {
		ROS_DEBUG("Now Ready!");
		ready = true;
	}
	
	// if(msg->data.c_str() == "send_octomap") {

		// reset();
		// sleep(1);
		// take snapshots with one octomap
		//

		// setResolution(msg->data);
		// frame();
		// save_trajectory();
		// sleep(1);
		// save();

		// sleep(5);

		// read an octomap from file and publish an array of occupied points
		// file_to_octomap_array("gatlin.ot");

		// ros::spinOnce();

		// sleep(2);
	// }
}

void ekf_poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {
	base_pose_pub.publish(msg->pose.pose);

}

int main( int argc, char **argv ) {
	ros::init(argc, argv, "octomap_server");
	
	ros::NodeHandle n;

	pub = n.advertise<std_msgs::UInt16MultiArray>("/octomap_array", 10000000);

	// pose_pub = n.advertise<geometry_msgs::Pose>("/octomap_pose", 10);
	base_pose_pub = n.advertise<geometry_msgs::Pose>("/octomap_pose", 10);
	ekf_sub = n.subscribe("/robot_pose_ekf/odom_combined", 1000, ekf_poseCb);

	n.advertise<std_msgs::Int32>("/gatlin_cmd", 1000);

	sub = n.subscribe("/gatlin_cmd", 1000, cmdCb);

	sub_octomap = n.subscribe("/octomap_update", 10, octomapCb);
	sub_res = n.subscribe("/gatlin_res", 10, resCb);

	// sub_octomap = n.subscribe("/octomap_binary", 1000, octomapCb);

	if(!sub_res) {
		ROS_ERROR("Not valid sub");
	}

	client = n.serviceClient<rgbdslam::rgbdslam_ros_ui>("/rgbdslam/ros_ui");
	client_save = n.serviceClient<rgbdslam::rgbdslam_ros_ui_s>("/rgbdslam/ros_ui_s");
	client_pause = n.serviceClient<rgbdslam::rgbdslam_ros_ui_b>("/rgbdslam/ros_ui_b");

	// sleep(4);
	// pause();
	//TODO: write 360 degree octomap generator
	// have rgbdslam send out message when there arent enough features to track
	// so head can try to go back and cover the area
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		if(capture) {
			serverCb();
			// poseCb();
		}

		// tfCb();

		ros::spinOnce();

		loop_rate.sleep();
	}

	exit(0);


	return 0;
}

#endif