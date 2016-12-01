// key_move.cpp
// 161014
// this is a modified code from the old project for the new version of ROS.
// 
// topic was changed from odom to /odom
// "world" to "odom" (but it sounds a little weird...)
// 
// the original copy right is shown below.


/* transf.cpp
 
 Author: Tetsuya Idota
 Team: Zhang & Idota
 Instructor: Dr.Pyeatt
 Course: CSC-515 Introduction to Robotics
 Update: 2012/12/12
 Version: 1.0.0
 Description:
  This program listens to the odometry /odom and publishes the translation /tf from local frame to world one.
  Once "t" key is pushed down, then the local frame is copied to the world frame.

 Dependency:
  <depend package="tf"/>
  <depend package="nav_msgs"/>
  <depend package="rospy"/>
  <depend package="roscpp"/>
 
 Build:
  Create a new package including the above dependencies
  After creating a new package, put this source file to ./src directory
  In CMakeLists.txt, add this line
     rosbuild_add_executable(transf src/transf.cpp)
  Then, execut this command
     $make

 Environment:
  It works on electric of ROS
  After launching turtlebot simulator, rosrun command can be used to run this program
     $rosrun <package name> transf
  Then rviz can be started
*/


/* =========About Copy right===============

 This code was created by modifying the original code from ROS tutorials
 The copy rights are related to the following original claim.

*/

/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor
 *     the names of its contributors may be used to endorse or promote 
 *     products derived from this software without specific prior written 
 *     permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <signal.h>
#include <termios.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// =========variables used in multiple threads or functions==========
boost::mutex mtx_update; //the access control to the flag update
bool update = false; //true: t has been pushed, false: t is not pushed.
boost::mutex mtx_myfrm; //the access control to variables of frame
tf::Transform my_frame; //the translation from local to world
boost::mutex mtx_rbtbs; //the access control to variables of frame
tf::Transform robot_base; //the translation from robot base to world
tf::Transform local_frame; //the local frame to publish
bool initialize = true; //Once my_frame is initialized, this flag becomes false
int kfd = 0;// for key board input
struct termios cooked, raw; // for key board input

// ===========================functions============================
void quit(int sig); //for reset of keyboard input at the end of the process
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom); //callback for subscribing /odom
void keyloop(); //The thread to watch "t" key

// =========================main function============================
int main(int argc, char **argv)
{
  // setup
  ros::init(argc, argv, "my_tf");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/odom", 10, odomCallback);
  signal(SIGINT,quit);

  // thread to watch keyboard
  boost::thread my_thread(&keyloop);

  // main process to publish tf
  ros::Rate loop_rate(10);
  while(ros::ok()){
    //-----publish frame----------
    if(initialize == false){
      static tf::TransformBroadcaster br;
      { //my_frame
        boost::mutex::scoped_lock lock_frm(mtx_myfrm);
        br.sendTransform(tf::StampedTransform(my_frame, ros::Time::now(), "odom", "my_frame"));
      }
      { //the local frame from myframe to robot_base
        boost::mutex::scoped_lock lock_frm(mtx_rbtbs);
        br.sendTransform(tf::StampedTransform(my_frame.inverse()*robot_base, ros::Time::now(), "my_frame", "robot_base"));
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  // finishing procedure for thread
  my_thread.interrupt();
  my_thread.join();

  return 0;
}


// ============================================================================
void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

// ============================================================================
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  double px,py,pz,rx,ry,rz,rw;
  if(update | initialize){ //when ordered by t key or at the beginning of the process
    //update my_frame
    px = odom->pose.pose.position.x;
    py = odom->pose.pose.position.y;
    pz = odom->pose.pose.position.z;
    rx = odom->pose.pose.orientation.x;
    ry = odom->pose.pose.orientation.y;
    rz = odom->pose.pose.orientation.z;
    rw = odom->pose.pose.orientation.w;
    boost::mutex::scoped_lock lock_frm(mtx_myfrm);
    my_frame.setOrigin( tf::Vector3(px, py, pz) );
    my_frame.setRotation( tf::Quaternion(rx, ry, rz, rw) );

    //uncheck
    boost::mutex::scoped_lock lock_update(mtx_update);
    update = false;

    initialize = false;
  }
  
  // receive position in robot_base
  px = odom->pose.pose.position.x;
  py = odom->pose.pose.position.y;
  pz = odom->pose.pose.position.z;
  rx = odom->pose.pose.orientation.x;
  ry = odom->pose.pose.orientation.y;
  rz = odom->pose.pose.orientation.z;
  rw = odom->pose.pose.orientation.w;
  boost::mutex::scoped_lock lock_frm(mtx_rbtbs);
  robot_base.setOrigin( tf::Vector3(px, py, pz) );
  robot_base.setRotation( tf::Quaternion(rx, ry, rz, rw) );


}

// ============================================================================
void keyloop(){ //The thread to watch "t" key
  char c;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("To update the frame, push t");


  while (ros::ok())
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    if(c == 't'){
      boost::mutex::scoped_lock lock_update(mtx_update);
      update = true;
      ROS_INFO("frame initialized");
    }

  }

  return;
}

