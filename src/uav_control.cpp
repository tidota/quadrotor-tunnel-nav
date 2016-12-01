// uav_control.cpp
// 161130
// this is the control code for UAV in the simulated enviornment by gazebo.
// 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#include "sub.hpp"

#define KEYCODE_R 0x6C 
#define KEYCODE_L 0x6A
#define KEYCODE_U 0x69
#define KEYCODE_D 0x6D
#define KEYCODE_S 0x6B
#define KEYCODE_SPACE 0x20


class UAV_Control
{
public:
  UAV_Control();
  //void keyLoop();
  void command();

private:
  ros::NodeHandle nh_;//,ph_;
  //double linear_, angular_;
  //ros::Time first_publish_;
  //ros::Time last_publish_;
  //double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  //void publish(double, double);
  //boost::mutex publish_mutex_;

  bool f_up;
};

UAV_Control::UAV_Control():
  f_up(true)
  //ph_("~"),
  //linear_(0),
  //angular_(0),
  //l_scale_(1.0),
  //a_scale_(1.0)
{
  //ph_.param("scale_angular", a_scale_, a_scale_);
  //ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{

  subfunc(0);

  ros::init(argc, argv, "uav_control");
  UAV_Control uav_control;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  //boost::thread my_thread(boost::bind(&UAV_Control::keyLoop, &uav_control));
  
  
  ros::Timer timer = n.createTimer(ros::Duration(4.0), boost::bind(&UAV_Control::command, &uav_control));

  //ros::spin();

  //while(ros::ok())
  //{
  //  ros::spinOnce();
  //}
  printInTerm("before");
  timer.stop();
  printInTerm("after");

  //my_thread.interrupt() ;
  //my_thread.join() ;
      
  return(0);
}

void UAV_Control::command()
{
/*
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, linear_);
*/
  geometry_msgs::Twist vel;

  if(f_up)
  {
    vel.linear.z = 0.5;
    f_up = false;
    printInTerm("going up");
  }
  else
  {
    vel.linear.z = -0.5;
    f_up = true;
    printInTerm("going down");
  }

  vel_pub_.publish(vel);    

}

/*
void UAV_Control::keyLoop()
{
  char c;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use keyboard to move the turtlebot.");
  puts("---------------------------");
  puts("i,m: increase / decrease forward velocity");
  puts("j,l: bump the robot's angle to the left / right");
  puts("k, space: stop the robot");
  puts("---------------------------");


  while (ros::ok())
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    
    angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 0.5;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -0.5;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ += 0.025;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ += -0.025;
        break;
      case KEYCODE_SPACE:
      case KEYCODE_S:
        ROS_DEBUG("STOP");
        linear_ = 0.0;
	angular_ = 0.0;
        break;

    }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) { 
      first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish(angular_, linear_);
  }

  return;
}

void UAV_Control::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;

    vel_pub_.publish(vel);    


  return;
}

*/
