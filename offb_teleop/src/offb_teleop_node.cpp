#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Header.h>

int getch(void){
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Header>
        ("teleop", 1000);

    ros::Rate loop_rate(32);

    while (ros::ok()){
        int key = getch();
        if (key == 27){
            ros::shutdown();
        }
        char c = static_cast<char>(key);
        ROS_INFO("Publishing %c", c);

        std_msgs::Header msg;
        msg.frame_id = c;
        msg.stamp = ros::Time::now();
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}