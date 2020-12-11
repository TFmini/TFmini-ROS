#include <TFmini.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "tfmini_ros_node");
  ros::NodeHandle nh("~");
  std::string frame_id;
  std::string portName;
  int baud_rate;
  double loop_rate;
  benewake::TFmini *tfmini_obj;

  nh.param("serial_port", portName, std::string("/dev/ttyUSB0"));
  nh.param("baud_rate", baud_rate, 115200);
  nh.param<std::string>("frame_id", frame_id, "TFmini");
  nh.param("loop_rate", loop_rate, 100.);

  tfmini_obj = new benewake::TFmini(portName, baud_rate);
  ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>("range", 100, true);
  sensor_msgs::Range TFmini_range;
  TFmini_range.radiation_type = sensor_msgs::Range::INFRARED;
  TFmini_range.field_of_view = 0.0628319;
  TFmini_range.min_range = 0.1;
  TFmini_range.max_range = 12;
  TFmini_range.header.frame_id = frame_id;
  float dist = 0;
  ros::Rate r(loop_rate);
  ROS_INFO_STREAM("Start processing ...");

  while (ros::master::check() && ros::ok()) {
    ros::spinOnce();
    dist = tfmini_obj->getDist();
    TFmini_range.header.stamp = ros::Time::now();
    if (dist > 0 && dist < TFmini_range.max_range) {
      TFmini_range.range = dist;
    } else if (dist == -1.0) {
      ROS_ERROR_STREAM("Failed to read data. TFmini ros node stopped!");
      break;
    } else if (dist == 0.0) {
      TFmini_range.range = std::numeric_limits<double>::infinity();;
      ROS_WARN_STREAM_THROTTLE(5., "Data validation error!");
    }
    pub_range.publish(TFmini_range);
    r.sleep();
  }

  tfmini_obj->closePort();
}
