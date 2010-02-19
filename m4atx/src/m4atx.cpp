/* M4-ATX diagnostics and configuration node
 *
 * LGPL (c) 2010 Ken Tossell <ktossell@umd.edu>
 */

#include <ros/ros.h>
#include <power_msgs/PowerReading.h>

extern "C" {
  #include "m4api.h"
}

class m4atx {
  private:
  ros::NodeHandle node;
  ros::Publisher diag_pub;
  double diag_frequency;
  double input_nominal;

  public:
  int main (int argc, char **argv) {
    struct usb_dev_handle *dev;
  
    dev = m4Init();
    
    if (!dev) {
      perror("Initializing M4-ATX");
      return -1;
    }

    ros::NodeHandle node_param("~");
    node_param.param<double>("diag_frequency", diag_frequency, 1.0);
    node_param.param<double>("input_nominal", input_nominal, 13.8);

    diag_pub = node.advertise<power_msgs::PowerReading>("m4atx", 1);

    ros::Rate loop_rate(diag_frequency);
    while (node.ok()) {
      m4Diagnostics diag;
      power_msgs::PowerReading reading;
      
      if (m4GetDiag(dev, &diag)) {
        perror("Reading from M4-ATX");
	return -1;
      }
    
      reading.volts_read.push_back(diag.vin);
      reading.volts_read.push_back(diag.v12);
      reading.volts_read.push_back(diag.v5);
      reading.volts_read.push_back(diag.v33);

      reading.volts_full.push_back(input_nominal);
      reading.volts_full.push_back(12.0);
      reading.volts_full.push_back(5.0);
      reading.volts_full.push_back(3.3);

      reading.temperature.push_back(diag.temp);

      reading.header.stamp = ros::Time::now();
  
      diag_pub.publish(reading);

      loop_rate.sleep();
    }
    
    return 0;
  }

};
  
int main (int argc, char **argv) {
  ros::init(argc, argv, "m4atx");

  m4atx m;
  return m.main(argc, argv);
}
