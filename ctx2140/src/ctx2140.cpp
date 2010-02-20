/* M4-ATX diagnostics and configuration node
 *
 * LGPL (c) 2010 Ken Tossell <ktossell@umd.edu>
 * underlying driver (c) Stepan Moskovchenko
 */

#include <ros/ros.h>
#include <power_msgs/PowerReading.h>
#include <usb.h>

extern "C" {
  #include "ctxapi.h"
}

class ctx2140 {
  private:
  ros::NodeHandle node;
  ros::Publisher diag_pub;
  double diag_frequency;
  double input_nominal;
  double primary;
  double secondary;

  public:
  int main (int argc, char **argv) {
    struct usb_dev_handle *dev;
  
    dev = ctxInit();
    
    if (!dev) {
      perror("Initializing CNX-P2140");
      return -1;
    }

    ros::NodeHandle node_param("~");
    node_param.param<double>("diag_frequency", diag_frequency, 1.0);
    node_param.param<double>("input", input_nominal, 13.8);
    node_param.param<double>("primary", primary, 18.5);
    node_param.param<double>("secondary", secondary, 12.0);

    diag_pub = node.advertise<power_msgs::PowerReading>("ctx2140", 1);

    ros::Rate loop_rate(diag_frequency);
    while (node.ok()) {
      ctxValues diag;
      power_msgs::PowerReading reading;
      
      if (ctxReadValues(dev, &diag)) {
	printf("Error reading from ctx2140\n");
	return -1;
      }

      reading.volts_read.push_back(diag.battVoltage);
      reading.volts_read.push_back(diag.priVoltage);
      reading.volts_read.push_back(diag.secVoltage);

      reading.current.push_back(diag.battCurrent);
      reading.current.push_back(diag.priCurrent);
      reading.current.push_back(diag.secCurrent);

      reading.volts_full.push_back(input_nominal);
      reading.volts_full.push_back(primary);
      reading.volts_full.push_back(secondary);

      reading.temperature.push_back(diag.temperature);

      reading.header.stamp = ros::Time::now();
  
      diag_pub.publish(reading);

      loop_rate.sleep();
    }

    ctxClose(dev);
    
    return 0;
  }

};
  
int main (int argc, char **argv) {
  ros::init(argc, argv, "ctx2140");

  ctx2140 m;
  return m.main(argc, argv);
}
