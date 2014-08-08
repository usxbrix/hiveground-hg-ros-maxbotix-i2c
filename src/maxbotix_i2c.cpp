#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>


#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#define RANGE_COMMAND   0x51


typedef std::pair<std::string, std::pair<int, int> > SensorInfo;

class MaxBotixI2C
{
public:
  MaxBotixI2C();
  ~MaxBotixI2C();
  void query();
  void publish();

  bool takeRangeReading(int address);
  int requestRange(int address);

private:
  ros::NodeHandle n_, n_private_;
  std::vector<SensorInfo> info_;
  std::vector<ros::Publisher> range_pub_;
  std::vector<double> range_;
  double rate_;

  std::string i2c_dev_;
  int i2c_handle_;
  uint8_t rx_buffer_[32];  // receive buffer
  uint8_t tx_buffer_[32];  // transmit buffer
};

bool compareInfoByDelay(const SensorInfo& a, const SensorInfo& b)
{
  return a.second.second < b.second.second;
}

MaxBotixI2C::MaxBotixI2C()
  : n_(), n_private_("~")
{
  n_private_.getParam("dev", i2c_dev_); //default port on Odroid-U3
  ROS_INFO("I2C device: %s", i2c_dev_.c_str());

  n_private_.getParam("rate", rate_); //default port on Odroid-U3
  ROS_INFO("Sensor reading rate: %f Hz", rate_);


  std::vector<std::string> name;
  std::vector<int> address;
  std::vector<int> trigger_delay;

  n_.getParam("/i2cxl/name", name);
  n_.getParam("/i2cxl/address", address);
  n_.getParam("/i2cxl/trigger_delay", trigger_delay);

  range_pub_.resize(name.size());
  range_.resize(name.size());
  for(int i = 0; i < name.size(); i++)
  {
    SensorInfo info;
    info.first = name[i];
    info.second.first = address[i];
    info.second.second = trigger_delay[i];
    info_.push_back(info);
    ROS_INFO("Sensor %s @ 0x%02x scan delay %d ms", info.first.c_str(), info.second.first, info.second.second);
    range_pub_[i] = n_.advertise<sensor_msgs::Range>("/i2cxl/" + name[i], 1);
  }
  std::sort(info_.begin(), info_.end(), compareInfoByDelay);

  // Create a file descriptor for the I2C bus
  i2c_handle_ = open(i2c_dev_.c_str(), O_RDWR);
  if(i2c_handle_ < 0)
  {
    ROS_ERROR("Cannot open %s : %s", i2c_dev_.c_str(), strerror(i2c_handle_));
    return;
  }

  // Tell the I2C peripheral that the device address is (or isn't) a 10-bit value. Most probably won't be.
  int result = ioctl(i2c_handle_, I2C_TENBIT, 0);
  if (result < 0)
  {
    ROS_ERROR("Cannot set I2C addressing mode");
  }
}

MaxBotixI2C::~MaxBotixI2C()
{
  if(i2c_handle_ >= 0)
  {
    close(i2c_handle_);
  }
}

void MaxBotixI2C::query()
{
  int delay = 0;
  int last_delay = 0;
  int result = -1;
  for(int i = 0; i < info_.size(); i++)
  {
    if(info_[i].second.second > last_delay)
    {
      delay = info_[i].second.second-last_delay;
      usleep(delay * 1000);
      last_delay += delay;
    }
    //trigger the sensor
    ROS_DEBUG("Trigger sensor %s @ 0x%02x scan delay %d ms", info_[i].first.c_str(), info_[i].second.first, last_delay);

    //takeRangeReading(info_[i].second.first);
  }
  ros::Rate(rate_).sleep();
  for(int i = 0; i < info_.size(); i++)
  {
    //range_[i] = requestRange(info_[i].second.first);
  }
}

void MaxBotixI2C::publish()
{

}

bool MaxBotixI2C::takeRangeReading(int address)
{
  // Tell the I2C peripheral what the address of the device is.
  int result = ioctl(i2c_handle_, I2C_SLAVE, address);
  if(result < 0)
  {
    ROS_ERROR("Cannot set I2C address to 0x%02x", address);
    return false;
  }

  tx_buffer_[0] = RANGE_COMMAND;
  result = write(i2c_handle_, tx_buffer_, 1);
  if (result != 1)
  {
    ROS_WARN("No ACK bit from 0x%02x", address);
    return false;
  }

  return true;
}

int MaxBotixI2C::requestRange(int address)
{
  // Tell the I2C peripheral what the address of the device is.
  int result = ioctl(i2c_handle_, I2C_SLAVE, address);
  if (result < 0)
  {
    ROS_ERROR("Cannot set I2C address to 0x%02x", address);
    return -1;
  }

  result = read(i2c_handle_, rx_buffer_, 2);
  if (result != 1)
  {
    ROS_WARN("No ACK bit from 0x%02x", address);
    return -1;
  }

  return *((short*)(rx_buffer_));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "maxbotix_i2c");

  MaxBotixI2C sonar;

  while (ros::ok())
  {
    ros::spinOnce();
    sonar.query();
    //sleep(1);
  }

  return 0;
}

#if 0
{
  // Set up some variables that we'll use along the way
  char rxBuffer[32];  // receive buffer
  char txBuffer[32];  // transmit buffer
  int gyroAddress = 0x68; // gyro device address
  int xlAddress = 0x53;   // accelerometer device address
  int tenBitAddress = 0;  // is the device's address 10-bit? Usually not.
  int opResult = 0;   // for error checking of operations

  // Create a file descriptor for the I2C bus
  int i2cHandle = open("/dev/i2c-2", O_RDWR);

  // Tell the I2C peripheral that the device address is (or isn't) a 10-bit
  //   value. Most probably won't be.
  opResult = ioctl(i2cHandle, I2C_TENBIT, tenBitAddress);

  // Tell the I2C peripheral what the address of the device is. We're going to
  //   start out by talking to the gyro.
  opResult = ioctl(i2cHandle, I2C_SLAVE, gyroAddress);

  // Clear our buffers
  memset(rxBuffer, 0, sizeof(rxBuffer));
  memset(txBuffer, 0, sizeof(txBuffer));

  // The easiest way to access I2C devices is through the read/write
  //   commands. We're going to ask the gyro to read back its "WHO_AM_I"
  //   register, which contains the I2C address. The process is easy- write the
  //   desired address, the execute a read command.
  txBuffer[0] = 0x00; // This is the address we want to read from.
  opResult = write(i2cHandle, txBuffer, 1);
  if (opResult != 1)
    printf("No ACK bit!\n");
  opResult = read(i2cHandle, rxBuffer, 1);
  printf("Part ID: %d\n", (int)rxBuffer[0]); // should print 105

  // Next, we'll query the accelerometer using the same process- but first,
  //   we need to change the slave address!
  opResult = ioctl(i2cHandle, I2C_SLAVE, xlAddress);
  txBuffer[0] = 0x00;  // This is the address to read from.
  opResult = write(i2cHandle, txBuffer, 1);
  if (opResult != 1)
    printf("No ACK bit!\n");
  opResult = read(i2cHandle, rxBuffer, 1);
  printf("Part ID: %d\n", (int)rxBuffer[0]); // should print 229
}
#endif
