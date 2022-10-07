#include <ros/ros.h>
#include <acoustic_msgs/RawSonarImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher targets_pub;

float start_range = 1.0;
float end_range = 200.0;
float threshold = -75.0;

std::string csv_out;
std::ofstream csv_out_stream;

void sonarCallback(const acoustic_msgs::RawSonarImage::ConstPtr &msg)
{
  switch(msg->image.dtype)
  {
    case acoustic_msgs::SonarImageData::DTYPE_FLOAT32:
    {
      pcl::PointCloud<pcl::PointXYZI> out;
      out.is_dense = false;
      
      double sample_size = (msg->ping_info.sound_speed/msg->sample_rate)/2.0;
      int i = std::max(int(0), int((start_range/sample_size)-msg->sample0));
      double range = (i+msg->sample0)*sample_size;
      const float* samples = reinterpret_cast<const float*>(&msg->image.data.front());
      //std::cerr << i << " range: " << range << std::endl;
      while(i < msg->samples_per_beam && range < end_range)
      {
        if(samples[i] > threshold)
        {
          pcl::PointXYZI p;
          p.intensity = samples[i];
          p.x = 0.0;
          p.y = 0.0;
          p.z = range;
          out.push_back(p);
          if (csv_out_stream.is_open())
            csv_out_stream << std::fixed << std::setprecision(6) << msg->header.stamp.toSec() << ", " << std::setprecision(3) << p.z << ", " << p.intensity << std::endl;
        }
        i++;
        range = (i+msg->sample0)*sample_size;
      }
      //std::cerr << out.size() << " targets in ping" << std::endl;
      if(!out.empty())
      {
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(out, out_msg);
        out_msg.header = msg->header;
        targets_pub.publish(out_msg);
      }
      break;
    }
    default:
      ROS_WARN_STREAM_ONCE("Unsupported sonar image format: " << msg->image.dtype);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_extract");
  ros::NodeHandle nh;

  start_range = ros::param::param("~start_range", start_range);
  end_range = ros::param::param("~end_range", end_range);
  threshold = ros::param::param("~threshold", threshold);
  csv_out = ros::param::param("~csv_out", csv_out);
  if(!csv_out.empty())
    csv_out_stream.open(csv_out);

  ros::Subscriber sonar_image_sub = nh.subscribe("sonar_image", 10, &sonarCallback);
  targets_pub = nh.advertise<sensor_msgs::PointCloud2>("targets", 10);
  ros::spin();
  csv_out_stream.close();
  return 0;
}

