#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace mocap_optitrack
{
  class MarkerArrayPublisher
  {

    public:
      MarkerArrayPublisher(ros::NodeHandle nh)
      {
        // Advertise the publisher
        marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("single_markers",1000);

        // Set the marker reference frame
        marker_.header.frame_id = "world";

        // Set the marker shape
        marker_.type = marker_.SPHERE;
        marker_.pose.orientation.x = 0.0;
        marker_.pose.orientation.y = 0.0;
        marker_.pose.orientation.z = 0.0;
        marker_.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker_.scale.x = 0.02;
        marker_.scale.y = 0.02;
        marker_.scale.z = 0.02;

        // Set the color -- be sure to set alpha to something non-zero!
        marker_.color.r = 0.0f;
        marker_.color.g = 1.0f;
        marker_.color.b = 0.0f;
        marker_.color.a = 1.0;

      }
      
      void PublishMarkers(ros::Time time_now, std::vector<Marker> other_markers)
      { 

        // Anticipate return if no unlabeled markers found
        // if(other_markers.size() == 0)
        // {
        //   // Set timestamp
        //   marker_.header.stamp = ros::Time::now();

        //   // Remove old markers using the first of the new markers
        //   marker_.action = marker_.DELETEALL;
        //   marker_.id = 0;
        //   marker_array_.markers.push_back(marker_);
        //   marker_publisher_.publish(marker_array_);
        //   marker_array_.markers.clear();

        //   return;
        // }

        // Set timestamp
        marker_.header.stamp = ros::Time::now();
        // marker_.header.stamp.setNow(time_now);

        // Remove old markers using the first of the new markers
        marker_.action = marker_.DELETEALL;
        marker_.id = 0;
        marker_.pose.position.x = other_markers[0].x;
        marker_.pose.position.y = other_markers[0].y;
        marker_.pose.position.z = other_markers[0].z;
        marker_array_.markers.push_back(marker_);
        // marker_publisher_.publish(marker_array_);
        // marker_array_.markers.clear();

        // return;
        
        // Loop through all the other markers
        marker_.action = marker_.ADD;
        for(int i = 0; i < other_markers.size(); i++)
        {
          marker_.id = i;
          if(other_markers[i].x > 500)
          {
            other_markers[i].x -= 1000;
            marker_.color.r = 1;
            marker_.color.g = 0;
          }
          else
          {
            marker_.color.r = 0;
            marker_.color.g = 1;
          }
          marker_.pose.position.x = other_markers[i].x;
          marker_.pose.position.y = other_markers[i].y;
          marker_.pose.position.z = other_markers[i].z;
          marker_array_.markers.push_back(marker_);
        }

        // Publish the rest of the new markers
        // marker_.header.stamp.setNow(time_now);
        marker_publisher_.publish(marker_array_);
        marker_array_.markers.clear();
      }

    private:
      ros::Publisher marker_publisher_;
      visualization_msgs::Marker marker_;
      visualization_msgs::MarkerArray marker_array_;

  };
}