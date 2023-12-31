#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate r(30);

    float f = 0.0;
    while (ros::ok())
    {

        visualization_msgs::Marker points, line_strip, line_list, line_strip2;
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = line_strip2.header.frame_id = "my_frame";
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = line_strip2.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = line_list.ns = line_strip2.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = line_strip2.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = line_strip2.pose.orientation.w = 1.0;

        visualization_msgs::Marker points8, line_strip8, line_list8, line_strip28;
        points8.header.frame_id = line_strip8.header.frame_id = line_list8.header.frame_id = line_strip28.header.frame_id = "my_frame";
        points8.header.stamp = line_strip8.header.stamp = line_list8.header.stamp = line_strip28.header.stamp = ros::Time::now();
        points8.ns = line_strip8.ns = line_list8.ns = line_strip28.ns = "points_and_lines";
        points8.action = line_strip8.action = line_list8.action = line_strip28.action = visualization_msgs::Marker::ADD;
        points8.pose.orientation.w = line_strip8.pose.orientation.w = line_list8.pose.orientation.w = line_strip28.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;
        line_strip2.id = 3;
        line_list.id = 2;

        points8.id = 4;
        line_strip8.id = 5;
        line_strip28.id = 7;
        line_list8.id = 6;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip2.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        points8.type = visualization_msgs::Marker::POINTS;
        line_strip8.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip28.type = visualization_msgs::Marker::LINE_STRIP;
        line_list8.type = visualization_msgs::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        points8.scale.x = 0.2;
        points8.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;
        line_strip2.scale.x = 0.1;

        line_strip8.scale.x = 0.1;
        line_list8.scale.x = 0.1;
        line_strip28.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;
        points8.color.g = 1.0f;
        points8.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        line_strip8.color.b = 1.0;
        line_strip8.color.a = 1.0;

        // Line strip is black
        line_strip2.color.b = 0.0;
        line_strip2.color.a = 1.0;

        line_strip28.color.b = 0.0;
        line_strip28.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        line_list8.color.r = 1.0;
        line_list8.color.a = 1.0;

        // Create the vertices for the points and lines
        for (uint32_t i = 0; i < 100; ++i)
        {
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

            //float y8 = 5 * sin(f + i / 100.0f * 2 * M_PI+M_PI/2);
            //float z8 = 5 * cos(f + i / 100.0f * 2 * M_PI+M_PI/2);

            float y8 = 5 * cos(f + i / 100.0f * 2 * M_PI);
            float z8 = 5 * sin(f + i / 100.0f * 2 * M_PI);

            geometry_msgs::Point p, p2;
            p.x = (int32_t)i - 50;
            p.y = y;
            p.z = z;
            p2.x = (int32_t)i - 50;
            p2.y = y;
            p2.z = z+1.0;

            geometry_msgs::Point p8, p28;
            p8.x = (int32_t)i - 50;
            p8.y = y8;
            p8.z = z8;
            p28.x = (int32_t)i - 50;
            p28.y = y8;
            p28.z = z8 + 1.0;

            points.points.push_back(p);
            line_strip.points.push_back(p);
            line_strip2.points.push_back(p2);

            points.points.push_back(p8);
            line_strip.points.push_back(p8);
            line_strip2.points.push_back(p28);

            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.z += 1.0;
            line_list.points.push_back(p);

            line_list8.points.push_back(p8);
            p8.z += 1.0;
            line_list8.points.push_back(p8);
        }

        marker_pub.publish(points);
        marker_pub.publish(line_strip);
        marker_pub.publish(line_list);
        marker_pub.publish(line_strip2);

        marker_pub.publish(points8);
        marker_pub.publish(line_strip8);
        marker_pub.publish(line_list8);
        marker_pub.publish(line_strip28);

        r.sleep();

        f += 0.04;
    }
}