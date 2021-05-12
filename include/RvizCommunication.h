//
// Created by ricky on 20.04.21.
//

#pragma once

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tfMessage.h>

struct Data{
    float *sensor;
    float &RSPEED;
    float &LSPEED;
};

//#define DEBUG

#define format std::fixed << std::setw(5) << std::showpos << std::setprecision(2)

class RosCommunication {

public:
    RosCommunication(ros::NodeHandle& node, const std::string& stopic,const std::string& ptopic, float freq,Data &data) : node_{node},
                                                                                                                          iData{data} {
        subscriber_ = node.subscribe(stopic, 0, &RosCommunication::subscriber_callback, this);
        timer_ = node.createTimer(freq, &RosCommunication::timer_callback, this);
        markers_publisher_ = node.advertise<visualization_msgs::MarkerArray>(ptopic, 0);
    }

private:

    double x = 0;
    double y = 0;
    double z = 0;

    ros::NodeHandle &node_;
    ros::Subscriber subscriber_;
    ros::Timer timer_;
    ros::Publisher markers_publisher_;

    Data iData;

    void timer_callback(const ros::TimerEvent& event) {
        auto time = ros::Time::now().toSec();

        visualization_msgs::MarkerArray msg;
        msg.markers.push_back(make_text_marker(x,y,z));
        markers_publisher_.publish(msg);
    }

    void subscriber_callback(const tf::tfMessage msg) {
        if (msg.transforms[0].child_frame_id == "robot"){
            x = msg.transforms[0].transform.translation.x;
            y = msg.transforms[0].transform.translation.y;
            z = msg.transforms[0].transform.translation.z+0.1;
#ifdef DEBUG
            std::cout << "Translace X: " << msg.transforms[0].transform.translation.x << std::endl
                  << "Translace Y: " << msg.transforms[0].transform.translation.y << std::endl
                  << "Translace Z: " << msg.transforms[0].transform.translation.z << std::endl;
#endif
        }

    }

    visualization_msgs::Marker make_text_marker(double ax, double ay, double az) {
        visualization_msgs::Marker text;

        // Coordination system
        text.header.frame_id = "origin";

        // Timestamp
        text.header.stamp = ros::Time();

        // Marker Type
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.id = 1;

        // Position
        text.pose.position.x = ax;
        text.pose.position.y = ay;
        text.pose.position.z = az+0.4;

        // Size
        text.scale.z = 0.05;

        // Text
        std::stringstream stream;
        stream << "  R_speed " << format << iData.RSPEED << std::endl
               << "  L_speed " << format << iData.LSPEED << std::endl
               << "  Senzor 1: " << format << iData.sensor[0] << std::endl
               << "  Senzor 2: " << format << iData.sensor[1] << std::endl
               << "  Senzor 3: " << format << iData.sensor[2] << std::endl
               << "  Senzor 4: " << format << iData.sensor[3] << std::endl
               << "  Senzor 5: " << format << iData.sensor[4];


        //TODO: Opravit vizualizaci
        //for (size_t i = 0; i < (sizeof(*iData.sensor)/sizeof(iData.sensor));i++) {
        //    stream << "  Senzor "<< i+1 <<" : " << format << iData.sensor[0] << std::endl;
        //}


        text.text = stream.str();

        // Color
        text.color.a = 1.0; // alpha - visibility
        text.color.r = 0.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        return text;
    }

};