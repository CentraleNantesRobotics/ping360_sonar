#ifndef ALTIMETER_PING360
#define ALTIMETER_PING360
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <ping360_sonar_msgs/msg/sonar_echo.hpp>

namespace ping360_sonar{

class Altimeter : public rclcpp::Node {

    public:
        Altimeter();
        
    private:

        // ---------- METHODS ---------- //
        void echoCallback(ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg);


        // ---------- UTILITY METHODS ---------- //

        // Utility method for setting parameter with min and max values. Float version
        void declareParamDescription(std::string name,
                                     float default_value,
                                     std::string description,
                                     float lower,
                                     float upper);
        // Utility method for setting parameter with min and max values. Integer version
        void declareParamDescription(std::string name,
                                     int default_value,
                                     std::string description,
                                     int lower,
                                     int upper);
        // Utility method for setting a string parameter
        void declareParamDescription(std::string name,
                                     std::string default_value,
                                     std::string description);

        // ---------- MEMBER VARIABLES ---------- //

        // Echo topic subscribed to
        std::string msEchoTopic;
        // Subscriber to the echo topic
        rclcpp::Subscription<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr mSubEcho;
        
        // Angle communicated in the most recent SonarEcho message
        float mfPrevAngle{};
        // Always true after the first SonarEcho message came in
        bool mbPrevAngle{};
        // If the transducer was previously moving clockwise. Needed to tell if the
        // Ping360 hit the end of its angle_sector and the transducer reversed direction.
        // If false, the previous motion was counter-clockwise.
        bool mbPrevMotionClockwise{};
};

}  // end namespace

#endif