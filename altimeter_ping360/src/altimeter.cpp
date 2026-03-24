#include <altimeter_ping360/altimeter.hpp>

using namespace ping360_sonar;

Altimeter::Altimeter()
: Node("altimeter_ping360_node") {

    std::cout << "Hello there." << std::endl;

    // Declare the parameters
    declareParamDescription("filter_center_std", 0.01f, 
                            "Filter out the signal at low distances from the sonar using a Gaussian "
                            "with this standard deviation in m", 0.001f, 10.0f);
    declareParamDescription("LoG_std", 0.1f,
                            "Standard deviation of the Laplacian of Gaussian applied to the signal",
                            0.001f, 10.0f);
    declareParamDescription("binarisation_threshold", 0.7f,
                            "Threshold for when to consider a signal as coming from the bottom",
                            0.001f, 1.0f);
    declareParamDescription("min_percentile", 0.2f,
                            "Min. detected distances from different beams are accumulated "
                            "by looking at their distribution and selecting the given percentile. "
                            "This gives more resiliance to outliers.",
                            0.001f, 1.0f);
    declareParamDescription("angle_sector", 60,
                            "See 'angle_sector' parameter of the ping360_sonar node",
                            60, 360);


    declareParamDescription("echo_topic", "/scan_echo", 
                            "The echo topic published by the Ping360 node");

    // Populate members based on parameters
    msEchoTopic = this->get_parameter("echo_topic").as_string();

    // ----- Subscribe to echo topic ----- //

    // QoS object 
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    mSubEcho = this->create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
        msEchoTopic, 
        qos,
        [this](ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
            this->echoCallback(msg);
        }
    );
}

void Altimeter::echoCallback(ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {

    // Find out if the transducer hit the edge of its angle sector and reversed. 
    // Default: false.
    bool end_turn{};

    if (mbPrevAngle) {
         // Now motion is clockwise
        if (msg->angle - mfPrevAngle > 0) {
            // Previously counterclockwise
            if (!mbPrevMotionClockwise) {
                end_turn = true;
            }
            // Set the direction indicator to what was just observed
            mbPrevMotionClockwise = true;
        }
        // Now motion is counterclockwise
        else {
            // Previously clockwise
            if (mbPrevMotionClockwise) {
                end_turn = true;
            }
            // Set the direction indicator to what was just observed
            mbPrevMotionClockwise = false;
        }
    }

    if (end_turn) std::cout << "Reversed!" << std::endl;

    // For the next callback
    mfPrevAngle = msg->angle;

    // After the first call, there is a valid previous angle
    if (!mbPrevAngle) {
        mbPrevAngle = true;
    }
    
};

void Altimeter::declareParamDescription(std::string name,
                                        float default_value,
                                        std::string description,
                                        float lower,
                                        float upper) {

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                        .set__from_value(lower)
                                        .set__to_value(upper)};
    declare_parameter<float>(name, default_value, descriptor);
}

void Altimeter::declareParamDescription(std::string name,
                                        int default_value,
                                        std::string description,
                                        int lower,
                                        int upper) {

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                        .set__from_value(lower)
                                        .set__to_value(upper)};
    declare_parameter<int>(name, default_value, descriptor);
}

void Altimeter::declareParamDescription(std::string name,
                                        std::string default_value,
                                        std::string description) {

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    declare_parameter<std::string>(name, default_value, descriptor);
}

