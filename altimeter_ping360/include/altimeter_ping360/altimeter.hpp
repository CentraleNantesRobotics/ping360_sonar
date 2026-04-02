#ifndef ALTIMETER_PING360
#define ALTIMETER_PING360
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <ping360_sonar_msgs/msg/sonar_echo.hpp>
#include "std_msgs/msg/float32.hpp"
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace ping360_sonar{

class Altimeter : public rclcpp::Node {

    public:
        Altimeter(const rclcpp::NodeOptions & options);
        
    private:

        // ---------- METHODS ---------- //
        
        // Callback for SonarEcho message.
        void echoCallback(ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg);
        // Computes the altitude of the UUV across one swipe
        double computeSwipeAltitude();
        // Fillst the member mImage based on what is in the buffer and the mmIntensities matrix
        // Should only be called when the buffer is filled and corresponds to the values in mmIntensities
        void pubImg();

        // ---------- UTILITY METHODS ---------- //

        // Compute Laplacian of Gaussian kernel based on parameters
        static Eigen::VectorXd sampleLoG(double sigma, double dx, double threshold);
        // Do a 1D-vector-correlation in place
        static void correlate1DinPlace(Eigen::VectorXd& out, const Eigen::VectorXd& x, const Eigen::VectorXd& k);
        // In-place normalisation of a matrix. Needed for visualising the results after filtering before converting to uint8_t
        static void normaliseTo255(Eigen::MatrixXd& mat);
        // Finding the nth percentile of a historgram of indices
        static int findPercentileRow(const Eigen::VectorXi& counts, double percentile);

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
        // Utility method for setting a boolean parameter
        void declareParamDescription(std::string name,
                                     bool default_value,
                                     std::string description);

        // Convert degrees to radients the way the ping360_sonar package suggests
        // maybe these are steps of a stepper motor rather than conventional degrees
        static inline float grad2rad(int grad)
        {
            return (2*M_PI*grad)/400;
        }

        // ---------- MEMBER VARIABLES ---------- //

        // Echo topic subscribed to
        std::string msEchoTopic;
        // Subscriber to the echo topic
        rclcpp::Subscription<ping360_sonar_msgs::msg::SonarEcho>::SharedPtr mSubEcho;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mPubAlt;
        // The standard deviation of the Gaussian used for attenuating the signal at low distances
        double mfFilterCenterStd;
        // For the image publishers
        image_transport::Publisher mFilteredImagePub;
        image_transport::Publisher mRawImagePub;
        sensor_msgs::msg::Image mImage;
        int miImageSize;
        // Binarisation parameter
        double mdBinarisationThreshold;

        // Buffer of SonarEcho messages. Emptied at the end of each swipe
        std::vector<ping360_sonar_msgs::msg::SonarEcho::SharedPtr> mvBufEchoMsgs;

        // Variable checking the number of samples over a beam
        std::size_t miSamplesPrevBeam{};
        // Variable indicating if the number of samples changed during the swipe and the 
        // buffer has to be cleared
        bool mbMustClearBuf{};
        
        // Angle communicated in the most recent SonarEcho message
        float mfPrevAngle{};
        // Integer used as a flag. After 2 iteraitons of echoCallback, its value is 2 and
        // from this point on messages are compared with past messages
        int miPrevEchoMsg{};
        // If the transducer was previously moving clockwise. Needed to tell if the
        // Ping360 hit the end of its angle_sector and the transducer reversed direction.
        // If false, the previous motion was counter-clockwise.
        bool mbPrevMotionClockwise{};

        // Eigen matrix for processing the sensor information. Resized on first use.
        Eigen::MatrixXd mmIntensities;
        // Eigen vector with samples from Gaussian pdf. Used for attenuating the signal
        Eigen::VectorXd mvAttenuationFac;
        // Laplacian of Gaussian kernel for convolution
        Eigen::VectorXd mvLoGKernel;

        // Angular resolution of each beam in degrees
        int miAngleStep;
        // If the unfiltered sonar image should be published
        bool mbPubRawImg{};
        // If the filtered sonar image should be published
        bool mbPubAltImg{};
};

}  // end namespace

#endif