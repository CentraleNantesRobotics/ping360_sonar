#include <altimeter_ping360/altimeter.hpp>

using namespace ping360_sonar;

Altimeter::Altimeter(const rclcpp::NodeOptions & options)
: Node("altimeter_ping360_node", options) {

    RCLCPP_DEBUG(this->get_logger(), "Hello there. Debug logging is active");


    // ----- Parameters ----- //

    // Declare the parameters
    declareParamDescription("filter_center_std", 0.5f, 
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
    declareParamDescription("thresholded_sonar_image", true,
                            "Displays the sonar image as in the ping360_sonar node, but highlighting "
                            "those pixels that are above the threshold. Mainly for debugging and "
                            "parameter tuning.");
    // declareParamDescription("debug_img", true,
    //                         "If there should be a debugging image");
    declareParamDescription("debug_img_size", 300,
                            "Size of the debugging image", 100, 500);
    declareParamDescription("angle_step", 4,
                            "Gives the angular resolution. Should agree with what the ping360_sonar node uses "
                            "otherwise the debug image visualisaiton will not be consistent. Angle in degrees.", 1, 20);
    declareParamDescription("echo_topic", std::string("/scan_echo"), 
                            "The echo topic published by the Ping360 node");

    // ----- Populate members based on parameters ----- //
    
    msEchoTopic = this->get_parameter("echo_topic").as_string();

    mfFilterCenterStd = this->get_parameter("filter_center_std").as_double();

    miAngleStep = this->get_parameter("angle_step").as_int();

    miImageSize = this->get_parameter("debug_img_size").as_int();

    // Find places to evaluate the Gaussian for dampening
    // -> The Gaussian damping is done by a Hadamard product of a Gauss-pdf vector of the same size as the echo vector
    
    // Convert Ga

    // Find places to evaluate the LoG filter 
    // -> LoG filter has a different dimensionality depending on parameters
    // -> Have dimensionality adapt to the std. At some point towards the margins the value is basically zero

    // QoS object 
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    mSubEcho = this->create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
        msEchoTopic, 
        qos,
        [this](ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
            this->echoCallback(msg);
        }
    );

    mImagePub = image_transport::create_publisher(this, "altimeter_image");
    mImage.set__encoding("mono8");
    mImage.set__is_bigendian(0);
    const int64_t size{this->get_parameter("debug_img_size").as_int()};
    mImage.data.resize(size*size);
    mImage.height = mImage.width = mImage.step = size;
}

void Altimeter::echoCallback(ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {

    // ----- Read in the message ----- //

    // Find out if the transducer hit the edge of its angle sector and reversed. 
    // Default: false.
    bool end_turn{};

    if ((mbPrevEchoMsg && miSamplesPrevBeam != msg->intensities.size()) || msg->intensities.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "Samples in beam changed during swipe or there are none. "
                         "Wiping buffer at the end of the swipe.");
            mbMustClearBuf = true;
    }

    if (mbPrevEchoMsg) {
         // Now motion is counterclockwise
        if (msg->angle - mfPrevAngle > 0) {
            // Previously clockwise
            if (mbPrevMotionClockwise) {
                end_turn = true;
            }
            // Set the direction indicator to what was just observed
            mbPrevMotionClockwise = false;
        }
        // Now motion is clockwise (no motion (0 angle) is considered counterclockwise)
        else {
            // Previously counterclockwise
            if (!mbPrevMotionClockwise) {
                end_turn = true;
            }
            // Set the direction indicator to what was just observed
            mbPrevMotionClockwise = true;
        }
    }

    if (end_turn) RCLCPP_DEBUG(this->get_logger(), "Reversed!");

    // ----- Manage buffer ----- //

    if (end_turn && mbMustClearBuf){ 
        // we reached the end of the swipe but the dimensionalities changed. We reset the buffer
        // but do not call computeSwipeAltitude()
        auto last_msg = mvBufEchoMsgs.back();
        mvBufEchoMsgs.clear();
        mvBufEchoMsgs.push_back(last_msg);      // push message from last iteration

        // if the previous and the current dimensionalities are the same, we can reset the flag
        if (miSamplesPrevBeam == msg->intensities.size() && msg->intensities.size() != 0) {
            mbMustClearBuf = false;
        }
    }
    else if (end_turn) {
        // The previous message was the last one of the swipe and we can compute the altitude with our
        // current buffer
        double altitude = computeSwipeAltitude();
    }
    
    // Add the current message to the buffer for the next swipe
    mvBufEchoMsgs.push_back(msg);

    // ----- Upkeep for the next iteration ----- //

    // For the next callback
    mfPrevAngle = msg->angle;
    miSamplesPrevBeam = msg->intensities.size();

    // After the first call to this method we can compare with the previous message
    if (!mbPrevEchoMsg) {
        mbPrevEchoMsg = true;
    }
    
};


double Altimeter::computeSwipeAltitude() {
    std::cout << "Called Altimeter::computeSwipeAltitude() with buffer length " << mvBufEchoMsgs.size() << std::endl;

    const Eigen::Index num_cols = static_cast<Eigen::Index>(mvBufEchoMsgs.size());
    const Eigen::Index num_rows = static_cast<Eigen::Index>(mvBufEchoMsgs[0]->intensities.size());

    // ----- Construct Eigen matrix ----- //

    // Resize Eigen matrix if needed
    if (mmIntensities.cols() != num_cols || mmIntensities.rows() != num_rows)
        mmIntensities.resize(num_rows, num_cols);

    // Fill in matrix. Copies to preserve orignal values for plotting the image
    // To make sure negative angles stay on the left and postive ones on the right, 
    // the direction in which the matrix is filled alternates between swipes
    if (mvBufEchoMsgs.back()->angle - mvBufEchoMsgs[0]->angle < 0) {
        for (Eigen::Index j = 0; j < num_cols; ++j) {
            for (Eigen::Index i = 0; i < num_rows; ++i) {
                mmIntensities(i, j) = static_cast<double>(mvBufEchoMsgs[j]->intensities[i]);
            }
        }
    }
    else {
        for (Eigen::Index j = 0; j < num_cols; ++j) {
            for (Eigen::Index i = 0; i < num_rows; ++i) {
                // Only difference: what column we fill in
                mmIntensities(i, num_cols - 1 - j) = static_cast<double>(mvBufEchoMsgs[j]->intensities[i]);
            }
        }
    }
    // ---- New quantities for processing data if needed ----- //

    if (mvAttenuationFac.size() != num_rows) {  // check if nr. samples changed
        
        // Create an Eigen vector with a Gaussian pdf to attenuate the reflective orb around the sonar
        mvAttenuationFac.resize(num_rows);
        // Spacing between samples: max_range / num_samples
        const double dx = static_cast<double>(mvBufEchoMsgs[0]->range) / num_rows;
        // The x value that the pdf is sampled at
        Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(
            num_rows, 
            dx, 
            static_cast<double>(mvBufEchoMsgs[0]->range)
        );
        // Want to attenuate meaurements close to the device, far away should be factor 1.
        mvAttenuationFac = 1 - (-0.5 * x.array().square() 
                                / 
                                (mfFilterCenterStd * mfFilterCenterStd)).exp();

        // New LoG kernel since the scale dx may have changed.
        // Multiply by -1 since the bottom (positive detection) shows as neg, pos, neg on the sonar
        mvLoGKernel = -1 * sampleLoG(
            this->get_parameter("LoG_std").as_double(),       // sigma
            dx,
            0.1                                               // threshold where kernel stops
        );

    }

    pubImg();

    // Apply attenuation factors
    mmIntensities.array().colwise() *= mvAttenuationFac.array();

    // Apply filter on each column of the matrix
    // Eigen::VectorXd tmp(mmIntensities.rows());  // temporary vector for 1D convolution
    // for (Eigen::Index c = 0; c < mmIntensities.cols(); ++c) {
    //     correlate1DinPlace(tmp, mmIntensities.col(c), mvLoGKernel);
    //     mmIntensities.col(c) = tmp;  // write new column back into matrix
    // }

    // Clear all messages except the most recent one. The last message of the previous swipe is the first
    // message of the next swipe
    auto last_msg = mvBufEchoMsgs.back();
    mvBufEchoMsgs.clear();
    mvBufEchoMsgs.push_back(last_msg);

    return 1.0;
}


void Altimeter::pubImg() {

    // Wipe previous image
    std::fill(mImage.data.begin(), mImage.data.end(), 0);

    const int num_samples = mmIntensities.rows();
    const int num_beams   = mmIntensities.cols();
    const int center_x = miImageSize / 2;
    const int center_y = miImageSize;   // bottom-center → fan goes upward
    const double max_radius = static_cast<double>(miImageSize);

    // Compute the width of the beams by looking at the difference in
    // angle for the first and last SonarEcho message in the buffer
    const double swipe_angle_diff = fabs(mvBufEchoMsgs[0]->angle - mvBufEchoMsgs.back()->angle);
    const double beam_width_rad = swipe_angle_diff / (num_beams - 1);

    // Init image
    mImage.height = miImageSize;
    mImage.width  = miImageSize;
    mImage.encoding = "mono8";
    mImage.step = miImageSize;
    mImage.data.assign(miImageSize * miImageSize, 0);

    const double total_angle = num_beams * beam_width_rad;
    const double angle_min = -total_angle / 2.0;
    const double angle_max =  total_angle / 2.0;

    // Loop over all pixels (inverse mapping)
    for (int py = 0; py < miImageSize; ++py)
    {
        for (int px = 0; px < miImageSize; ++px)
        {
            // Cartesian relative to sonar origin
            double x = px - center_x;
            double y = center_y - py;  // flip axis

            double r = std::sqrt(x*x + y*y);

            // Reject outside radius
            if (r <= 0.0 || r >= max_radius)
                continue;

            double theta = std::atan2(x, y);  // note: swapped for vertical fan

            // Reject outside fan angle
            if (theta < angle_min || theta > angle_max)
                continue;

            // Map to indices
            int j = static_cast<int>((theta - angle_min) / beam_width_rad);
            int i = static_cast<int>((r / max_radius) * num_samples);

            // Bounds safety
            if (i < 0 || i >= num_samples || j < 0 || j >= num_beams)
                continue;

            uint8_t value = static_cast<uint8_t>(mmIntensities(i, j));

            mImage.data[py * mImage.step + px] = value;
        }
    }

    mImage.header.set__stamp(mvBufEchoMsgs.back()->header.stamp);
    std::cout << "Publised an image!" << std::endl;
    mImagePub.publish(mImage);
}


void Altimeter::correlate1DinPlace(Eigen::VectorXd& out,
                                   const Eigen::VectorXd& x,
                                   const Eigen::VectorXd& k)
{
    int n = x.size();
    int m = k.size();
    int half = m / 2;

    for (int i = 0; i < n; ++i) {
        double acc = 0.0;
        for (int j = 0; j < m; ++j) {
            int idx = i + j - half;
            if (idx >= 0 && idx < n)
                acc += x(idx) * k(j);
        }
        out(i) = acc;
    }
}


Eigen::VectorXd Altimeter::sampleLoG(double sigma,
                                     double dx,
                                     double threshold)
{
    if (sigma <= 0.0 || dx <= 0.0 || threshold <= 0.0)
        return Eigen::VectorXd();

    // const double norm = 1.0 / (sigma * std::sqrt(2.0 * M_PI));

    std::vector<double> values;

    // Always include x = 0
    {
        double x = 0.0;
        double val = ((x * x - sigma * sigma) / std::pow(sigma, 4)) *
                     std::exp(-0.5 * (x * x) / (sigma * sigma));
        values.push_back(val);
    }

    // Expand symmetrically
    int k = 1;
    while (true)
    {
        double x = k * dx;

        double val = ((x * x - sigma * sigma) / std::pow(sigma, 4)) *
                     std::exp(-0.5 * (x * x) / (sigma * sigma));

        // If we are more than 2 STDs out, we stop when the abs. val. is below the threshold
        if (std::abs(val) < threshold && x > 2 * sigma)
            break;

        // Insert symmetric values
        values.insert(values.begin(), val);  // -x
        values.push_back(val);               // +x

        ++k;
    }

    // Convert to Eigen vector
    Eigen::VectorXd result(values.size());
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(values.size()); ++i)
    {
        result[i] = values[i];
    }

    return result;
}


void Altimeter::normaliseTo255(Eigen::MatrixXd& mat)
{
    if (mat.size() == 0)
        return;

    double min_val = mat.minCoeff();
    double max_val = mat.maxCoeff();

    // Avoid division by zero (constant matrix)
    if (max_val == min_val)
    {
        mat.setZero();
        return;
    }

    mat.array() -= min_val;
    mat.array() /= (max_val - min_val);
    mat.array() *= 255.0;
}


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


void Altimeter::declareParamDescription(std::string name,
                                        bool default_value,
                                        std::string description) {

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.set__name(name).set__description(description);
    declare_parameter<bool>(name, default_value, descriptor);
}