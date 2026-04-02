#include <altimeter_ping360/altimeter.hpp>

using namespace ping360_sonar;

Altimeter::Altimeter(const rclcpp::NodeOptions & options)
: Node("altimeter_ping360_node", options) {

    RCLCPP_DEBUG(this->get_logger(), "Hello there. Debug logging is active");


    // ----- Parameters ----- //

    // Declare the parameters
    declareParamDescription("filter_center_std", 1.2f, 
                            "Filter out the signal at low distances from the sonar using a Gaussian "
                            "with this standard deviation in m", 0.001f, 10.0f);
    declareParamDescription("LoG_std", 0.28f,
                            "Standard deviation of the Laplacian of Gaussian applied to the signal",
                            0.001f, 10.0f);
    declareParamDescription("binarisation_threshold", 60.0f,
                            "Threshold for when to consider a signal as coming from the bottom",
                            0.0f, 255.0f);
    declareParamDescription("min_percentile", 0.75f,
                            "Min. detected distances from different beams are accumulated "
                            "by looking at their distribution and selecting the given percentile. "
                            "This gives more resiliance to outliers.",
                            0.001f, 1.0f);
    declareParamDescription("angle_sector", 20,
                            "See 'angle_sector' parameter of the ping360_sonar node",
                            0, 360);
    declareParamDescription("pub_raw_img", false,
                            "Publishes the unfiltered sonar image. However, the short distance echos have "
                            "been filtered out already as the image is very hard to read otherwise.");
    declareParamDescription("pub_altimeter_img", false,
                            "Publishes the filtered sonar image");
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

    mdBinarisationThreshold = this->get_parameter("binarisation_threshold").as_double();

    mbPubRawImg = this->get_parameter("pub_raw_img").as_bool();
    mbPubAltImg = this->get_parameter("pub_altimeter_img").as_bool();

    // QoS object 
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    mSubEcho = this->create_subscription<ping360_sonar_msgs::msg::SonarEcho>(
        msEchoTopic, 
        qos,
        [this](ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {
            this->echoCallback(msg);
        }
    );

    mFilteredImagePub = image_transport::create_publisher(this, "altimeter_image");
    mImage.set__encoding("mono8");
    mImage.set__is_bigendian(0);
    const int64_t size{this->get_parameter("debug_img_size").as_int()};
    mImage.data.resize(size*size);
    mImage.height = mImage.width = mImage.step = size;

    mPubAlt = create_publisher<std_msgs::msg::Float32>("altimeter", qos);

    mRawImagePub = image_transport::create_publisher(this, "raw_image");
}

void Altimeter::echoCallback(ping360_sonar_msgs::msg::SonarEcho::SharedPtr msg) {

    // ----- Read in the message ----- //

    // Find out if the transducer hit the edge of its angle sector and reversed. 
    // Default: false.
    bool end_turn{};

    if ((miPrevEchoMsg >= 2 && miSamplesPrevBeam != msg->intensities.size()) || msg->intensities.size() == 0) {
            RCLCPP_ERROR(this->get_logger(), "Samples in beam changed during swipe or there are none. "
                         "Wiping buffer at the end of the swipe.");
            mbMustClearBuf = true;
    }

    if (miPrevEchoMsg >= 2) {
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
        const auto last_msg = mvBufEchoMsgs.back();
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
        std_msgs::msg::Float32 altitude;
        altitude.data = computeSwipeAltitude();
        // std::cout << "Altitude " << altitude.data << " m" << std::endl;
        mPubAlt->publish(altitude);
    }
    
    // Add the current message to the buffer for the next swipe
    mvBufEchoMsgs.push_back(msg);

    // ----- Upkeep for the next iteration ----- //

    // For the next callback
    mfPrevAngle = msg->angle;
    miSamplesPrevBeam = msg->intensities.size();

    // After the first two calls to this method we can compare with the previous message
    if (miPrevEchoMsg < 2) {
        ++miPrevEchoMsg;

        // if this is the last turn of initialisaiton, we can already determine the direction of rotation
        if (miPrevEchoMsg == 2) {
            if (msg->angle - mfPrevAngle > 0) {
                mbPrevMotionClockwise = false;
            }
            else {
                mbPrevMotionClockwise = true;
            }
        }
    }
    
};


double Altimeter::computeSwipeAltitude() {
    // std::cout << "Called Altimeter::computeSwipeAltitude() with buffer length " << mvBufEchoMsgs.size() << std::endl;

    const Eigen::Index num_cols = static_cast<Eigen::Index>(mvBufEchoMsgs.size());
    const Eigen::Index num_rows = static_cast<Eigen::Index>(mvBufEchoMsgs[0]->intensities.size());
    // Spacing between samples: max_range / num_samples
    const double dx = static_cast<double>(mvBufEchoMsgs[0]->range) / num_rows;

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
        
        // Create an Eigen vector with a Gaussian pdf to attenuate the mImage.header.set__stamp(mvBufEchoMsgs.back()->header.stamp);reflective orb around the sonar
        mvAttenuationFac.resize(num_rows);
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

    // Apply attenuation factors
    mmIntensities.array().colwise() *= mvAttenuationFac.array();

    // Publish the raw image if desired
    if (mbPubRawImg) {
        pubImg();
        mImage.header.set__stamp(mvBufEchoMsgs.back()->header.stamp);
        mRawImagePub.publish(mImage);
    }

    // Apply filter on each column of the matrix
    Eigen::VectorXd tmp(num_rows);  // temporary vector for 1D convolution
    for (Eigen::Index c = 0; c < num_cols; ++c) {
        correlate1DinPlace(tmp, mmIntensities.col(c), mvLoGKernel);
        mmIntensities.col(c) = tmp;  // write new column back into matrix
    }
    // Technical threshold: negative values always cut
    mmIntensities = mmIntensities.array().max(0.0);

    normaliseTo255(mmIntensities);

    // Semantic threshold: these values are cut on the remaining 0...255 scale
    mmIntensities = mmIntensities.array().max(mdBinarisationThreshold) - mdBinarisationThreshold;

    if (mbPubAltImg) {
        pubImg();
        mImage.header.set__stamp(mvBufEchoMsgs.back()->header.stamp);
        mFilteredImagePub.publish(mImage);
    }

    // Voting for the distance
    // Boolean mask -> cast to int -> sum across columns
    Eigen::VectorXi counts =
        (mmIntensities.array() != 0.0)
            .cast<int>()
            .rowwise()
            .sum();

    const int winning_index = findPercentileRow(counts, this->get_parameter("min_percentile").as_double());

    // std::cout << "Winning index " << winning_index << std::endl;

    // Clear all messages except the most recent one. The last message of the previous swipe is the first
    // message of the next swipe
    const auto last_msg = mvBufEchoMsgs.back();
    mvBufEchoMsgs.clear();
    mvBufEchoMsgs.push_back(last_msg);

    return (winning_index + 1) * dx;
}


void Altimeter::pubImg() {

    // Normalise the current matrix
    normaliseTo255(mmIntensities);

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

    // // --- Draw distance scale arcs ---
    // const double range = static_cast<double>(mvBufEchoMsgs[0]->range);
    // const double meters_per_pixel = range / num_samples; // adjust if max_radius corresponds to range
    // const double distance_step_m = 1.0;                        // 1 meter steps
    // int num_arcs = static_cast<int>(range / distance_step_m / meters_per_pixel);

    // for (int n = 1; n <= num_arcs; ++n)
    // {
    //     double r_meters = n * distance_step_m;
    //     double r_pixels = r_meters / meters_per_pixel;

    //     // Draw circle / arc using polar coordinates
    //     for (double theta = angle_min; theta <= angle_max; theta += 0.001) // fine step
    //     {
    //         int px = static_cast<int>(center_x + r_pixels * std::sin(theta));
    //         int py = static_cast<int>(center_y - r_pixels * std::cos(theta)); // y inverted

    //         if (px >= 0 && px < miImageSize && py >= 0 && py < miImageSize)
    //         {
    //             // Optional: avoid overwriting strong intensity
    //             mImage.data[py * mImage.step + px] = 255;
    //         }
    //     }
    // }
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


int Altimeter::findPercentileRow(const Eigen::VectorXi& counts, double percentile)
{
    if (counts.size() == 0 || percentile <= 0.0)
        return 0;

    if (percentile >= 1.0)
        return counts.size() - 1;

    int total = counts.sum();
    if (total == 0)
        return -1;  // no data

    double threshold = percentile * static_cast<double>(total);

    int cumulative = 0;

    for (Eigen::Index i = 0; i < counts.size(); ++i)
    {
        cumulative += counts[i];

        if (static_cast<double>(cumulative) >= threshold)
            return static_cast<int>(i);
    }

    return counts.size() - 1;  // fallback
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