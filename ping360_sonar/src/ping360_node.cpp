// basic node to begin the project

#include <ping360_sonar/ping360_node.h>
#include <ping-message-common.h>
#include "ping-message-ping360.h"

using namespace std::chrono_literals;
using namespace ping360_sonar;


Ping360Sonar::Ping360Sonar(rclcpp::NodeOptions options)
  : Node("ping360", options)
{
    RCLCPP_INFO(get_logger(),"configuring");

    if(_sensor.initialize())
    {

        RCLCPP_INFO(get_logger(),"connected");
        updateSonarConfig();

        _image_publisher = create_publisher<sensor_msgs::msg::Image>("/ping360_images", _queue_size);
        _scan_publisher = create_publisher<sensor_msgs::msg::LaserScan>("/ping360_scan", _queue_size);
        _data_publisher = create_publisher<ping360_sonar_msgs::msg::SonarEcho>("/ping360_data", _queue_size);

        timer_ = this->create_wall_timer(10ms, std::bind(&Ping360Sonar::timerCallback, this));

        _cv_bridge.image = cv::Mat(_img_size, _img_size, CV_8UC1, cv::Scalar(0));
    }
    else
    {
        RCLCPP_ERROR(get_logger(),"failed to connect");
    }
}

void Ping360Sonar::getSonarData()
{
    transmitAngle(_angle);
    _data.resize(_sensor.device_data_data.data_length);
    _raw_data.resize(_sensor.device_data_data.data_length);
    for (int i = 0; i < _sensor.device_data_data.data_length; i++) {
        _data[i] = _sensor.device_data_data.data[i] / 255.0;
        _raw_data[i] = _sensor.device_data_data.data[i];
    } //find a way to use _sensor.device_data_data.data without copying it to change type
}

float Ping360Sonar::calculateRange(int index, float _samplePeriodTickDuration)
{
    return index*_speed_of_sound*_samplePeriodTickDuration*_sample_period/2;
}

int Ping360Sonar::calculateSamplePeriod(float _samplePeriodTickDuration)
{
    return int(2*_sonar_range/(_number_of_samples*_speed_of_sound*_samplePeriodTickDuration));
}

int Ping360Sonar::adjustTransmitDuration(int _firmwareMinTransmitDuration)
{
   auto duration = 8000*_sonar_range/_speed_of_sound;
   auto transmitDurationMax = std::max(int(2.5*Ping360Sonar::getSamplePeriod()/1000), int(duration));
   return std::max(_firmwareMinTransmitDuration, std::min(Ping360Sonar::getSamplePeriod(),transmitDurationMax));
}

int Ping360Sonar::transmitDurationMax(int _firmwareMaxTransmitDuration)
{
    return std::min(_firmwareMaxTransmitDuration, int(Ping360Sonar::getSamplePeriod()*64e6));
}

int Ping360Sonar::getSamplePeriod(float _samplePeriodTickDuration)
{
    return(_sample_period*_samplePeriodTickDuration);
}

void Ping360Sonar::updateSonarConfig()
{
    _sensor.set_transducer(
        1           //mode
        ,_gain          //gain_setting
        ,_angle     //angle
        ,_transmit_duration        //transmit_duration
        ,_sample_period        //sample_period
        ,_transmit_frequency        //transmit_frequency
        ,_number_of_samples        //number_of_samples
        ,1          //transmit
        ,0          //reserved
    );
    _sensor.waitMessage(Ping360Id::DEVICE_DATA, 8000);
}

void Ping360Sonar::dataSelection()
{
    for (int index =0; index<_data.size(); index++)
    {
        if ( _data[index] >= _threshold)
        {
            int distance = calculateRange(index+1);
            std::cout << distance <<std::endl;
            if(0.75 <= distance || distance <= _sonar_range)
            {
                _ranges[0] = distance;
                _intensities[0] = _data[index];
                if(_debug)
                {
                    std::cout<<"Object at "<< _angle <<" grad :" << distance << "m - " << _intensities[0]*100/255 <<"%" <<std::endl;
                }
                break;
            }
        }
    }
}

sensor_msgs::msg::Image Ping360Sonar::generateImageMsg()
{
    sensor_msgs::msg::Image msg;
    float linear_factor = float(_data.size()) / float(_center.x);
    cv::Point point;

    if(_data.size() != 0)
    {
        for(int i = 0; i < int(_center.x); i ++)
        {
            auto pointColor = _raw_data[int(i*linear_factor)]; //find a way to use _sensor.device_data_data.data

            for(int k = 0; k<= 8 * _step; k+=_step)
            {
                auto theta = 2* M_PI * (_angle + k) / 400;
                point.x = float(i) * cos(theta);
                point.y = float(i) * sin(theta);
                _cv_bridge.image.at<uint8_t>(point + _center) = pointColor;
            }
        }
    }
    else
    {
       RCLCPP_WARN(get_logger(), "an error occured, skipping image");
    }
    _cv_bridge.encoding = "mono8";
    _cv_bridge.header.set__stamp(now());
    _cv_bridge.header.set__frame_id(_frame_id);
    _cv_bridge.toImageMsg(msg);

    return (msg);
};

sensor_msgs::msg::LaserScan Ping360Sonar::generateScanMsg()
{
    sensor_msgs::msg::LaserScan msg;

    msg.header.set__stamp(now());
    msg.header.set__frame_id(_frame_id);
    msg.set__angle_min(2*M_PI*_min_angle/400);
    msg.set__angle_max(2*M_PI*_max_angle/400);
    msg.set__angle_increment(2*M_PI*_step/400);
    msg.set__time_increment(0);
    msg.set__range_min(.75);
    msg.set__range_max(_sonar_range);
    msg.set__ranges(_ranges);
    msg.set__intensities(_intensities);

    return(msg);
}

ping360_sonar_msgs::msg::SonarEcho Ping360Sonar::generateRawMsg()
{
    ping360_sonar_msgs::msg::SonarEcho msg;

    msg.header.set__stamp(now());
    msg.header.set__frame_id(_frame_id);
    msg.set__angle(_angle);
    msg.set__gain(_gain);
    msg.set__range(_sonar_range);
    msg.set__speed_of_sound(_speed_of_sound);
    msg.set__number_of_samples(_number_of_samples);
    msg.set__transmit_frequency(_transmit_frequency);
    msg.set__intensities(_raw_data); //find a way to use _sensor.device_data_data.data

    return(msg);
}

void Ping360Sonar::updateAngle()
{
    _angle +=_step * _sign;
    if (_angle >= _max_angle)
    {
        if (not _oscillate)
        {
            _angle = _min_angle;
        }
        else
        {
            _angle = _max_angle;
            _sign = -1;
        }
    }
    if (_angle <= _min_angle)
    {
        _angle = _min_angle;
        _sign = 1;
    }
}

void Ping360Sonar::transmitAngle(int angle)
{
    _sensor.set_transducer(
                1,
                _sensor.device_data_data.gain_setting,
                angle,
                _sensor.device_data_data.transmit_duration,
                _sensor.device_data_data.sample_period,
                _sensor.device_data_data.transmit_frequency,
                _sensor.device_data_data.number_of_samples,
                1,
                0);
    _sensor.waitMessage(Ping360Id::DEVICE_DATA, 8000);
}

void Ping360Sonar::timerCallback()
{
    if(_debug)
    {
        RCLCPP_INFO(get_logger(), "running");
    }

    if(_updated)
    {
        updateSonarConfig();
    }

    getSonarData();

    if(_enable_data_topic)
    {
        _data_publisher->publish(generateRawMsg());
    }

    if(_enable_scan_topic)
    {
        dataSelection();
        _scan_publisher->publish(generateScanMsg());
    }

    if(_enable_image_topic)
    {
        _image_publisher->publish(generateImageMsg());
    }

    updateAngle();
}
