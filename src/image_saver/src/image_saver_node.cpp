#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

using namespace message_filters;
using namespace sensor_msgs;
using namespace cv;
using namespace std;

class ImageSaver
{
public:
    ImageSaver(const string &rgb_path, const string &depth_path, const string &output_file)
        : rgb_path_(rgb_path), depth_path_(depth_path), output_file_(output_file)
    {
        // Ensure the directories exist
        // cv::utils::fs::createDirectory(rgb_path_);
        // cv::utils::fs::createDirectory(depth_path_);
        fs::create_directory(rgb_path_);
        fs::create_directory(depth_path_);
        // Open the output file
        output_file_stream_.open(output_file_.c_str());
    }

    ~ImageSaver()
    {
        if (output_file_stream_.is_open())
        {
            output_file_stream_.close();
        }
    }

    void callback(const ImageConstPtr &rgb_msg, const ImageConstPtr &depth_msg)
    {
        // Convert ROS image messages to OpenCV images
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        cv_bridge::CvImageConstPtr cv_ptrD;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(rgb_msg, "bgr8");
            cv_ptrD = cv_bridge::toCvShare(depth_msg);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Compose file names using the timestamp
        stringstream ssrgb;
        stringstream ssdep;
        ssrgb << fixed << setprecision(2) << rgb_msg->header.stamp.toSec();
        ssdep << fixed << setprecision(2) << depth_msg->header.stamp.toSec();
        string rgb_filename = rgb_path_ + "/" + ssrgb.str() + ".png";
        string depth_filename = depth_path_ + "/" + ssdep.str() + ".png";
        string txt_path_rgb = "rgb/" + ssrgb.str() + ".png";
        string txt_path_depth = "depth/" + ssdep.str() + ".png";

        // 设置 PNG 压缩级别为 0 (无压缩)
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(0);

        // 保存图像时使用参数
        imwrite(rgb_filename, cv_ptrRGB->image, compression_params);
        imwrite(depth_filename, cv_ptrD->image, compression_params);
        // // Save images
        // imwrite(rgb_filename, cv_ptrRGB->image);
        // imwrite(depth_filename, cv_ptrD->image);

        // Write to the output file
        if (output_file_stream_.is_open())
        {
            output_file_stream_ << rgb_msg->header.stamp << " " << txt_path_rgb << " "
                                << depth_msg->header.stamp << " " << txt_path_depth << endl;
        }
    }

private:
    string rgb_path_;
    string depth_path_;
    string output_file_;
    ofstream output_file_stream_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_saver_node");
    ros::NodeHandle nh;

    string rgb_path = "/usr/local/project/keystar/rgbd_data/test3/rgb";
    string depth_path = "/usr/local/project/keystar/rgbd_data/test3/depth";
    string output_file = "/usr/local/project/keystar/rgbd_data/test3/timestamps_and_filenames.txt";

    ImageSaver image_saver(rgb_path, depth_path, output_file);

    Subscriber<Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    Subscriber<Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageSaver::callback, &image_saver, _1, _2));

    ros::spin();

    return 0;
}
