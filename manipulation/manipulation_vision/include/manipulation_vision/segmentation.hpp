#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <manipulation_vision/SegmentFlowers.h>
#include <manipulation_vision/SegmentFlowersFF.h>
#include <manipulation_vision/Segment.h>
#include <fstream>

namespace manipulation
{

class Segmentation {
  public:
    Segmentation();

    bool loadLookupTableRGB ();
    bool segmentImage       (manipulation_vision::SegmentFlowers::Request  &req,
                             manipulation_vision::SegmentFlowers::Response &res);

    bool segmentImageFF     (manipulation_vision::SegmentFlowersFF::Request  &req,
                             manipulation_vision::SegmentFlowersFF::Response &res);

    ros::NodeHandle    nh;
    ros::ServiceServer segmentationServ;
    ros::ServiceServer segmentationFFServ;

  private:
    bool _setImage           (const cv::Mat & image);
    bool _assignPixelValues  ();
    bool _extractBinary      ();
    bool _openBinary         (int erode_size, int dilate_size);
    bool _extractContours    ();
    bool _extractContoursFF  ();
    bool _writeOutput        ();

    // std::vector<int>      writeBlobsToFile  (const std::vector<cv::Rect> & rectangles,
                                             // const cv::Mat               & image);

    unsigned char      ***_lut_c;
    cv::Mat               _input_image;
    cv::Mat               _segmented_image;
    cv::Mat               _binary_image;
    std::vector<cv::Rect> _contour_list;

    std::vector<manipulation_vision::Segment> _segment_list;
};

}
