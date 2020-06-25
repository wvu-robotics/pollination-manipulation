#include <manipulation_vision/segmentation.hpp>

namespace manipulation
{

//----------------------------------------------------------------------------
/**
 * Constructor
 */
Segmentation::Segmentation()
{
    segmentationServ = nh.advertiseService("segment_image", &Segmentation::segmentImage, this);
    segmentationFFServ = nh.advertiseService("segment_image_ff", &Segmentation::segmentImageFF, this);
}

//----------------------------------------------------------------------------
/**
 * This function loads the RGB LUT from file. To generate or modify the RGB
 * LUT, see the generate_lookup.cpp.
 */
bool Segmentation::loadLookupTableRGB()
{
    //get path to lookup table
    std::string filepath = ros::package::getPath("manipulation_vision") + "/data/lookup_table.csv";

    //open file containing lookup table
    std::ifstream inputFile;
    inputFile.open(filepath.c_str());
    if(!inputFile)
    {
        std::cout << "Error! Failed to open file " << filepath.c_str() << "!" << std::endl;
        std::cout << "Try running generate_lookup..." << std::endl;
        return -1;
    }
    else
    {
      std::cout << "File " << filepath.c_str() << " open success!" << std::endl;
    }

    //allocate memory for lookup table
    _lut_c = new unsigned char**[256];
    for (int i = 0; i < 256; ++i)
    {
        _lut_c[i] = new unsigned char*[256];
        for (int j = 0; j < 256; ++j)
        {
            _lut_c[i][j] = new unsigned char[256];
        }
    }

    //load data in the lookup table
    int element = 0;
    for (int i=0; i<256; i++) //R
    {
        for (int j=0; j<256; j++) //G
        {
            for (int k=0; k<256; k++) //B
            {
                if(inputFile.eof())
                {
                    std::cout << "Error! Unexpected end of file while accessing element (" << i << "," << j << ") in file " << filepath <<  "." << std::endl;
                    return -2;
                }
                else
                {
                    inputFile >> element;
                    _lut_c[i][j][k] = element;
                }
            }
        }
    }

    //close file
    inputFile.close();

    return true;
}

//----------------------------------------------------------------------------
/**
 * Server for performing segmentation on image (this node should call the
 * function/server that captures the image or it should be provided in
 * the request; however, for now, it just loads the image saved to file)
 */
bool Segmentation::segmentImage(manipulation_vision::SegmentFlowers::Request  &req,
                                manipulation_vision::SegmentFlowers::Response &res)
{
    //temporary load and segment image then save results
    // std::string filepath = ros::package::getPath("manipulation_vision") + "/data/example.jpg";
    std::string filepath = ros::package::getPath("manipulation_vision") + "/data/rgb_constrained.jpg";

    //set image (need to update to use realsense camera)
    cv::Mat image = cv::imread(filepath);
    _setImage(image);

    //segment image
    _assignPixelValues();
    _extractBinary();
    _openBinary(4,4);
    _extractContours();

    //write output
    _writeOutput();

    //copy points inside each contour (using flood fill algorithm)
    std::vector<int> U;
    std::vector<int> V;
    for(int i=0; i< _contour_list.size(); i++)
    {
      //for each contour
    }

    //update response
    res.u.clear();
    res.v.clear();
    res.width.clear();
    res.height.clear();
    for(int i=0; i< _contour_list.size(); i++)
    {
        res.u.push_back(_contour_list[i].tl().x + _contour_list[i].width/2.0);
        res.v.push_back(_contour_list[i].tl().y + _contour_list[i].height/2.0);
        res.width.push_back(_contour_list[i].width);
        res.height.push_back(_contour_list[i].height);
    }
    res.numberOfSegments = _contour_list.size();
    res.success = true;

    return true;
}

//----------------------------------------------------------------------------
/**
 * Server for performing segmentation on image (this node should call the
 * function/server that captures the image or it should be provided in
 * the request; however, for now, it just loads the image saved to file).
 *
 * The different between segmentImageFF and segmentImage is that
 * segmentImageFF uses a flood fill algorithm to extract all the pixels inside
 * each contour instead of just returning the contours.
 */
bool Segmentation::segmentImageFF(manipulation_vision::SegmentFlowersFF::Request  &req,
                                  manipulation_vision::SegmentFlowersFF::Response &res)
{
    ROS_INFO("Running segmentImageFF");
    //temporary load and segment image then save results
    // std::string filepath = ros::package::getPath("manipulation_vision") + "/data/example.jpg";
    std::string filepath = ros::package::getPath("manipulation_vision") + "/data/rgb_constrained.jpg";

    //set image (need to update to use realsense camera)
    cv::Mat image = cv::imread(filepath);
    _setImage(image);

    //segment image
    _assignPixelValues();
    _extractBinary();
    _openBinary(4,4);
    _extractContoursFF();

    //write output
    _writeOutput();

    //update response
    res.segments.clear();
    res.u.clear();
    res.v.clear();
    res.width.clear();
    res.height.clear();
    for(int i=0; i< _contour_list.size(); i++)
    {
        res.segments.push_back(_segment_list[i]);
        res.u.push_back(_contour_list[i].tl().x + _contour_list[i].width/2.0);
        res.v.push_back(_contour_list[i].tl().y + _contour_list[i].height/2.0);
        res.width.push_back(_contour_list[i].width);
        res.height.push_back(_contour_list[i].height);
    }
    res.numberOfSegments = _contour_list.size();
    res.success = true;

    return true;
}

//----------------------------------------------------------------------------
/**
 * Set image for performing segmentation
 */
bool Segmentation::_setImage(const cv::Mat & image)
{
    //check if image is valid
    if(!image.data)
    {
        return false;
    }

    //update image
    _input_image = image.clone();
    _segmented_image = image.clone();

    return true;
}

//----------------------------------------------------------------------------
/**
 * This function performs the segmentation by assigning the class labels to
 * the image. Note that the blue channel is replaced with the class labels,
 * so to view the segmented image, the blue channel must be displayed
 * separately.
 */
bool Segmentation::_assignPixelValues()
{
    CV_Assert(_segmented_image.depth() == CV_8U);

    int channels = _segmented_image.channels();

    int n_rows = _segmented_image.rows;
    int n_cols = _segmented_image.cols * channels;

    if (_segmented_image.isContinuous())
    {
        n_cols *= n_rows;
        n_rows = 1;
    }

    int i,j;
    int r, g, b;
    uchar* p;

    for( i = 0; i < n_rows; ++i)
    {
        p = _segmented_image.ptr<uchar>(i);

        for ( j = 0; j < n_cols-3; j=j+3)
        {
            p[j] = _lut_c[p[j+2]][p[j+1]][p[j]];
        }
    }

    return true;
}

//----------------------------------------------------------------------------
/**
 * This function extracts the binary image from the input image after the
 * color values have been assigned to the blue channel
 */
bool Segmentation::_extractBinary()
{
    //extract blue channel (contains segmented image)
    std::vector<cv::Mat> channels(3);
    split(_segmented_image, channels);

    //convert to binary image
    cv::multiply(channels[0],cv::Scalar(255),_binary_image);

    return true;
}

//----------------------------------------------------------------------------
/**
 * This function opens the binary image to remove small particles in the
 * image see details on opening image [*]
 *
 * [*]
 */
bool Segmentation::_openBinary(int erode_size, int dilate_size)
{
    //set structuring elements
    cv::Mat erode_element =
    getStructuringElement(cv::MORPH_CROSS, cv::Size(erode_size, erode_size));
    cv::Mat dilate_element =
    getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size, dilate_size));

    //erode binary
    erode(_binary_image, _binary_image, erode_element);
    erode(_binary_image, _binary_image, erode_element);

    //dilate binary
    dilate(_binary_image, _binary_image, dilate_element);
    dilate(_binary_image, _binary_image, dilate_element);

    return true;
}

//----------------------------------------------------------------------------
/**
 * This function find the bounding box for each contour in the image using
 * the method proposed in [*]
 *
 * [*] Suzuki, S. and Abe, K., Topological Structural Analysis of Digitized
 * Binary Images by Border Following. CVGIP 30 1, pp 32-46 (1985)
 */
bool Segmentation::_extractContours()
{
    //convert image to binary
    _binary_image.convertTo(_binary_image, CV_8U);

    //get contours and draw contours
    std::vector<std::vector<cv::Point> > contour_points;
    findContours(_binary_image, contour_points, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//CV_RETR_EXTERNAL    CV_CHAIN_APPROX_SIMPLE
    //extract bounding box for each contour
    _contour_list.clear();
    for(int i = 0; i < contour_points.size(); i++)
    {
        _contour_list.push_back(boundingRect(cv::Mat(contour_points[i])));
    }

    return true;
}

//----------------------------------------------------------------------------
/**
 * This function extracts the set of points contained in each contour in the
 * image using the method proposed in [*] to extract the contour then each
 * point contained in the bounding box aronud the contour is tested using
 * the pointPolygonTest. If inside, the point is extracted.s
 *
 * [*] Suzuki, S. and Abe, K., Topological Structural Analysis of Digitized
 * Binary Images by Border Following. CVGIP 30 1, pp 32-46 (1985)
 */
bool Segmentation::_extractContoursFF()
{
    //convert image to binary
    _binary_image.convertTo(_binary_image, CV_8U);

    //get contours and draw contours
    std::vector<std::vector<cv::Point> > contour_points;
    findContours(_binary_image, contour_points, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //extract bounding box for each contour
    _contour_list.clear();
    // _segment_list.clear();
    for(int i = 0; i < contour_points.size(); i++)
    {
        //bounding box
        cv::Rect box = boundingRect(cv::Mat(contour_points[i]));
        _contour_list.push_back(boundingRect(cv::Mat(contour_points[i])));

        //points inside contour (test each point in bounding box)
        manipulation_vision::Segment segment;
        segment.numberOfPoints = 0;

        int w = box.br().x - box.tl().x;
        int h = box.br().y - box.tl().y;

        int u = box.tl().x;
        while(u < box.br().x)
        {
          int v = box.tl().y;
          while(v < box.br().y)
          {
            cv::Point2f point;
            point.x = u;
            point.y = v;
            if(cv::pointPolygonTest(contour_points[i], point, false) > 0)
            {
              segment.b.push_back((unsigned short)_input_image.at<cv::Vec3b>(v,u)[0]);
              segment.g.push_back((unsigned short)_input_image.at<cv::Vec3b>(v,u)[1]);
              segment.r.push_back((unsigned short)_input_image.at<cv::Vec3b>(v,u)[2]);
              segment.u.push_back(u);
              segment.v.push_back(v);
              segment.width = box.width;
              segment.height = box.height;
              segment.numberOfPoints++;
            }

            v++;
          }

          u++;
        }

        _segment_list.push_back(segment);
    }

    //display number of points for each contour
    // for(int i = 0; i < _segment_list.size(); i++)
    // {
    //   std::cout << "i, n = " << i << ", "
    //                          << _segment_list[i].numberOfPoints << std::endl;
    // }

    //display (u,v) coordinate of each point on each contour
    // for(int i = 0; i < contour_points.size(); i++)
    // {
    //     if(i==1)
    //     {
    //       break;
    //     }
    //     for(int j = 0; j < contour_points[i].size(); j++)
    //     {
    //       std::cout << "i,j,u,v = " << i << ", "
    //                                 << j << ", "
    //                                 << contour_points[i][j].x << ", "
    //                                 << contour_points[i][j].y << std::endl;
    //     }
    // }

    return true;
}

bool Segmentation::_writeOutput()
{
    //get filepath to write contours
    std::string filepath = ros::package::getPath("manipulation_vision") + "/output/";

    //write input, binary, and segmented images
    cv::imwrite(filepath + "_segmentation_input.png", _input_image );
    cv::imwrite(filepath + "_segmentation_output.png", _segmented_image );
    cv::imwrite(filepath + "_segmentation_binary.png", _binary_image );

    //write contours
    for(int i = 0; i<_contour_list.size(); i++)
    {
        //enlarge bounding box for each blob
        int tl_x, tl_y, br_x, br_y;
        tl_x = _contour_list[i].tl().x - (double)_contour_list[i].width*(50.0/100.0);
        tl_y = _contour_list[i].tl().y - (double)_contour_list[i].height*(50.0/100.0);
        br_x = _contour_list[i].br().x + (double)_contour_list[i].width*(50.0/100.0);
        br_y = _contour_list[i].br().y + (double)_contour_list[i].height*(50.0/100.0);

        //trim parts of bounding box outside of the image
        if(tl_x < 0) tl_x = 0;
        if(tl_y < 0) tl_y = 0;
        if(br_x > _input_image.cols) br_x = _input_image.cols;
        if(br_y > _input_image.rows) br_y = _input_image.rows;

        cv::Rect extended_rect = cv::Rect( cv::Point(tl_x,tl_y),
                                           cv::Point(br_x,br_y) );

        //write image of blob to file
        cv::imwrite(filepath + "contour" + std::to_string(i) + ".png", _input_image(extended_rect) );
    }

    return true;
}

}
