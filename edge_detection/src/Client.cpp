#include "ros/ros.h"
#include <edge_detection/EdgeDetector.hpp>
#include <cv_bridge/cv_bridge.h>
#include <edge_detection/EdgeDetection.h>
#include <opencv2/opencv.hpp>
#include <memory> // For std::unique_ptr

/**
 * Parses the CLI-arguments to an OpenCV Mat containing the image.
*/
bool parse_arguments(int argc, char **argv, cv::Mat& input_image){
	// How to format the keys: https://docs.opencv.org/3.4/d0/d2e/classcv_1_1CommandLineParser.html
	// Define arguments
	const std::string keys =
	"{help h usage ?            |          | Shows the help message.}"
	"{@path                     |          | The path to the image of interest.  }"
	;
	cv::CommandLineParser parser(argc, argv, keys);
	parser.about("ROS client for detecting images from the specified path.");
	//Handle help parameter
	if (parser.has("help"))
	{
    	parser.printMessage();
    	return false;
	}

	// Process positional arguments
	std::string path = parser.get<std::string>(0);

	// Load image
	input_image = cv::imread( path, cv::IMREAD_COLOR );
	if ( !input_image.data )
    {
        throw std::invalid_argument(std::string("No image data found at path ") + path);
    }

	// Check for erros
	if (!parser.check())
	{	
    	parser.printErrors();
    	return false;
	}
	return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "edge_detection_client");

    //Parse CLI arguments
    cv::Mat image;
    bool parsed = parse_arguments(argc, argv, image);
    if(!parsed){
        return -1;
    }

    // Intialize client
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<edge_detection::EdgeDetection>("detect_edges");
    edge_detection::EdgeDetection srv;

    // Prepare request message
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg(srv.request.image);
    // Call service
    if (client.call(srv))
    {   
        // Parse response message to image
        cv::Mat edges = cv_bridge::toCvCopy(srv.response.edges)->image;
        cv::cvtColor(edges, edges, cv::COLOR_GRAY2RGB);
        cv::imshow("Edges", edges);
        cv::waitKey(0);
        return 0;
    }
    else
    {
        ROS_ERROR("Failed to call service detect_edges");
        return -1;
    }
}