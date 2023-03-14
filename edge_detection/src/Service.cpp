#include "ros/ros.h"
#include <edge_detection/EdgeDetector.hpp>
#include <cv_bridge/cv_bridge.h>
#include <edge_detection/EdgeDetection.h>
#include <opencv2/opencv.hpp>
#include <memory> // For std::unique_ptr

       
/**
 * Parses the CLI-arguments to an OpenCV Mat containing the image and to an edge detector.
*/
bool parse_arguments(int argc, char **argv, std::unique_ptr<edge_detection::BaseEdgeDetector>& edge_detector){
	// How to format the keys: https://docs.opencv.org/3.4/d0/d2e/classcv_1_1CommandLineParser.html
	// Define arguments
	const std::string keys =
	"{help h usage ?            |          | Shows the help message.}"
	"{@edge_detector            |canny_auto| The type of edge detector to used. Options are: canny_auto, canny. Depending on thos value, the following parameters are determined.}"
	"{threshold1                |200       | Only used if edge_detector is 'canny'. Determines the lower hysteresis theshold.}"
	"{threshold2                |225       | Only used if edge_detector is 'canny'. Determines the upper hysteresis theshold.}"
	"{aperture_size             |3         | Only used for canny edge detectors. Aperture size of the Sobel operator for canny edge detectors.}"
	"{l2 l2_gradient            |          | Only used for canny edge detectors. If true, the L2-norm is used to compute gradients of the image. Otherwise, the L1-norm is used.  }"
	"{sigma                     |0.33      | Only used if edge_detector is 'auto_canny'. The sigma value for computing the lower and upper hysteresis threshold.}"
	"{no_bf no_bilateral_filter |          | If not specified, the image is preprocessed with a bilateral filter, parametrized by the values specified for  bf_diameter, bf_sigma_color and bf_sigma_space.}"
	"{bf_diameter               |15        | Only used if 'no_bf' is not specified. Diameter of each pixel neighborhood that is used during filtering. If it is non-positive, it is computed from bf_sigma_space.     }"
	"{bf_sigma_color            |200       | Only used if 'no_bf' is not specified. Filter sigma in the color space. A larger value of the parameter means that farther colors within the pixel neighborhood (see bf_sigma_space) will be mixed together, resulting in larger areas of semi-equal color.        }"
	"{bf_sigma_space            |200       | Only used if 'no_bf' is not specified. Filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will influence each other as long as their colors are close enough (see bf_sigma_color ). When bf_diameter>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, d is proportional to sigmaSpace.       }"
	;
	cv::CommandLineParser parser(argc, argv, keys);
	parser.about("Detects edges in a specified image. Implements different methods for edge detection. Currently the canny edge detector and an addaptive version of the canny edge detector (auto canny) are implemented. For preprocessing, a bliateral filter as described in https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MANDUCHI1/Bilateral_Filtering.html can be used.");
	//Handle help parameter
	if (parser.has("help"))
	{
    	parser.printMessage();
    	return false;
	}

	// Process positional arguments
	std::string edge_detector_arg = parser.get<std::string>(0);

	// Initialize edge detector
	if (edge_detector_arg == "canny_auto"){
		double sigma = parser.get<double>("sigma");
		int apertureSize = parser.get<int>("aperture_size");
		bool l2Gradient = parser.has("l2");
		edge_detector = std::make_unique<edge_detection::AutoCannyEdgeDetector>(sigma, apertureSize, l2Gradient);
	}
	else if (edge_detector_arg == "canny")
	{
		double threshold1 = parser.get<double>("threshold1");
		double threshold2 = parser.get<double>("threshold2");
		int apertureSize = parser.get<int>("aperture_size");
		bool l2Gradient = parser.has("l2");
		edge_detector = std::make_unique<edge_detection::CannyEdgeDetector>(threshold1, threshold2, apertureSize, l2Gradient);
	}else{
		throw std::invalid_argument(std::string("Unknown edge detector ") + edge_detector_arg + std::string(". Available edge detectors are: auto_canny, canny."));
	}
	// Add filters
	if (!parser.has("no_bf")){
		int diameter = parser.get<int>("bf_diameter");
		double sigmaColor = parser.get<double>("bf_sigma_color");
		double sigmaSpace = parser.get<double>("bf_sigma_space");
		edge_detector->filters.push_back(new edge_detection::BilateralFilter(diameter, sigmaColor, sigmaSpace));
	}

	// Check for erros
	if (!parser.check())
	{	
    	parser.printErrors();
    	return false;
	}
	return true;
}

std::unique_ptr<edge_detection::BaseEdgeDetector> detector;

bool detectEdges(edge_detection::EdgeDetectionRequest  &req, edge_detection::EdgeDetectionResponse &res)
    {
        // Parse sensor_msgs::Image to cv::Mat 
        ROS_INFO("Received request.");
        cv::Mat image = (*cv_bridge::toCvCopy(req.image)).image;
        
        // Detect edges
        cv::Mat edges = detector->detectEdges(image);
        // Parse edges back to sensor_msgs::Image
        cv_bridge::CvImage(std_msgs::Header(), "mono8", edges).toImageMsg(res.edges);
        ROS_INFO("Send respond.");
        return true;
    } 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "edge_detection_service");
    ros::NodeHandle n;
    bool parsed = parse_arguments(argc, argv, detector);
    if (!parsed){
        return -1;
    }
    ros::ServiceServer service_server = n.advertiseService("detect_edges", detectEdges);
    ROS_INFO("Ready to detect edges.");
    ros::spin();
     
    return 0;
}