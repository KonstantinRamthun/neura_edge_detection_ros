#include <edge_detection/EdgeDetector.hpp>
#include <stdio.h>
#include <memory> // For std::unique_ptr
#include <opencv2/opencv.hpp>
using namespace edge_detection;

// Your class methods definition goes here
BilateralFilter::BilateralFilter(int diameter, double sigmaColor, double sigmaSpace)
:diameter_(diameter),sigmaColor_(sigmaColor), sigmaSpace_(sigmaSpace)  
{}

cv::Mat BilateralFilter::apply(const cv::Mat& image){
	cv::Mat processed_image;
	cv::bilateralFilter(image, processed_image, diameter_,sigmaColor_,sigmaSpace_);
	return processed_image;
}

cv::Mat BaseEdgeDetector::applyFilters(const cv::Mat& image){
	cv::Mat processed;
	image.copyTo(processed);
	for(int i = 0; i < filters.size(); i++){
		processed = filters[i]->apply(processed);
	}
	return processed;
}

cv::Mat BaseEdgeDetector::detectEdges(const cv::Mat& image){
	cv::Mat preprocessed_image = applyFilters(image);
	return detectEdges_(preprocessed_image);
}

CannyEdgeDetector::CannyEdgeDetector(double threshold1, double threshold2, int apertureSize, bool L2Gradient)
: BaseEdgeDetector(), threshold1_(threshold1), threshold2_(threshold2), apertureSize_(apertureSize), L2Gradient_(L2Gradient)
{}

cv::Mat CannyEdgeDetector::detectEdges_(const cv::Mat& image){
	cv::Mat edges;
	cv::Canny(image, edges, threshold1_, threshold2_, apertureSize_, L2Gradient_);
	return edges;
}

AutoCannyEdgeDetector::AutoCannyEdgeDetector(double sigma, int apertureSize, bool L2Gradient)
: CannyEdgeDetector(0.0, 0.0, apertureSize, L2Gradient), sigma_(sigma){}

void AutoCannyEdgeDetector::determineThresholds(const cv::Mat& image){
	cv::Mat image_grayscale;
	cv::cvtColor(image, image_grayscale, cv::COLOR_RGB2GRAY);
	double mean = cv::mean(image_grayscale)[0];
	threshold1_ = int(std::max(0.0, (1.0 + sigma_) * mean));
	threshold2_ = int(std::max(255.0, (1.0 +sigma_) * mean));
}

cv::Mat AutoCannyEdgeDetector::detectEdges_(const cv::Mat& image){
	cv::Mat edges;
	determineThresholds(image);
	cv::Canny(image, edges, threshold1_, threshold2_, apertureSize_, L2Gradient_);
	return edges;
}

cv::Mat BaseEdgeDetector::superimposeImageAndEdges(const cv::Mat& image, const cv::Mat& edges, const cv::Scalar& edgeColor){
	cv::Mat superimposed, mask;
	cv::inRange(edges, cv::Scalar(255,255,255), cv::Scalar(255,255,255), mask);
	image.copyTo(superimposed);
	superimposed.setTo(edgeColor, mask);
	return superimposed;
}

/**
 * Parses the CLI-arguments to an OpenCV Mat containing the image and to an edge detector.
*/
bool parse_arguments(int argc, char **argv, cv::Mat& input_image, std::unique_ptr<BaseEdgeDetector>& edge_detector){
	// How to format the keys: https://docs.opencv.org/3.4/d0/d2e/classcv_1_1CommandLineParser.html
	// Define arguments
	const std::string keys =
	"{help h usage ?            |          | Shows the help message.}"
	"{@path                     |          | The path to the image of interest.  }"
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
	std::string path = parser.get<std::string>(0);
	std::string edge_detector_arg = parser.get<std::string>(1);

	// Load image
	input_image = cv::imread( path, cv::IMREAD_COLOR );
	if ( !input_image.data )
    {
        throw std::invalid_argument(std::string("No image data found at path ") + path);
    }
	// Initialize edge detector
	if (edge_detector_arg == "canny_auto"){
		double sigma = parser.get<double>("sigma");
		int apertureSize = parser.get<int>("aperture_size");
		bool l2Gradient = parser.has("l2");
		edge_detector = std::make_unique<AutoCannyEdgeDetector>(sigma, apertureSize, l2Gradient);
	}
	else if (edge_detector_arg == "canny")
	{
		double threshold1 = parser.get<double>("threshold1");
		double threshold2 = parser.get<double>("threshold2");
		int apertureSize = parser.get<int>("aperture_size");
		bool l2Gradient = parser.has("l2");
		edge_detector = std::make_unique<CannyEdgeDetector>(threshold1, threshold2, apertureSize, l2Gradient);
	}else{
		throw std::invalid_argument(std::string("Unknown edge detector ") + edge_detector_arg + std::string(". Available edge detectors are: auto_canny, canny."));
	}
	// Add filters
	if (!parser.has("no_bf")){
		int diameter = parser.get<int>("bf_diameter");
		double sigmaColor = parser.get<double>("bf_sigma_color");
		double sigmaSpace = parser.get<double>("bf_sigma_space");
		edge_detector->filters.push_back(new BilateralFilter(diameter, sigmaColor, sigmaSpace));
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
	// Parse CLI arguments
    cv::Mat image;
	std::unique_ptr<BaseEdgeDetector> detector;
	bool parsing_successfull;
	try{
		parsing_successfull = parse_arguments(argc, argv, image, detector);
	}catch(std::invalid_argument exception){
		printf("Invalid argument: %s\n", exception.what());
		return -1;
	}
	if(!parsing_successfull){
		return -1;
	}


	// Detect Edges
    cv::Mat edges = detector->detectEdges(image);
	// Superimpose edges and image
	cv::Scalar color(0, 255, 0);
	cv::Mat superimposed = detector->superimposeImageAndEdges(image, edges, color);

	//Show results
	cv::cvtColor(edges, edges, cv::COLOR_GRAY2RGB);
	// Concatenate original image, edges and superimposed
	cv::Mat all[] = {image, edges, superimposed};
	cv::Mat concatenated;
	cv::hconcat(all, 3, concatenated);
	// Show edges
    cv::imshow("Original Image | Edges | Superimposed Image and Edges", concatenated);
    cv::waitKey(0);
	return 0;
}
