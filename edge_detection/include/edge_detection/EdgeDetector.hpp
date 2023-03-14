#pragma once
#include <opencv2/opencv.hpp>

namespace edge_detection
{


/** Base class for preprocessing filters. Abstract base class which enforces inheriting classes to implemented the apply-method.
*/
class Filter{
	public:
	/** Applies the filter on the specified image and returns the resulting image.
    @param image An OpenCV matrix containing the image of interest.
    */
	virtual cv::Mat apply(const cv::Mat& image) = 0;
};

/** Bilateral preprocessing filter as described in https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/MANDUCHI1/Bilateral_Filtering.html.
 * 
 * Uses the OpenCV implementation: https://docs.opencv.org/4.x/d4/d86/group__imgproc__filter.html#ga9d7064d478c95d60003cf839430737ed
 * 
*/
class BilateralFilter : public Filter{
	private:
		/**
	 	* Diameter of each pixel neighborhood that is used during filtering. If it is non-positive, it is computed from sigmaSpace. 
		*/
		int diameter_;
		/**
		 * Filter sigma in the color space. A larger value of the parameter means that farther colors within the pixel neighborhood (see sigmaSpace) will be mixed together, resulting in larger areas of semi-equal color. 
		*/
		double sigmaColor_;
		/**
		 * Filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will influence each other as long as their colors are close enough (see sigmaColor ). When d>0, it specifies the neighborhood size regardless of sigmaSpace. Otherwise, d is proportional to sigmaSpace.
		*/
		double sigmaSpace_;
	public:
		BilateralFilter(int diameter, double sigmaColor, double sigmaSpace);
		virtual cv::Mat apply(const cv::Mat& image);
};

/**
 * Abstract base class for edge detectors. The core functionalities of these classes is implemented in detectEdges.
*/
class BaseEdgeDetector{
	public:
		/**
		 * Detects the edges of the specified image. Therefore the image is preprocessed with the filtes associated with the edge detector. Afterwards the edges are detected.
		 * @param image The image of interest as an OpenCV matrix.
		 * @returns An OpenCV grayscale matrix containing the detectes edges as white pixels. All other pixels are black.
		*/
		cv::Mat detectEdges(const cv::Mat& image);
		/**
		 * Superimoses the specified image with the specified edges by recoloring all edge pixels in the original image with the specified color.
		 * @param image The image which was used to detect edges.
		 * @param edges The edges which were detected beforehand. This shall be a result of detectEdges(image).
		 * @param edgeColor The color of the superimposed edges as an openCV Scalar with filles with RGB values.
		*/
		cv::Mat superimposeImageAndEdges(const cv::Mat& image, const cv::Mat& edges, const cv::Scalar& edgeColor);
		/**
		 * Contains pointers to the filters associated with the edge detector. These filters arer always applied before detecting edges with detectEdges().
		*/
		std::vector<Filter*> filters;
		/**
		 * Applies the filters associated with the edge detector onto the specified image.
		 * @param image An OpenCV Mat representing the image of interest.
		 * @returns The processed version of the input.
		*/
		cv::Mat applyFilters(const cv::Mat& image);
	private:
	/**
	 * Implements the core functionality of the edge detector. This method must be implemented by inheriting classes and is called in detectEdges after preprocessing the image.
	 * @param image The image of interest as a OpenCV Mat.
	 * @returns An OpenCV grayscale matrix containing the detectes edges as white pixels. All other pixels are black.
	*/
		virtual cv::Mat detectEdges_(const cv::Mat& image) = 0;
};

/**
 * Implementation of the Canny edge detector. 
*/
class CannyEdgeDetector : public BaseEdgeDetector
{
	

	public:
		CannyEdgeDetector(double threshold1, double threshold2, int apertureSize, bool L2Gradient);
	protected:
		/**
	 	* The lower threshold for hysteresis thresholding.
		*/
		double threshold1_;
		/**
	 	* The upper threshold for hysteresis thresholding.
		*/
		double threshold2_;
		/**
	 	* The aperture size of the Sobel operator..
		*/
		int apertureSize_;
		/**
	 	* If True, L2-norm is used for gradient computation. Otherwise, L1-norm is used.
		*/
		bool L2Gradient_;
		virtual cv::Mat detectEdges_(const cv::Mat& image);
		
};

/**
 * Adaptive Canny edge detector based on https://pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/. It automatically adapts the hysteresis thresholds based on the mean intensity of an image and the hyperparameter sigma.
*/
class AutoCannyEdgeDetector : public CannyEdgeDetector
{
	public:
		AutoCannyEdgeDetector(double sigma, int apertureSize, bool L2Gradient);
	
	protected:
		/**
	 	* Hyperparameter in the range [0,1] determining the size of the gap between the hysteresis thresholds. 
		*/
		double sigma_;
		/**
		 * Determines the hysteresis thresholds based on the mean intensits of the image.
		 * @param image The image of interest.
		*/
		void determineThresholds(const cv::Mat& image);
		virtual cv::Mat detectEdges_(const cv::Mat& image);
};

}
