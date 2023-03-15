#include "ros/ros.h"
#include <edge_detection/EdgeDetector.hpp>
#include <cv_bridge/cv_bridge.h>
#include <edge_detection/EdgeDetection.h>
#include <opencv2/opencv.hpp>
#include <memory> // For std::unique_ptr
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * Parses the CLI-arguments to an optional OpenCV Mat containing the image.
 * @returns A bool, indicating if a path was specifeid or not.
*/
bool parse_arguments(int argc, char **argv, cv::Mat& input_image){
	// How to format the keys: https://docs.opencv.org/3.4/d0/d2e/classcv_1_1CommandLineParser.html
	// Define arguments
	const std::string keys =
	"{help h usage ?            |          | Shows the help message.}"
	"{path                      |          | The path to the image of interest.  }"
	;
	cv::CommandLineParser parser(argc, argv, keys);
	parser.about("ROS client for detecting images from the specified path.");
	//Handle help parameter
	if (parser.has("help"))
	{
    	parser.printMessage();
    	return false;
	}

    // Check for erros
	if (!parser.check())
	{	
    	parser.printErrors();
    	throw std::invalid_argument("Could not parse the arguments.");
	}

	// Process image path, if specified.
    if (parser.has("path")){
	    std::string path = parser.get<std::string>(0);
        // Load image
        input_image = cv::imread( path, cv::IMREAD_COLOR );
        if ( !input_image.data )
        {
            throw std::invalid_argument(std::string("No image data found at path ") + path);
        }
        return true;
    }else{
        return false;
    }
}

/// @brief Detects edges in the specfiied image by using the specified service.
/// @param srv The service used for edge detection.
/// @param client The service client instance used to call the service.
/// @param image The image of interest.
/// @return True, if the detection is successfull. Otherwise false.
bool detectFromPath(edge_detection::EdgeDetection& srv, ros::ServiceClient& client, cv::Mat& image){
    
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
        return true;
    }
    else
    {
        // Detection failed
        ROS_ERROR("Failed to call service detect_edges");
        return false;
    }
}

/// @brief This class implements the egde to point clound trasnformation.
class EdgeTransformer{

    private:
        ros::Publisher adaptee;
        edge_detection::EdgeDetection srv;
        ros::ServiceClient& client;

    public:
        EdgeTransformer(ros::NodeHandle& nodeHandle, edge_detection::EdgeDetection& srv, ros::ServiceClient& client)
        : adaptee(nodeHandle.advertise<sensor_msgs::Image>("edge_points", 100)), srv(srv), client(client){}


        void callback(const sensor_msgs::Image::ConstPtr& colorImage, const sensor_msgs::Image::ConstPtr& depthImage){
            // Detect edges on image
            srv.request.image = *colorImage;
            if (client.call(srv))
            { 
                cv::Mat edges = cv_bridge::toCvCopy(srv.response.edges)->image;
                cv::Mat depths = cv_bridge::toCvCopy(depthImage)->image;
                cv::cvtColor(edges, edges, cv::COLOR_GRAY2RGB);
                cv::imshow("Edges", edges);
                cv::imshow("Depths", depths);
                cv::waitKey(1);
                // Steps which are missing to complete the task:
                // Receive depth image from topic /camera/depth/image_rect_raw with additional subscriber
                ROS_INFO("%d, %d, %s\n", srv.response.edges.width, srv.response.edges.width, srv.response.edges.encoding.c_str());
                ROS_INFO("%d, %d, %s\n", depthImage->width, depthImage->width, depthImage->encoding.c_str());
                // Transform edges to point cloud with info from depth image and camera info (/camera/color/camera_info)
                // Publish point cloud with adaptee
            }
            else
            {
                ROS_ERROR("Failed to call service detect_edges");
            }
            
        }

        sensor_msgs::PointCloud2 edgesToPointCloud(const cv::Mat& edges, const cv::Mat& depths){
            sensor_msgs::PointCloud2 result;
            return result;
        }
};

/// @brief Detects edges from images provided in the topic '/camera/color/image_raw' and plots them. For full completion of the task, this should transform the edges to a point cloud.
/// @param nodeHandle The node handle used fpr publishing and subscribing.
/// @param srv The service used for edge detection.
/// @param client The service client used to call the service.
/// @return True
bool detectAndTransform(ros::NodeHandle& nodeHandle, edge_detection::EdgeDetection& srv, ros::ServiceClient& client){
    EdgeTransformer edgeTransformer(nodeHandle, srv, client);
    // http://wiki.ros.org/message_filters#Policy-Based_Synchronizer_.5BROS_1.1.2B-.5D
    message_filters::Subscriber<sensor_msgs::Image> color_sub(nodeHandle, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nodeHandle, "/camera/depth/image_rect_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), color_sub, depth_sub);
    sync.registerCallback(boost::bind(&EdgeTransformer::callback, &edgeTransformer, _1, _2));
    ros::spin();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "edge_detection_client");

    //Parse CLI arguments
    cv::Mat image;
    bool path_specified = parse_arguments(argc, argv, image);

    // Shared node handle and service
    ros::NodeHandle nodeHandle;
    edge_detection::EdgeDetection srv;
    ros::ServiceClient client = nodeHandle.serviceClient<edge_detection::EdgeDetection>("detect_edges");
    // Differentiate between specified path and subscriber-publisher-interaction.
    bool success;
    if (path_specified){
        success = detectFromPath(srv, client, image);
    }else{
        success = detectAndTransform(nodeHandle, srv, client);
    }
    
}