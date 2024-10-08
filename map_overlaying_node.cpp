#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

class MapOverlayNode : public rclcpp::Node {
public:
    MapOverlayNode() : Node("map_overlay_node") {
        RCLCPP_INFO(this->get_logger(), "Map Overlay Node started.");

        // Load the two maps
        std::string map1_path = "/home/student/map.png";
        std::string map2_path = "/home/student/my_map.png";

        cv::Mat img1 = cv::imread(map1_path, cv::IMREAD_UNCHANGED);
        cv::Mat img2 = cv::imread(map2_path, cv::IMREAD_UNCHANGED);

        // Check if images are loaded successfully
        if (img1.empty() || img2.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not load one or both images.");
            return;
        }

        // Resize img2 to be slightly smaller than img1 (e.g., -5 pixels for each dimension)
        int new_width = img1.cols - 15;  // Slightly reduce width
        int new_height = img1.rows - 15; // Slightly reduce height
        cv::resize(img2, img2, cv::Size(new_width, new_height));

        // Create a canvas that is large enough to hold the larger of the two images
        int canvas_width = std::max(img1.cols, img2.cols);
        int canvas_height = std::max(img1.rows, img2.rows);
        cv::Mat canvas = cv::Mat::zeros(canvas_height, canvas_width, img1.type()); // Use img1's type for canvas

        // Calculate the offset to center img1 on the canvas
        int x_offset_img1 = (canvas.cols - img1.cols) / 2;
        int y_offset_img1 = (canvas.rows - img1.rows) / 2;

        // Place img1 in the center of the canvas
        img1.copyTo(canvas(cv::Rect(x_offset_img1, y_offset_img1, img1.cols, img1.rows)));

        // Calculate the offset to center img2 on the canvas
        int x_offset_img2 = (canvas.cols - img2.cols) / 2;
        int y_offset_img2 = (canvas.rows - img2.rows) / 2;

        // Overlay img2 on top of img1 (centered on the canvas) using transparency
        double alpha = 0.5;  // Transparency factor
        cv::Mat img2_region = canvas(cv::Rect(x_offset_img2, y_offset_img2, img2.cols, img2.rows));
        cv::addWeighted(img2, alpha, img2_region, 1.0 - alpha, 0.0, img2_region);

        // Display the result
        cv::imshow("Overlayed Map", canvas);
        cv::waitKey(0);

        // Save the result to a file
        cv::imwrite("/home/student/overlayed_map.png", canvas);
        RCLCPP_INFO(this->get_logger(), "Overlay image saved as overlayed_map.png.");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOverlayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
