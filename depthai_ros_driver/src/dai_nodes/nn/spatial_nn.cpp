#include "depthai_ros_driver/dai_nodes/nn/spatial_nn.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_driver/dai_nodes/nn/spatial_mobilenet.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_yolo.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
SpatialNN::SpatialNN(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s base", daiNodeName.c_str());
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    auto family = ph->getNNFamily(getROSNode());
    switch(family) {
        case param_handlers::nn::NNFamily::Yolo: {
            nnNode = std::make_unique<dai_nodes::nn::SpatialYolo>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Mobilenet: {
            nnNode = std::make_unique<dai_nodes::nn::SpatialMobilenet>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Segmentation: {
            throw(std::runtime_error("Segmentation not supported for spatial network!"));
        }
    }

    RCLCPP_INFO(node->get_logger(), "Base node %s created", daiNodeName.c_str());
};
void SpatialNN::setNames() {}

void SpatialNN::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {}

void SpatialNN::setupQueues(std::shared_ptr<dai::Device> device) {
    nnNode->setupQueues(device);
}
void SpatialNN::closeQueues() {
    nnNode->closeQueues();
}

void SpatialNN::link(const dai::Node::Input& in, int linkType) {
    nnNode->link(in, linkType);
}

dai::Node::Input SpatialNN::getInput(int linkType) {
    return nnNode->getInput(linkType);
}

void SpatialNN::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(getROSNode(), params);
    nnNode->updateParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
