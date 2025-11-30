///////////////////////////////////////////////////////////////////////////////
//      Title     : log_req_to_file.hpp
//      Project   : cca_ros_behavior_util
//      Created   : Fall 2025
//      Author    : Janak Panthi (Crasun Jans)
///////////////////////////////////////////////////////////////////////////////

#ifndef LOG_REQ_TO_FILE_HPP
#define LOG_REQ_TO_FILE_HPP

#include <behaviortree_cpp/action_node.h>
#include <filesystem>
#include <memory>
#include <vector>

#include <cca_ros/cca_ros.hpp>
#include <cca_ros_util/cca_ros_util.hpp>

namespace cca_ros_behavior_util
{

/**
 * @class LogReqToFile
 * @brief BehaviorTree.CPP synchronous action that logs a single
 *        PlanningRequest to a specified file.
 *
 * This node retrieves a `cca_planning_request` from its input port,
 * converts it into a log-formatted string representation using
 * `cca_ros_util::log_cca_planning_request()`, and logs the resulting
 * text into a file specified by the `output_path` input port.
 *
 * This is typically used for debugging or for storing input requests
 * for later offline analysis, verification, or regression testing.
 */
class LogReqToFile : public BT::SyncActionNode
{
public:
    /**
     * @brief Constructor.
     *
     * @param name   Node instance name within the behavior tree.
     * @param config Node configuration containing input/output ports.
     */
    inline LogReqToFile(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}

    /**
     * @brief Defines the input ports required by this node.
     *
     * Required input ports:
     * - **"cca_planning_request"**: A shared pointer to a
     *   `cca_ros::PlanningRequest`.
     * - **"output_path"**: A filesystem path where the serialized request
     *   will be written.
     *
     * @return A list of ports associated with this node.
     */
    static inline BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<cca_ros::PlanningRequest>>("cca_planning_request"),
            BT::InputPort<std::filesystem::path>("output_path"),
        };
    }

    /**
     * @brief Executes the node.
     *
     * The node performs the following steps:
     *  1. Retrieves the output file path.
     *  2. Retrieves the individual PlanningRequest.
     *  3. Converts the request into a text log using
     *     `cca_ros_util::log_cca_planning_request()`.
     *  4. Delegates file-writing to
     *     `cca_ros_util::log_cca_planning_request_to_file()`.
     *
     * @return BT::NodeStatus::SUCCESS upon successful completion.
     *
     * @throws BT::RuntimeError if any required input port is missing.
     * @throws std::runtime_error if the output file cannot be opened.
     */
    inline BT::NodeStatus tick() override
    {
        // Retrieve output path
        BT::Expected<std::filesystem::path> filepath_expected =
            getInput<std::filesystem::path>("output_path");

        if (!filepath_expected)
        {
            throw BT::RuntimeError(
                "Missing required input [output_path]: ", filepath_expected.error());
        }
        std::filesystem::path filepath = filepath_expected.value();

        // Retrieve PlanningRequest
        BT::Expected<std::shared_ptr<cca_ros::PlanningRequest>> req_expected =
            getInput<std::shared_ptr<cca_ros::PlanningRequest>>("cca_planning_request");

        if (!req_expected)
        {
            throw BT::RuntimeError(
                "Missing required input [cca_planning_request]: ", req_expected.error());
        }

        std::shared_ptr<cca_ros::PlanningRequest> req = req_expected.value();

        // Log using utility function
        cca_ros_util::log_cca_planning_request_to_file(*req, filepath);

        return BT::NodeStatus::SUCCESS;
    }
};


/**
 * @class LogReqsToFile
 * @brief BehaviorTree.CPP synchronous action that logs a sequence of
 *        PlanningRequests (vector) to a specified file.
 *
 * This node retrieves a vector of PlanningRequests from the
 * `cca_planning_requests` input port, converts each entry into a
 * log-formatted string using `cca_ros_util::log_cca_planning_request()`,
 * and logs all entries—annotated with request indices—into a single file
 * specified by the `output_path` port.
 *
 * The resulting file is typically used to store batches of requests
 * from planning pipelines or replay logs for debug and analysis.
 */
class LogReqsToFile : public BT::SyncActionNode
{
public:
    /**
     * @brief Constructor.
     *
     * @param name   Node instance name within the behavior tree.
     * @param config Node configuration containing ports.
     */
    inline LogReqsToFile(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}

    /**
     * @brief Defines the input ports required by this node.
     *
     * Required input ports:
     * - **"cca_planning_requests"**: A shared pointer to a vector of
     *   `cca_ros::PlanningRequest` objects.
     * - **"output_path"**: The filesystem path to store the output log file.
     *
     * @return A list of ports associated with this node.
     */
    static inline BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<std::vector<cca_ros::PlanningRequest>>>(
                "cca_planning_requests"),
            BT::InputPort<std::filesystem::path>("output_path"),
        };
    }

    /**
     * @brief Executes the node.
     *
     * The node performs the following steps:
     *  1. Retrieves the output file path.
     *  2. Retrieves the vector of PlanningRequests.
     *  3. Delegates log generation + file writing to
     *     `cca_ros_util::log_cca_planning_requests_to_file()`.
     *
     * Each entry in the result file is delineated using this format:
     *
     * @code
     * --- Request 0 ---
     * <log text>
     *
     * --- Request 1 ---
     * <log text>
     * @endcode
     *
     * @return BT::NodeStatus::SUCCESS on successful log.
     *
     * @throws BT::RuntimeError if required input ports are missing.
     * @throws std::runtime_error if the output file cannot be opened.
     */
    inline BT::NodeStatus tick() override
    {
        // Retrieve output path
        BT::Expected<std::filesystem::path> filepath_expected =
            getInput<std::filesystem::path>("output_path");

        if (!filepath_expected)
        {
            throw BT::RuntimeError(
                "Missing required input [output_path]: ", filepath_expected.error());
        }
        std::filesystem::path filepath = filepath_expected.value();

        // Retrieve vector of PlanningRequests
        BT::Expected<std::shared_ptr<std::vector<cca_ros::PlanningRequest>>> reqs_expected =
            getInput<std::shared_ptr<std::vector<cca_ros::PlanningRequest>>>(
                "cca_planning_requests");

        if (!reqs_expected)
        {
            throw BT::RuntimeError(
                "Missing required input [cca_planning_requests]: ", reqs_expected.error());
        }

        std::shared_ptr<std::vector<cca_ros::PlanningRequest>> reqs =
            reqs_expected.value();

        // Log using utility function
        cca_ros_util::log_cca_planning_requests_to_file(*reqs, filepath);

        return BT::NodeStatus::SUCCESS;
    }
};

} // namespace cca_ros_behavior_util

#endif // LOG_REQ_TO_FILE_HPP
