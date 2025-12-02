///////////////////////////////////////////////////////////////////////////////
//      Title     : log_res_to_file.hpp
//      Project   : cca_ros_behavior_util
//      Created   : Fall 2025
//      Author    : Janak Panthi (Crasun Jans)
///////////////////////////////////////////////////////////////////////////////

#ifndef LOG_RES_TO_FILE_HPP
#define LOG_RES_TO_FILE_HPP

#include <behaviortree_cpp/action_node.h>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <filesystem>
#include <memory>
#include <vector>

#include <cca_ros/cca_ros.hpp>
#include <cca_ros_util/cca_ros_util.hpp>

namespace BT
{
/**
 * @brief Convert a BT string value into a std::filesystem::path.
 *
 * Ensures the string is non-empty and, if a parent directory is present,
 * verifies that it exists. Throws BT::RuntimeError on invalid input.
 *
 * @param key String from the BehaviorTree port.
 * @return Parsed filesystem path.
 */
template<>
inline std::filesystem::path convertFromString(BT::StringView key)
{
    std::string s = std::string(key);

    // Reject empty input
    if (s.empty())
    {
        throw RuntimeError("convertFromString<std::filesystem::path>: empty string");
    }

    std::filesystem::path p{s};

    // Validate parent directory if present
    if (p.has_parent_path())
    {
        const auto parent = p.parent_path();
        if (!parent.empty() && !std::filesystem::exists(parent))
        {
            throw RuntimeError(
                "convertFromString<std::filesystem::path>: parent directory does not exist: " +
                parent.string());
        }
    }

    return p;
}
} // namespace BT

namespace cca_ros_behavior_util
{

/**
 * @class LogResToFile
 * @brief BehaviorTree.CPP synchronous action that logs a single
 *        PlanningResponse to a specified file.
 *
 * This node retrieves a `cca_planning_response` from its input port,
 * converts it into a log-formatted string representation using
 * `cca_ros_util::log_cca_planning_result()`, and logs the resulting
 * text into a file specified by the `output_path` input port.
 * NOTE: Currently, this behavior only logs the cca_result portion of 
 * the planning response.
 *
 * This is typically used for debugging or for storing input responses
 * for later offline analysis, verification, or regression testing.
 */
class LogResToFile : public BT::SyncActionNode
{
public:
    /**
     * @brief Constructor.
     *
     * @param name   Node instance name within the behavior tree.
     * @param config Node configuration containing input/output ports.
     */
    inline LogResToFile(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}

    /**
     * @brief Defines the input ports required by this node.
     *
     * Required input ports:
     * - **"cca_planning_response"**: A shared pointer to a
     *   `cca_ros::PlanningResponse`.
     * - **"output_path"**: A filesystem path where the serialized response
     *   will be written.
     *
     * @return A list of ports associated with this node.
     */
    static inline BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<cca_ros::PlanningResponse>>("cca_planning_response"),
            BT::InputPort<std::filesystem::path>("output_path"),
        };
    }

    /**
     * @brief Executes the node.
     *
     * The node performs the following steps:
     *  1. Retrieves the output file path.
     *  2. Retrieves the individual PlanningResponse.
     *  3. Converts the response into a text log using
     *     `cca_ros_util::log_cca_planning_result()`.
     *  4. Delegates file-writing to
     *     `cca_ros_util::log_cca_planning_result_to_file()`.
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

        // Retrieve PlanningResponse
        BT::Expected<std::shared_ptr<cca_ros::PlanningResponse>> res_expected =
            getInput<std::shared_ptr<cca_ros::PlanningResponse>>("cca_planning_response");

        if (!res_expected)
        {
            throw BT::RuntimeError(
                "Missing required input [cca_planning_response]: ", res_expected.error());
        }

        std::shared_ptr<cca_ros::PlanningResponse> res = res_expected.value();

        const cc_affordance_planner::PlannerResult cca_result = res->result.cca_result;
        // Log using utility function
        cca_ros_util::log_cca_planning_result_to_file(cca_result, filepath);

        return BT::NodeStatus::SUCCESS;
    }
};

} // namespace cca_ros_behavior_util

#endif // LOG_RES_TO_FILE_HPP
