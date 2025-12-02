///////////////////////////////////////////////////////////////////////////////
//      Title     : cca_ros_util.hpp
//      Project   : cca_ros_util
//      Created   : Spring 2025
//      Author    : Janak Panthi (Crasun Jans)
///////////////////////////////////////////////////////////////////////////////

#ifndef CCA_ROS_UTIL_H
#define CCA_ROS_UTIL_H

#include "affordance_util/affordance_util.hpp"
#include "cc_affordance_planner/cc_affordance_planner.hpp"
#include "cc_affordance_planner/cc_affordance_planner_interface.hpp"
#include "cc_affordance_planner/cc_affordance_planner_util.hpp"
#include "cca_ros/cca_ros.hpp"
#include "cca_ros_msgs/msg/gripper_goal_type.hpp"
#include "cca_ros_msgs/msg/motion_type.hpp"
#include "cca_ros_msgs/msg/planning_request.hpp"
#include "cca_ros_msgs/msg/update_method.hpp"
#include "cca_ros_msgs/msg/virtual_screw_order.hpp"
#include "cca_ros_msgs/msg/ee_orientation_constraint.hpp"
#include <Eigen/Dense>
#include <stdexcept>
#include <unordered_map>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <filesystem>

namespace cca_ros_util
{

/**
 * @brief Converts a ROS PlanningRequest message to a CCA PlanningRequest object.
 *
 * This function takes a cca_ros_msgs::msg::PlanningRequest message and converts it
 * into a cca_ros::PlanningRequest struct for use with CCA libraries.
 *
 * @param msg The input PlanningRequest message containing planning data.
 * @return A CCA PlanningRequest object for further processing.
 */
cca_ros::PlanningRequest convert_cca_ros_action_to_req(const cca_ros_msgs::msg::PlanningRequest &msg);

/**
 * @brief Converts a CCA PlanningRequest object to a ROS PlanningRequest message.
 *
 * This function transforms a cca_ros::PlanningRequest object into a
 * cca_ros_msgs::msg::PlanningRequest message for communication within ROS.
 *
 * @param req The CCA PlanningRequest object to be converted.
 * @return A ROS PlanningRequest message.
 */
cca_ros_msgs::msg::PlanningRequest convert_req_to_cca_ros_action(const cca_ros::PlanningRequest &req);

/**
* @brief Logs the details of a CCA PlannerConfig object as a formatted string.
*
* Generates a detailed, human-readable representation of the provided
* cc_affordance_planner::PlannerConfig object for debugging and logging purposes.
*
* @param planner_config The CCA PlannerConfig object to log.
*
* @return A std::stringstream containing the formatted log output.
*/
std::stringstream log_cca_planner_config(const cc_affordance_planner::PlannerConfig &planner_config);

/**
* @brief Logs the details of a CCA TaskDescription object as a formatted string.
*
* Generates a detailed, human-readable representation of the provided
* cc_affordance_planner::TaskDescription object for debugging and logging purposes.
*
* @param task_description The CCA TaskDescription object to log.
*
* @return A std::stringstream containing the formatted log output.
*/
std::stringstream log_cca_task_description(const cc_affordance_planner::TaskDescription &task_description);

/**
 * @brief Logs the details of a CCA PlanningRequest object as a formatted string.
 *
 * Generates a detailed, human-readable representation of the provided
 * cca_ros::PlanningRequest object for debugging and logging purposes.
 *
 * @param req The CCA PlanningRequest object to log.
 * @return A std::stringstream containing the formatted log output.
 */
std::stringstream log_cca_planning_request(const cca_ros::PlanningRequest &req);

/**
 * @brief Logs the details of a CCA PlannerResult object as a formatted string.
 *
 * Generates a detailed, human-readable representation of the provided
 * cc_affordance_planner::PlannerResult object for debugging and logging purposes.
 *
 * @param res The CCA PlannerResult object to log.
 * @return A std::stringstream containing the formatted log output.
 */
std::stringstream log_cca_planning_result(const cc_affordance_planner::PlannerResult& res);

/**
 * @brief Writes a single PlanningRequest to a specified file.
 *
 * Converts the provided PlanningRequest into a textual log representation using
 * `log_cca_planning_request()` and writes the resulting content into the file
 * specified by @p filepath. Existing file contents are overwritten.
 *
 * @param req       The PlanningRequest to serialize.
 * @param filepath  Destination file path where the log will be written.
 *
 * @throws std::runtime_error If the file cannot be opened for writing.
 */
void log_cca_planning_request_to_file(const cca_ros::PlanningRequest& req,
                                  const std::filesystem::path& filepath);

/**
 * @brief Writes multiple PlanningRequests to a single file.
 *
 * Iterates through the collection of PlanningRequests, logging each request
 * using `log_cca_planning_request()`. Each request is delineated by an index
 * marker in the output file, following this format:
 *
 * @code
 * --- Request 0 ---
 * <log text>
 *
 * --- Request 1 ---
 * <log text>
 * @endcode
 *
 * Existing file contents are overwritten.
 *
 * @param reqs      The list of PlanningRequests to serialize.
 * @param filepath  Destination file path where the combined log will be written.
 *
 * @throws std::runtime_error If the file cannot be opened for writing.
 */
void log_cca_planning_requests_to_file(const std::vector<cca_ros::PlanningRequest>& reqs,
                                   const std::filesystem::path& filepath);

/**
* @brief Writes a CCA PlannerResult to a specified file.
*
* Converts the provided PlannerResult into a textual log representation using
* `log_cca_planning_result()` and writes the resulting content into the file
* specified by @p filepath. Existing file contents are overwritten.
*
* @param res      The PlannerResult to serialize.
* @param filepath Destination file path where the log will be written.
*/
void log_cca_planning_result_to_file(const cc_affordance_planner::PlannerResult& res,
                                  const std::filesystem::path& filepath);
} // namespace cca_ros_util

#endif // CCA_ROS_UTIL_H
