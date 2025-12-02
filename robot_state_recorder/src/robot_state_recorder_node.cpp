///////////////////////////////////////////////////////////////////////////////
//      Title     : robot_state_recorder_node.cpp
//      Project   : robot_state_recorder
//      Created   : Spring 2025
//      Author    : Janak Panthi (Crasun Jans)
///////////////////////////////////////////////////////////////////////////////

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <rclcpp/node.hpp>
#include <ros_cpp_util/ros_cpp_util.hpp>
#include <condition_variable>
#include <csignal>
#include <fstream>
#include <iomanip> // for std::precision
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.h>
#include <string>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <cca_ros/cca_ros.hpp>
#include <unordered_map>
/*
Author: Crasun Jans
*/

/****************** Signal handling ********************************/
static bool g_exit_flag = false; // signal to shutdown ROS on ctrl+c

// Function to handle ctrl+c signal
static void signal_callback_handler([[maybe_unused]] int signum) { g_exit_flag = true; }

/****************** EOF Signal handling ****************************/

namespace robot_state_recorder{

/***** Joint Trajectory and EE TF Recorder class ******************/
class JointTrajAndTfRecorder
{
  public:
    JointTrajAndTfRecorder(std::shared_ptr<rclcpp::Node> node, 
                          const affordance_util::RobotConfig &robot_config, 
                          const std::string &as_server_name, 
                          const std::string& joint_states_topic,
                          const std::string& recorder_name,
			  const std::string& output_dir) 
        : node_(node), recorder_name_(recorder_name), output_dir_(output_dir)
    {
        // Subscribers
        follow_joint_traj_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            as_server_name + "/goal", 1000,
            std::bind(&JointTrajAndTfRecorder::follow_joint_traj_sub_cb_, this, std::placeholders::_1));
        joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic, 1000,
            std::bind(&JointTrajAndTfRecorder::joint_states_cb_, this, std::placeholders::_1));

        // Extract robot config info
        slist_ = robot_config.Slist;
        joint_names_ = robot_config.joint_names.robot;
        M_ = robot_config.M;
        tool_name_ = robot_config.frame_names.tool;

        // Concurrently, while writing predicted data, we'll write actual data as
        // well, because while predicted data is being written, action server is
        // probably executing joint movement already
        act_data_writer_thread_ = std::thread(&JointTrajAndTfRecorder::write_act_data_, this);
        sentinel_cleanup_thread_ = std::thread(&JointTrajAndTfRecorder::cleanup_post_interruption_,
                                               this); // post-signal cleanup thread
    }

    ~JointTrajAndTfRecorder()
    {
        // Join the threads before exiting
        if (act_data_writer_thread_.joinable())
        {
            act_data_writer_thread_.join();
        }
        if (sentinel_cleanup_thread_.joinable())
        {
            sentinel_cleanup_thread_.join();
        }
    }

  private:
    // ROS variables
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr follow_joint_traj_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    ros_cpp_util::JointTrajPoint joint_states_;
    // Robot data
    Eigen::MatrixXd slist_;
    std::vector<std::string> joint_names_;
    Eigen::MatrixXd M_;
    std::string tool_name_;
    std::string recorder_name_;
    // Multithreading and data-sync tools
    std::mutex mutex_;
    std::thread act_data_writer_thread_;
    std::thread sentinel_cleanup_thread_; // thread to ensure cleanup after ctrl+c
                                          // interruption
    std::condition_variable joint_states_cv_;
    std::condition_variable follow_joint_traj_cv_;
    std::condition_variable cleanup_cv_; // CV to wake thread up after successful
                                         // cleanup in act_data_writer_thread_
    bool cb_called_ = false;
    bool joint_states_ready_ = false;
    // Other variables
    std::string output_dir_;

    // Function to handle cleanup on signal interruption
    void cleanup_post_interruption_()
    {
        // Wait until signal interruption is received
        while (!g_exit_flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Wake up act_data_writer_thread_
        joint_states_cv_.notify_all();
        follow_joint_traj_cv_.notify_all();
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cleanup_cv_.wait(lock); // Wait until act_data_writer_ thread is
                                    // successfully cleaned up
        }
        std::cout << "Successfully exited recorder: " << recorder_name_ << std::endl;
    }

    // Callback function for the follow_joint_trajectory goal subscriber
    void follow_joint_traj_sub_cb_(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        // Lock the mutex and update cb_called_ flag
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cb_called_ = true;
        } // scope for mutex

        // Wake the other thread up
        follow_joint_traj_cv_.notify_one();

        // Extract trajectory, put it in the right order, and then, call the writing
        // function
        const auto &unordered_pred_traj_ = *msg;
        const std::vector<ros_cpp_util::JointTrajPoint> pred_traj_ =
            ros_cpp_util::get_ordered_joint_traj(unordered_pred_traj_, joint_names_);
        std::cout << "Writing predicted data for " << recorder_name_ << std::endl;
        write_pred_data(pred_traj_);
        std::cout << "Finished writing predicted data for " << recorder_name_ << std::endl;
    }

    // Callback function for the joint_states subscriber
    void joint_states_cb_(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Lock the mutex and update joint_states and data-ready flag
        {
            std::lock_guard<std::mutex> lock(mutex_);
            joint_states_ = ros_cpp_util::get_ordered_joint_states(msg, joint_names_);
            joint_states_ready_ = true;
        }

        // Wake the other thread up
        joint_states_cv_.notify_one();
    }

    // Function to write predicted data to file
    void write_pred_data(const std::vector<ros_cpp_util::JointTrajPoint> &pred_traj_)
    {
        const std::string file_timestamp = std::to_string(node_->now().nanoseconds());
        const std::string filename = "pred_tf_and_joint_states_data_" + recorder_name_ + "_" + file_timestamp + ".csv";
        const std::string filepath = output_dir_ + filename;

        // Open a CSV file for writing
        std::ofstream csvFile(filepath);

        // Set precision for writing to file
        csvFile << std::fixed << std::setprecision(5); // fixed-type notation and 5 decimal places

        // Check if the file was opened successfully
        if (!csvFile.is_open())
        {
            std::cerr << "Failed to open the predicted-data CSV file for writing. " << std::endl;
            return;
        }

        /* Headers */
        // Joint_names
        for (const std::string &joint_name : joint_names_)
        {
            csvFile << joint_name << ",";
        }
        // EE position and timestamp
        csvFile << "Pred EE x,Pred EE y,Pred EE z,";                       // CSV header
        csvFile << "Pred EE x_or,Pred EE y_or,Pred EE z_or,Pred EE_w_or,"; // CSV header
        csvFile << "Timestamp"
                << "\n";

        for (const auto &pred_traj_point : pred_traj_)
        {
            // Joint positions
	    for (auto i = Eigen::Index{0}; i < pred_traj_point.positions.size(); ++i)
            {
                csvFile << pred_traj_point.positions[i] << ",";
            }

            // EE position
            Eigen::MatrixXd ee_htm = affordance_util::FKinSpace(M_, slist_, pred_traj_point.positions);
            Eigen::Quaterniond ee_htm_or(ee_htm.block<3, 3>(0, 0));
            csvFile << ee_htm(0, 3) << "," << ee_htm(1, 3) << "," << ee_htm(2, 3) << ",";
            csvFile << ee_htm_or.x() << "," << ee_htm_or.y() << "," << ee_htm_or.z() << "," << ee_htm_or.w() << ",";

            // Timestamp(nanosecs)
            csvFile << pred_traj_point.timestamp << "\n";
        }

        // Close file before exiting
        csvFile.close();
    }

    // Function to write actual data to file
    void write_act_data_()
    {
        rclcpp::Rate loop_rate(10); // Rate for the writing loop

        const std::string file_timestamp = std::to_string(node_->now().nanoseconds());
        const std::string filename = "act_tf_and_joint_states_data_" + recorder_name_ + "_" + file_timestamp + ".csv";
        const std::string filepath = output_dir_ + filename;

        // Open a CSV file for writing
        std::ofstream csvFile(filepath);

        // Set precision for writing to file
        csvFile << std::fixed << std::setprecision(5); // fixed-type notation and 5 decimal places

        // Check if the file was opened successfully
        if (!csvFile.is_open())
        {
            std::cerr << "Failed to open the actual-data CSV file for writing" << std::endl;
            return;
        }

        // Put the thread to sleep and check for cb_called_ to be true. Once true,
        // set it to false and move on.
        {
            std::unique_lock<std::mutex> lock(mutex_);
            follow_joint_traj_cv_.wait(lock, [this] { return (cb_called_ || g_exit_flag); });
            if (g_exit_flag)
            {
                // Close the file when done
                csvFile.close();
                std::cout << "Exited without writing actual data for " << recorder_name_ << std::endl;
                cleanup_cv_.notify_all(); // Wake up cleanup thread
                return;
            }

            cb_called_ = false;
        }

        std::cout << "Writing actual data for " << recorder_name_ << std::endl;

        for (const std::string &joint_name : joint_names_)
        {
            csvFile << joint_name << ",";
        }
        csvFile << "Act EE x,Act EE y,Act EE z,";                      // CSV header
        csvFile << "Act EE x_or,Act EE y_or,Act EE z_or,Act EE_w_or,"; // CSV header
        csvFile << "Timestamp"
                << "\n";

        // Write data to file until interrupted with ctrl-c
        while (true)
        {
            // Wait for joint states data to be ready routinely checking it while
            // putting the thread to sleep at other times and releasing the mutex.
            // When it is ready, copy it, set data-ready flag to false, release the
            // mutex, and move on
            ros_cpp_util::JointTrajPoint joint_states_copy;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                joint_states_cv_.wait(lock, [this] { return (joint_states_ready_ || g_exit_flag); });
                if (g_exit_flag)
                    break;
                joint_states_copy = joint_states_;
                joint_states_ready_ = false;
            }
            // Write joint_states data to file
	    for (auto i = Eigen::Index{0}; i < joint_states_copy.positions.size(); ++i)
            {
                csvFile << joint_states_copy.positions[i] << ",";
            }

            // Write EE position to file
            Eigen::MatrixXd ee_htm = affordance_util::FKinSpace(M_, slist_, joint_states_copy.positions);
            Eigen::Quaterniond ee_htm_or(ee_htm.block<3, 3>(0, 0));

            csvFile << ee_htm(0, 3) << "," << ee_htm(1, 3) << "," << ee_htm(2, 3) << ",";
            csvFile << ee_htm_or.x() << "," << ee_htm_or.y() << "," << ee_htm_or.z() << "," << ee_htm_or.w() << ",";

            // Write timestamp(nanosecs) to file
            csvFile << joint_states_copy.timestamp << "\n";

            // Sleep
            loop_rate.sleep();
        }

        // Close the file when done
        csvFile.close();

        std::cout << "Finished writing actual data for " << recorder_name_ << std::endl;

        cleanup_cv_.notify_all(); // Wake up cleanup thread
    }
};
/***** EOF Joint Trajectory and EE TF Recorder class *************/

struct JointTrajAndTfRecorderSet{
    std::unique_ptr<JointTrajAndTfRecorder> robot;
    std::unique_ptr<JointTrajAndTfRecorder> gripper;
    std::unique_ptr<JointTrajAndTfRecorder> robot_and_gripper;
};

void validate_output_directory(const std::string &dir_str)
{
    std::filesystem::path dir(dir_str);
    
    // Must exist
    if (!std::filesystem::exists(dir)) {
	throw std::runtime_error(
	    "Recorder directory does not exist: " + dir.string());
    }
    
    // Must be a directory
    if (!std::filesystem::is_directory(dir)) {
	throw std::runtime_error(
	    "Recorder path is not a directory: " + dir.string());
    }
}

} // namespace robot_state_recorder

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("robot_state_recorder");
    auto node_weak_ptr = node.get();

    // Extract planning group info including robot config and action server names for various planning groups
    const std::unordered_map<std::string, cca_ros::PlanningGroupInfo> planning_group_info_map = 
        cca_ros::CcaRos::get_planning_group_info_map(node_weak_ptr);
    // Extract joint states topic
    const std::string joint_states_topic = ros_cpp_util::get_required_str_param(node_weak_ptr, "cca_joint_states_topic");
    // Extract and validate output directory
    const std::string output_dir = ros_cpp_util::get_required_str_param(node_weak_ptr, "output_dir");
    robot_state_recorder::validate_output_directory(output_dir);
    
    // Create recorder map
    std::unordered_map<std::string, robot_state_recorder::JointTrajAndTfRecorderSet> planning_group_recorder_map;

    for (const auto& [pg_name, pg_info] : planning_group_info_map) {
        robot_state_recorder::JointTrajAndTfRecorderSet set;
        
        if (!pg_info.ex_as_names.robot.empty()) {
            // Initialize recorder for robot-only action server
            set.robot = std::make_unique<robot_state_recorder::JointTrajAndTfRecorder>(
                node,
                pg_info.robot_config,
                pg_info.ex_as_names.robot,
                joint_states_topic,
                pg_name + "_robot", 
		output_dir);
        }
        
        if (!pg_info.ex_as_names.gripper.empty()) {
            // Initialize recorder for gripper-only action server
            set.gripper = std::make_unique<robot_state_recorder::JointTrajAndTfRecorder>(
                node,
                pg_info.robot_config,
                pg_info.ex_as_names.gripper,
                joint_states_topic,
                pg_name + "_gripper", 
		output_dir);
        }
        
        if (!pg_info.ex_as_names.robot_and_gripper.empty()) {
            // Initialize recorder for robot-and-gripper combined action server
            set.robot_and_gripper = std::make_unique<robot_state_recorder::JointTrajAndTfRecorder>(
                node,
                pg_info.robot_config,
                pg_info.ex_as_names.robot_and_gripper,
                joint_states_topic,
                pg_name + "_robot_and_gripper",
		output_dir);
        }
        
        planning_group_recorder_map[pg_name] = std::move(set);
    }

    // Ctrl+c signal handling
    signal(SIGINT, signal_callback_handler);

    RCLCPP_INFO(node->get_logger(), "Robot state recorder is active for %zu planning group(s)", 
                planning_group_info_map.size());
    rclcpp::spin(node);

    // Note: On shutdown, the recorders will be properly destroyed via unique_ptr destructors
    rclcpp::shutdown();

    return 0;
}
