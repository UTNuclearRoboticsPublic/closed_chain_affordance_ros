///////////////////////////////////////////////////////////////////////////////
//      Title     : cca_ros_behavior_util.hpp
//      Project   : cca_ros_behavior
//      Created   : Spring 2025
//      Author    : Janak Panthi (Crasun Jans)
///////////////////////////////////////////////////////////////////////////////

#ifndef CCA_ROS_BEHAVIOR_UTIL_HPP
#define CCA_ROS_BEHAVIOR_UTIL_HPP

#include <behaviortree_cpp/action_node.h>
#include <memory>
#include <string>
#include <type_traits>
#include <cca_ros/cca_ros.hpp>

namespace cca_ros_behavior
{

/**
 * @brief Base template class converting an enum input to an output of specified type.
 *
 * This class serves as a base for Behavior Tree nodes that transform an enum value
 * (input port "enum") into an output value of type OutputType (output port "value").
 * Derived classes must override the pure virtual `toOutput()` to provide the
 * logic for this conversion.
 *
 * Output values are managed through std::shared_ptr to enable safe shared ownership.
 *
 * @tparam EnumType Enum type used as input.
 * @tparam OutputType Output type that the enum value is to be converted to.
 */
template <typename EnumType, typename OutputType> class EnumToType : public BT::SyncActionNode
{
  public:
    /**
     * @brief Compile-time check to ensure EnumType is an enum.
     */
    static_assert(std::is_enum<EnumType>::value, "Template parameter must be an enum type");

    /**
     * @brief Constructs the node with a name and configuration.
     *
     * @param name The node's unique name in the Behavior Tree.
     * @param config Node configuration including ports.
     */
    inline EnumToType(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config) {}

    /**
     * @brief Pure virtual function defining how to convert an enum value to an output.
     *
     * Derived classes implement this to provide specific mapping logic.
     *
     * @param enum_value The input enum value.
     * @return Shared pointer to the mapped output instance.
     */
    virtual std::shared_ptr<OutputType> toOutput(const EnumType &enum_value) = 0;

    /**
     * @brief Declares input and output ports used by the node.
     *
     * Input port "enum" receives the enum value.
     * Output port "value" provides the converted output.
     *
     * @return List of ports used by the node.
     */
    inline static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<EnumType>("enum_name"),
            BT::OutputPort<std::shared_ptr<OutputType>>("value")
        };
    }

    /**
     * @brief Execution function called when the node is ticked.
     *
     * Reads the enum input, calls `toOutput()` to get the output,
     * and sets the output port value.
     *
     * @return BT::NodeStatus::SUCCESS on successful tick.
     * @throws BT::RuntimeError if the enum input is missing.
     */
    inline BT::NodeStatus tick() override
    {
        auto result = getInput<EnumType>("enum_name");
        if (!result)
        {
            throw BT::RuntimeError("Missing required input [enum]: ", result.error());
        }

        EnumType enum_name = result.value();
        auto value = toOutput(enum_name);

        setOutput("value", value);

        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * @brief Template for nodes converting an enum to a single PlanningRequest.
 *
 * Specializes EnumToType with OutputType as cca_ros::PlanningRequest.
 * Must implement `toOutput()` in subclasses.
 *
 * @tparam EnumType Enum input type.
 */
template <typename EnumType> class EnumToReq : public EnumToType<EnumType, cca_ros::PlanningRequest>
{
  public:

    /**
     * @brief Constructs the node with a name and configuration.
     *
     * @param name The node's unique name in the Behavior Tree.
     * @param config Node configuration including ports.
     */
    inline EnumToReq(const std::string &name, const BT::NodeConfig &config) : EnumToType<EnumType, cca_ros::PlanningRequest>(name, config){}

};

/**
 * @brief Template for nodes converting an enum to multiple PlanningRequests.
 *
 * Specializes EnumToType with OutputType as cca_ros::PlanningRequests.
 * Must implement `toOutput()` in subclasses.
 *
 * @tparam EnumType Enum input type.
 */
template <typename EnumType> class EnumToReqs : public EnumToType<EnumType, cca_ros::PlanningRequests>
{
  public:

    /**
     * @brief Constructs the node with a name and configuration.
     *
     * @param name The node's unique name in the Behavior Tree.
     * @param config Node configuration including ports.
     */
    inline EnumToReqs(const std::string &name, const BT::NodeConfig &config) : EnumToType<EnumType, cca_ros::PlanningRequests>(name, config){}

};

} // namespace cca_ros_behavior

#endif // CCA_ROS_BEHAVIOR_UTIL_HPP

