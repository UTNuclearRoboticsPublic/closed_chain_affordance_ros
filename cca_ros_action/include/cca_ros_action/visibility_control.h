#ifndef CCA_ROS_ACTION_CPP__VISIBILITY_CONTROL_H_
#define CCA_ROS_ACTION_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CCA_ROS_ACTION_CPP_EXPORT __attribute__ ((dllexport))
    #define CCA_ROS_ACTION_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define CCA_ROS_ACTION_CPP_EXPORT __declspec(dllexport)
    #define CCA_ROS_ACTION_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef CCA_ROS_ACTION_CPP_BUILDING_DLL
    #define CCA_ROS_ACTION_CPP_PUBLIC CCA_ROS_ACTION_CPP_EXPORT
  #else
    #define CCA_ROS_ACTION_CPP_PUBLIC CCA_ROS_ACTION_CPP_IMPORT
  #endif
  #define CCA_ROS_ACTION_CPP_PUBLIC_TYPE CCA_ROS_ACTION_CPP_PUBLIC
  #define CCA_ROS_ACTION_CPP_LOCAL
#else
  #define CCA_ROS_ACTION_CPP_EXPORT __attribute__ ((visibility("default")))
  #define CCA_ROS_ACTION_CPP_IMPORT
  #if __GNUC__ >= 4
    #define CCA_ROS_ACTION_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define CCA_ROS_ACTION_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CCA_ROS_ACTION_CPP_PUBLIC
    #define CCA_ROS_ACTION_CPP_LOCAL
  #endif
  #define CCA_ROS_ACTION_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CCA_ROS_ACTION_CPP__VISIBILITY_CONTROL_H_
