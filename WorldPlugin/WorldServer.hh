//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: This is a world plugin template for this simulation that
//              provides an interface to serve as a middle ware. The main 
//              functions of this plugin are managing magnetic connections,
//              loading configurations, interpreting gait commands, managing 
//              and routing communications, providing APIs to manage the
//              shared libraries, managing module configurations, etc.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _GAZEBO_WORLD_SERVER_HH_
#define _GAZEBO_WORLD_SERVER_HH_
#ifndef _GAZEBO_CUTOMIZED_WORLD_PLUGIN
#define _GAZEBO_CUTOMIZED_WORLD_PLUGIN WorldServer
#endif
#include <stdlib.h>

#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iterator>
#include <boost/bind.hpp>
#include <queue>
#include <unistd.h>

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
// Libraries for messages needed to use to communicate between plugins
#include "collision_message.pb.h"
#include "command_message.pb.h"
#include "world_status_message.pb.h"
// XML paser libraries
#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
// Libraries for connectivity representation
#include "SmoresModule.hh"
// Other useful classes for storing information
#include "CollisionInformation.hh"
#include "Condition.hh"
// Library for colored log text
#include "ColorLog.hh"
// Libraries for configuration motion feedback control
#include "LibraryTemplate.hh"

#define PI 3.141593   // 3.1411593
#define VALIDCONNECTIONDISUPPER 0.110
#define VALIDCONNECTIONDISLOWER 0.098
// TODO: tests needed to make sure it depende on current execution folder
#define MODULEPATH "SMORE.sdf"
// #define INTIALCONFIGURATION "InitialConfiguration"

using std::string;
using std::vector;

typedef const boost::shared_ptr
    <const collision_message::msgs::CollisionMessage> CollisionMessagePtr;
typedef const boost::shared_ptr
    <const command_message::msgs::CommandMessage> CommandMessagePtr;
typedef boost::shared_ptr<gazebo::Condition> ConditionPtr;
typedef const boost::shared_ptr
    <const command_message::msgs::WorldStatusMessage> WorldStatusMessagePtr;

// TODO: Considering 
namespace {
inline string Int2String(int number)
{
  stringstream ss; //create a stringstream
  ss << number;    //add number to the stream
  return ss.str(); //return a string with the contents of the stream
} // Int2String
} // namespace 
namespace gazebo
{
class WorldServer : public WorldPlugin
{
 public:
  WorldServer();
  ~WorldServer();
  /// This function will be called in Load() to perform extra initialization
  virtual void ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf);
  /// Load a shared library in linux
  LibraryTemplate *DynamicallyLoadedLibrary(const char* library_path, 
      void *lib_handle);
  /// Close the loaded libraries
  void CloseLoadedLibrary(void **lib_handle);
  virtual void OnSystemRunningExtra(const common::UpdateInfo & _info);
  /// Insert a model to the current world, with joint angles specified
  virtual void InsertModel(string name, math::Pose position, string joint_angles);
  /// Insert a model to the current world, with joint angles and model path specified
  virtual void InsertModel(string name, math::Pose position, string joint_angles, 
      string model_path);
  /// Add a new position to set at the end of 'intialPosition' vector
  void AddInitialPosition(math::Pose position);
  /// Add new initial joint values to set at the end of 'initalJointValue' vector
  void AddInitialJoints(string joint_angles);
  /// Delete the smoresmodule object in the world
  /*!
    \param module_name Module name string
  */
  void DeleteSmoresmodulePtr(string module_name);
  /// Delete a model that already in the world
  /*!
    \param module_name Module name string
  */
  void DeleteModule(string module_name);
  /// Delete all models that already in the world
  void DeleteAllModules(void);
  /// Get the current position in world of the inserted configuration by averaging the pose of all modules
  math::Vector3 GetCurrentConfigurationPose(void);
  /// This function is used to build a configuration at origin using a XML file
  void BuildConfigurationFromXML(string file_name);
  /// This function is used to build a configuration at given initial_pose using a XML file
  void BuildConfigurationFromXML(string file_name, math::Vector3 initial_pose);
  /// This function is used to build connection using a XML file
  void BuildConnectionFromXML(string file_name);
  /// This function will be called after set the model initial position
  virtual void ExtraWorkWhenModelInserted(CommandMessagePtr &msg);
  /// Convert angles so that their absolutely value always smaller than Pi 
  double ConversionForAngleOverPi(double angle);
  /// Enable auto magnetic connection, all module
  /// Have to be called in Load()
  /// Default: disabled
  void EnableAutoMagneticConnection(void);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // These functions are used to connect or disconnect modules
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Connect two modules by pointers and node_ID
  void PassiveConnect(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID, double node_angle, double node_distance);
  /// With default angle and distance offset equal to 0
  void PassiveConnect(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID);
  /// TODO: Need add 'active' feature to this function
  void ActiveConnect(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID, double node_angle, double node_distance);
  /// With default angle and distance offset equal to 0
  void ActiveConnect(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID);
  /// Disconnect two modules on one edge
  /// TODO: This pointer must point to an element in the vector
  void Disconnect(SmoresEdgePtr aEdge);
  /// Disconnect two module base on one module and one node of that module
  void Disconnect(SmoresModulePtr aModule, int node_ID);
  /// Disconnect two module base on one module and one node of that module
  void Disconnect(string moduleName, int node_ID);
  /// Disconnect two module base on their names
  void Disconnect(string moduleName1, string moduleName2);

  // Theses functions are used to send 'gait table'
  // TODO: Think about how to combine these functions
  /// Send time based common gait with condition and dependency
  /*!
    \param module A pointer of SmoresModule object
    \param flag Message type for each joint angle
    0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore; 4: connect(Not implemented); 5: disconnect(Not implemented)
    \param gait_value Joints values, either be position, speed or torque
    \param msg_type Defined in command_message.proto, 3 for joint control
    \param time_stamp Time based gait table timer
    \param condition_str Condition string
    \param dependency_str Dependency string
  */
  void SendGaitTable(SmoresModulePtr module, const int *flag, 
      const double *gait_value, int msg_type, unsigned int time_stamp, 
      string condition_str, string dependency_str);
  /// Send time based common gait without condition and dependency
  /*!
    \param module A pointer of SmoresModule object
    \param flag Message type for each joint angle
    0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore; 4: connect(Not implemented); 5: disconnect(Not implemented)
    \param gait_value Joints values, either be position, speed or torque
    \param msg_type Defined in command_message.proto, 3 for joint control
    \param time_stamp Time based gait table timer
  */
  void SendGaitTable(SmoresModulePtr module, const int *flag, 
      const double *gait_value, int msg_type, unsigned int time_stamp);
  /// Send common gait with condition and dependency
  /*!
    \param module A pointer of SmoresModule object
    \param flag Message type for each joint angle
    0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore; 4: connect(Not implemented); 5: disconnect(Not implemented)
    \param gait_value Joints values, either be position, speed or torque
    \param msg_type Defined in command_message.proto, 3 for joint control
    \param condition_str Condition string
    \param dependency_str Dependency string
  */
  void SendGaitTable(SmoresModulePtr module, const int *flag, 
      const double *gait_value, int msg_type, 
      string condition_str, string dependency_str);
  /// Send common gait without condition and dependency
  /*!
    \param module A pointer of SmoresModule object
    \param flag Message type for each joint angle
    0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore; 4: connect(Not implemented); 5: disconnect(Not implemented)
    \param gait_value Joints values, either be position, speed or torque
    \param msg_type Defined in command_message.proto, 3 for joint control
  */
  void SendGaitTable(SmoresModulePtr module, const int *flag, 
      const double *gait_value, int msg_type);
  /// Send gait table to change a single joint of a module
  /*!
    \param module A pointer of SmoresModule object
    \param joint_ID Number index of a joint
    \param gait_value Joint value, either be position, speed or torque
    \param msg_type  0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore;
    \param time_stamp Time based gait table timer
    \param condition_str Condition string
    \param dependency_str Dependency string
  */
  void SendGaitTable(SmoresModulePtr module, int joint_ID, 
      double gait_value, int msg_type, unsigned int time_stamp, 
      string condition_str, string dependency_str);
  /// Send gait table to change a single joint of a module
  /*!
    \param module A pointer of SmoresModule object
    \param joint_ID Number index of a joint
    \param gait_value Joint value, either be position, speed or torque
    \param msg_type  0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore;
    \param time_stamp Time based gait table timer
  */
  void SendGaitTable(SmoresModulePtr module, int joint_ID, 
      double gait_value, int msg_type, unsigned int time_stamp);
  /// Send gait table to change a single joint of a module
  /*!
    \param module A pointer of SmoresModule object
    \param joint_ID Number index of a joint
    \param gait_value Joint value, either be position, speed or torque
    \param msg_type  0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore;
    \param condition_str Condition string
    \param dependency_str Dependency string
  */
  void SendGaitTable(SmoresModulePtr module, int joint_ID, 
      double gait_value, int msg_type, 
      string condition_str, string dependency_str);
  /// Send gait table to change a single joint of a module
  /*!
    \param module A pointer of SmoresModule object
    \param joint_ID Number index of a joint
    \param gait_value Joint value, either be position, speed or torque
    \param msg_type  0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore;
  */
  void SendGaitTable(SmoresModulePtr module, int joint_ID, 
      double gait_value, int msg_type);
  /// Send connect or disconnect command, 
  /// with condition or dependency and time based
  /*!
    \param module A pointer of SmoresModule object of module 1
    \param module1 Name string of module 1
    \param module2 Name string of module 2
    \param node1 Number index of the joint of module 1
    \param node2 Number index of the joint of module 2
    \param commandtype Special command indicator, 1 connect; 2 disconnect
    \param time_stamp Time based gait table timer
    \param condition_str Condition string
    \param dependency_str Dependency string
  */
  void SendGaitTable(SmoresModulePtr module, string module1, string module2, 
      int node1, int node2, int commandtype, unsigned int time_stamp, 
      string condition_str, string dependency_str);
  /// Send connect or disconnect command, 
  /// without condition or dependency and time based
  /*!
    \param module A pointer of SmoresModule object of module 1
    \param module1 Name string of module 1
    \param module2 Name string of module 2
    \param node1 Number index of the joint of module 1
    \param node2 Number index of the joint of module 2
    \param commandtype Special command indicator, 1 connect; 2 disconnect
    \param time_stamp Time based gait table timer
  */
  void SendGaitTable(SmoresModulePtr module, string module1, string module2, 
      int node1, int node2, int commandtype, unsigned int time_stamp);
  /// Send connect or disconnect command, 
  /// with condition or dependency
  /*!
    \param module A pointer of SmoresModule object of module 1
    \param module1 Name string of module 1
    \param module2 Name string of module 2
    \param node1 Number index of the joint of module 1
    \param node2 Number index of the joint of module 2
    \param commandtype Special command indicator, 1 connect; 2 disconnect
    \param condition_str Condition string
    \param dependency_str Dependency string
  */
  void SendGaitTable(SmoresModulePtr module, string module1, string module2, 
      int node1, int node2, int commandtype, 
      string condition_str, string dependency_str);
  /// Send connect or disconnect command, 
  /// without condition or dependency
  /*!
    \param module A pointer of SmoresModule object of module 1
    \param module1 Name string of module 1
    \param module2 Name string of module 2
    \param node1 Number index of the joint of module 1
    \param node2 Number index of the joint of module 2
    \param commandtype Special command indicator, 1 connect; 2 disconnect
  */
  void SendGaitTable(SmoresModulePtr module, string module1, string module2, 
      int node1, int node2, int commandtype);
  /// Used in the direct driving situation, ignore execution order
  /*!
    \param module A pointer of SmoresModule object
    \param flag Message type for each joint angle
    0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore; 4: connect(Not implemented); 5: disconnect(Not implemented)
    \param gait_value Joints values, either be position, speed or torque
    \param msg_type Defined in command_message.proto, 4 for direct drive
  */
  void SendGaitTableInstance(SmoresModulePtr module, const int *flag, 
      const double *gait_value, int msg_type);
  /// Used in the direct driving situation with default type 4
  /*!
    \param module A pointer of SmoresModule object
    \param flag Message type for each joint angle
    0: position; 1: speed; 2: torque(Not implemented); 
    3: ignore; 4: connect(Not implemented); 5: disconnect(Not implemented)
    \param gait_value Joints values, either be position, speed or torque
  */
  void SendGaitTableInstance(SmoresModulePtr module, const int *flag, 
      const double *gait_value);
  /// Used in direct control, dirve a module to a specific position
  /*!
    \param module A pointer of SmoresModule object
    \param x 2D coordinate x
    \param y 2D coordinate y
    \param orientation_angle Final orientation of the module in global frame
  */
  void SendPositionInstance(SmoresModulePtr module, double x, double y, 
      double orientation_angle);
  /// Erase all the existing commands of a specific module
  /// TODO: Need to be tested
  void EraseComaands(SmoresModulePtr module);
  /// Destroyer of the connection between different modules,
  /// which is dynamic joint here
  void DynamicJointDestroy(SmoresEdgePtr aEdge);
  /// Rebuild the dynamic joint using the information in the edge object
  void ReBuildDynamicJoint(SmoresEdgePtr a_edge);

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // These functions are utility functions
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  int GetNodeIDByName(string node_name);
  /// Get return true if the smores module with given name exist in the world
  bool CheckModuleExistByName(string module_name);
  /// Get SmoresModule object by specifying the name
  /*!
    \param module_name Module name string
    \return A pointer of SmoresModule object
  */
  SmoresModulePtr GetModulePtrByName(string module_name);
  /// Get SmoresModule object by specifying the index in the vector moduleList
  SmoresModulePtr GetModulePtrByIDX(unsigned int idx);
  /// Get the count of the modules that are in the list
  unsigned int GetModuleListSize(void);
  void EraseCommandPtrByModule(SmoresModulePtr module_ptr);
  int GetModuleIndexByName(string module_name);
  /// Check whether two nodes are connected together
  bool AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID);
  /// Check whether two modules have already connected on a node
  bool AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2);
  /// Check whether a node a of module has been occupied
  bool AlreadyConnected(SmoresModulePtr module, int node_ID);
  /// Read a 'gait table' stored in a text file
  void ReadFileAndGenerateCommands(const char* fileName);
  /// Used to interpret the number in gait table 
  /*!
    \param joints_spec String tokens of the joint specs
    \param type_flags Joint control flag, see SendGaitTable (return)
    \param joint_values Joint control values (return)
  */
  void FigureInterpret(const vector<string> *joints_spec, int *type_flags, 
      double *joint_values);
  /// Interpret normal command string
  /*!
    \param a_command_str A command string
  */
  void InterpretCommonGaitString(string a_command_str);
  /// Interpret special command
  /*!
    \param a_command_str A command string
  */
  void InterpretSpecialString(string a_command_str);
  /// Count how many modules are there in a cluster with the current module
  /*!
    \param module A pointer of SmoresModule object
  */
  unsigned int CountModules(SmoresModulePtr module);
  /// Get the length of the initial joint value setting sequence
  unsigned int GetInitialJointSequenceSize(void);
  /// Get the total counts of the edges that stored in the program
  unsigned int GetEdgeCounts(void);
  /// Get SmoresEdge object by specifying the index in the vector connectionEdges
  SmoresEdgePtr GetEdgePtrByIDX(unsigned int idx);

 private: 
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
  /// Callback when there is an entity added to the world
  void AddEntityToWorld(std::string & _info);
  /// This function will be called in the every iteration of the simulation
  void OnSystemRunning(const common::UpdateInfo & /*_info*/);
  /// Command information receiving callback
  void FeedBackMessageDecoding(CommandMessagePtr &msg);
  /// World status information receiving callback
  void WorldStatusMessageDecoding(WorldStatusMessagePtr &msg);
  /// Collision information receiving callback
  /// Used by automatic magnetic connection
  /// TODO: Need to be enabled by each individual module
  void AutomaticMagneticConnectionManagement(CollisionMessagePtr &msg);
  /// The function is used to physically connect different models 
  /// by generating dynamic joint
  void ConnectAndDynamicJointGeneration(SmoresModulePtr module_1, 
      SmoresModulePtr module_2, int node1_ID, int node2_ID, SmoresEdgePtr an_edge);
  /// Calculate the rotation matrix of cluster 2
  /// Used in 'ConnectAndDynamicJointGeneration'
  void RotationQuaternionCalculation(math::Vector3 normal_axis,
    math::Vector3 z_axis_of_link1, math::Vector3 z_axis_of_link2, 
    math::Vector3 first_rotation, math::Vector3 second_rotation,
    math::Quaternion *first_rotation_of_link2, 
    math::Quaternion *second_rotation_of_link2);
  /// Calculate the new position of the connecting modules
  void NewPositionCalculation(SmoresEdgePtr an_edge,
    math::Pose old_pose_of_module1, math::Pose old_pose_of_module2, 
    int node1_ID, int node2_ID, 
    math::Pose *new_pose_of_module1, math::Pose *new_pose_of_module2);
  /// These functions are used to manage and send message to model
  void CommandManager(void); 
  void CommandExecution(ModuleCommandsPtr current_command_container);
  /// Condition manipulation
  void AddCondition(string conditionid);
  void FinishOneConditionCommand(string conditionid);
  bool CheckCondition(string conditionid);
  /// Find the timer in a command string
  int StripOffTimerInCommandString(string &command_string);
  /// Find the condition in a command string
  string StripOffCondition(string &command_string);
  /// Find the dpendency in a command string
  string StripOffDependency(string &command_string);

 public:
  physics::WorldPtr currentWorld;
 private: 
  event::ConnectionPtr addEntityConnection;
  transport::PublisherPtr welcomePub;

  transport::PublisherPtr smoreWorldPub;
  transport::SubscriberPtr smoreWorldSub;
  /// The pointer vector for all the models in the world
  vector<SmoresModulePtr> moduleList;
  /// The vectors that store the pending connections request and information
  vector<CollisionInformation> pendingRequest;
  /// The vector for connection record
  vector<physics::JointPtr> dynamicConnections;
  /// The event that will be refreshed in every iteration of the simulation
  event::ConnectionPtr updateConnection;
  /// The container that has all the edges
  vector<SmoresEdgePtr> connectionEdges;
  /// List of model names that need to assign the Model object pointer
  vector<string> waitingNameList;
  /// A String vector which contain the initial joint angles of modules
  vector<string> initalJointValue;
  /// Vector that stores the initial position when building a configuration
  vector<math::Pose> initialPosition;
  /// A vector created for command management
  vector<ConditionPtr> commandConditions;
  vector<ModuleCommandsPtr> moduleCommandContainer;
// private: vector<transport::PublisherPtr> WorldPublisher;
  /// This is used for automatical magnetic connection
  vector<transport::SubscriberPtr> WorldColSubscriber;
  /// Auto Magnetic Connection Enable Flag
  bool autoMagneticConnectionFlag;
  /// The path of the file of configuration
  string configurationFile;
//+++++++++ testing ++++++++++++++++++++++++++++
};
} // namespace gazebo
#endif