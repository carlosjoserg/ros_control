////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF, INC and Willow Garage, Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the names of Willow Garage, Inc., hiDOF Inc, nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////
/*
 * Author: Wim Meeussen
 */

#include "controller_manager/controller_manager.h"
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include <rcutils/logging_macros.h>
#include <controller_manager/controller_loader.h>
#include <controller_manager_msgs/msg/controller_state.hpp>
#include <chrono>
#include <thread>
#include <functional>

namespace controller_manager{


ControllerManager::ControllerManager(hardware_interface::RobotHW *robot_hw, const rclcpp::Node& nh) :
  robot_hw_(robot_hw),
  root_nh_(nh.get_name()),
  cm_node_(nh.get_name(), "controller_manager"),
  start_request_(0),
  stop_request_(0),
  please_switch_(false),
  current_controllers_list_(0),
  used_by_realtime_(-1)
{
  // create controller loader
  controller_loaders_.push_back(
    ControllerLoaderInterfaceSharedPtr(new ControllerLoader<controller_interface::ControllerBase>("controller_interface",
                                                                                                  "controller_interface::ControllerBase") ) );

  // Advertise services (this should be the last thing we do in init)
  srv_list_controllers_ = cm_node_.create_service<controller_manager_msgs::srv::ListControllers>("list_controllers", std::bind(&ControllerManager::listControllersSrv, this, std::placeholders::_1, std::placeholders::_2));
  srv_list_controller_types_ = cm_node_.create_service<controller_manager_msgs::srv::ListControllerTypes>("list_controller_types", std::bind(&ControllerManager::listControllerTypesSrv, this, std::placeholders::_1, std::placeholders::_2));
  srv_load_controller_ = cm_node_.create_service<controller_manager_msgs::srv::LoadController>("load_controller", std::bind(&ControllerManager::loadControllerSrv, this, std::placeholders::_1, std::placeholders::_2));
  srv_unload_controller_ = cm_node_.create_service<controller_manager_msgs::srv::UnloadController>("unload_controller", std::bind(&ControllerManager::unloadControllerSrv, this, std::placeholders::_1, std::placeholders::_2));
  srv_switch_controller_ = cm_node_.create_service<controller_manager_msgs::srv::SwitchController>("switch_controller", std::bind(&ControllerManager::switchControllerSrv, this, std::placeholders::_1, std::placeholders::_2));
  srv_reload_libraries_ = cm_node_.create_service<controller_manager_msgs::srv::ReloadControllerLibraries>("reload_controller_libraries", std::bind(&ControllerManager::reloadControllerLibrariesSrv, this, std::placeholders::_1, std::placeholders::_2));
}


ControllerManager::~ControllerManager()
{}




// Must be realtime safe.
void ControllerManager::update(const rclcpp::Clock& time, const rclcpp::Duration& period, bool reset_controllers)
{
  used_by_realtime_ = current_controllers_list_;
  std::vector<ControllerSpec> &controllers = controllers_lists_[used_by_realtime_];

  // Restart all running controllers if motors are re-enabled
  if (reset_controllers){
    for (size_t i=0; i<controllers.size(); i++){
      if (controllers[i].c->isRunning()){
        controllers[i].c->stopRequest(time);
        controllers[i].c->startRequest(time);
      }
    }
  }


  // Update all controllers
  for (size_t i=0; i<controllers.size(); i++)
    controllers[i].c->updateRequest(time, period);

  // there are controllers to start/stop
  if (please_switch_)
  {
    // switch hardware interfaces (if any)
    robot_hw_->doSwitch(switch_start_list_, switch_stop_list_);

    // stop controllers
    for (unsigned int i=0; i<stop_request_.size(); i++)
      if (!stop_request_[i]->stopRequest(time))
        RCUTILS_LOG_FATAL("Failed to stop controller in realtime loop. This should never happen.");

    // start controllers
    for (unsigned int i=0; i<start_request_.size(); i++)
      if (!start_request_[i]->startRequest(time))
        RCUTILS_LOG_FATAL("Failed to start controller in realtime loop. This should never happen.");

    please_switch_ = false;
  }
}

controller_interface::ControllerBase* ControllerManager::getControllerByName(const std::string& name)
{
  // Lock recursive mutex in this context
  boost::recursive_mutex::scoped_lock guard(controllers_lock_);

  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    if (controllers[i].info.name == name)
      return controllers[i].c.get();
  }
  return NULL;
}

void ControllerManager::getControllerNames(std::vector<std::string> &names)
{
  boost::recursive_mutex::scoped_lock guard(controllers_lock_);
  names.clear();
  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    names.push_back(controllers[i].info.name);
  }
}


bool ControllerManager::loadController(const std::string& name)
{
  RCUTILS_LOG_DEBUG("Will load controller '%s'", name.c_str());

  // lock controllers
  boost::recursive_mutex::scoped_lock guard(controllers_lock_);

  // get reference to controller list
  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (rclcpp::ok() && free_controllers_list == used_by_realtime_)
  {
    if (!rclcpp::ok())
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // Copy all controllers from the 'from' list to the 'to' list
  for (size_t i = 0; i < from.size(); ++i)
    to.push_back(from[i]);

  // Checks that we're not duplicating controllers
  for (size_t j = 0; j < to.size(); ++j)
  {
    if (to[j].info.name == name)
    {
      to.clear();
      RCUTILS_LOG_ERROR("A controller named '%s' was already loaded inside the controller manager", name.c_str());
      return false;
    }
  }

  rclcpp::Node c_nh(root_nh_.get_name(), name);
  // Constructs the controller
  /*try{
    c_nh = ;
  }
  catch(std::exception &e) {
    RCUTILS_LOG_ERROR("Exception thrown while constructing nodehandle for controller with name '%s':\n%s", name.c_str(), e.what());
    return false;
  }
  catch(...){
    RCUTILS_LOG_ERROR("Exception thrown while constructing nodehandle for controller with name '%s'", name.c_str());
    return false;
  }*/
  controller_interface::ControllerBaseSharedPtr c;
  std::string type;
  if (c_nh.get_parameter("type", type))
  {
    RCUTILS_LOG_DEBUG("Constructing controller '%s' of type '%s'", name.c_str(), type.c_str());
    try
    {
      // Trying loading the controller using all of our controller loaders. Exit once we've found the first valid loaded controller
      std::list<ControllerLoaderInterfaceSharedPtr>::iterator it = controller_loaders_.begin();
      while (!c && it != controller_loaders_.end())
      {
        std::vector<std::string> cur_types = (*it)->getDeclaredClasses();
        for(size_t i=0; i < cur_types.size(); i++){
          if (type == cur_types[i]){
            c = (*it)->createInstance(type);
          }
        }
        ++it;
      }
    }
    catch (const std::runtime_error &ex)
    {
      RCUTILS_LOG_ERROR("Could not load class '%s': %s", type.c_str(), ex.what());
    }
  }
  else
  {
    RCUTILS_LOG_ERROR("Could not load controller '%s' because the type was not specified. Did you load the controller configuration on the parameter server (namespace: '%s')?", name.c_str(), c_nh.get_namespace());
    to.clear();
    return false;
  }

  // checks if controller was constructed
  if (!c)
  {
    RCUTILS_LOG_ERROR("Could not load controller '%s' because controller type '%s' does not exist.",  name.c_str(), type.c_str());
    RCUTILS_LOG_ERROR("Use 'rosservice call controller_manager/list_controller_types' to get the available types");
    to.clear();
    return false;
  }

  // Initializes the controller
  RCUTILS_LOG_DEBUG("Initializing controller '%s'", name.c_str());
  bool initialized;
  controller_interface::ControllerBase::ClaimedResources claimed_resources; // Gets populated during initRequest call
  try{
    initialized = c->initRequest(robot_hw_, root_nh_, c_nh, claimed_resources);
  }
  catch(std::exception &e){
    RCUTILS_LOG_ERROR("Exception thrown while initializing controller '%s'.\n%s", name.c_str(), e.what());
    initialized = false;
  }
  catch(...){
    RCUTILS_LOG_ERROR("Exception thrown while initializing controller '%s'", name.c_str());
    initialized = false;
  }
  if (!initialized)
  {
    to.clear();
    RCUTILS_LOG_ERROR("Initializing controller '%s' failed", name.c_str());
    return false;
  }
  RCUTILS_LOG_DEBUG("Initialized controller '%s' successful", name.c_str());

  // Adds the controller to the new list
  to.resize(to.size() + 1);
  to.back().info.type = type;
  to.back().info.name = name;
  to.back().info.claimed_resources = claimed_resources;
  to.back().c = c;

  // Destroys the old controllers list when the realtime thread is finished with it.
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;
  while (rclcpp::ok() && used_by_realtime_ == former_current_controllers_list_)
  {
    if (!rclcpp::ok())
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  from.clear();

  RCUTILS_LOG_DEBUG("Successfully load controller '%s'", name.c_str());
  return true;
}




bool ControllerManager::unloadController(const std::string &name)
{
  RCUTILS_LOG_DEBUG("Will unload controller '%s'", name.c_str());

  // lock the controllers
  boost::recursive_mutex::scoped_lock guard(controllers_lock_);

  // get reference to controller list
  int free_controllers_list = (current_controllers_list_ + 1) % 2;
  while (rclcpp::ok() && free_controllers_list == used_by_realtime_)
  {
    if (!rclcpp::ok())
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  std::vector<ControllerSpec>
    &from = controllers_lists_[current_controllers_list_],
    &to = controllers_lists_[free_controllers_list];
  to.clear();

  // Transfers the running controllers over, skipping the one to be removed and the running ones.
  bool removed = false;
  for (size_t i = 0; i < from.size(); ++i)
  {
    if (from[i].info.name == name){
      if (from[i].c->isRunning()){
        to.clear();
        RCUTILS_LOG_ERROR("Could not unload controller with name '%s' because it is still running",
                  name.c_str());
        return false;
      }
      removed = true;
    }
    else
      to.push_back(from[i]);
  }

  // Fails if we could not remove the controllers
  if (!removed)
  {
    to.clear();
    RCUTILS_LOG_ERROR("Could not unload controller with name '%s' because no controller with this name exists",
              name.c_str());
    return false;
  }

  // Destroys the old controllers list when the realtime thread is finished with it.
  RCUTILS_LOG_DEBUG("Realtime switches over to new controller list");
  int former_current_controllers_list_ = current_controllers_list_;
  current_controllers_list_ = free_controllers_list;
  while (rclcpp::ok() && used_by_realtime_ == former_current_controllers_list_)
  {
    if (!rclcpp::ok())
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  RCUTILS_LOG_DEBUG("Destruct controller");
  from.clear();
  RCUTILS_LOG_DEBUG("Destruct controller finished");

  RCUTILS_LOG_DEBUG("Successfully unloaded controller '%s'", name.c_str());
  return true;
}



bool ControllerManager::switchController(const std::vector<std::string>& start_controllers,
                                         const std::vector<std::string>& stop_controllers,
                                         int strictness)
{
  if (!stop_request_.empty() || !start_request_.empty())
    RCUTILS_LOG_FATAL("The internal stop and start request lists are not empty at the beginning of the swithController() call. This should not happen.");

  if (strictness == 0){
    RCUTILS_LOG_WARN("Controller Manager: To switch controllers you need to specify a strictness level of controller_manager_msgs::srv::SwitchController::STRICT (%d) or ::BEST_EFFORT (%d). Defaulting to ::BEST_EFFORT.",
             controller_manager_msgs::srv::SwitchController::Request::STRICT,
             controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);
    strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  }

  RCUTILS_LOG_DEBUG("switching controllers:");
  for (unsigned int i=0; i<start_controllers.size(); i++)
    RCUTILS_LOG_DEBUG(" - starting controller '%s'", start_controllers[i].c_str());
  for (unsigned int i=0; i<stop_controllers.size(); i++)
    RCUTILS_LOG_DEBUG(" - stopping controller '%s'", stop_controllers[i].c_str());

  // lock controllers
  boost::recursive_mutex::scoped_lock guard(controllers_lock_);

  controller_interface::ControllerBase* ct;
  // list all controllers to stop
  for (unsigned int i=0; i<stop_controllers.size(); i++)
  {
    ct = getControllerByName(stop_controllers[i]);
    if (ct == NULL){
      if (strictness ==  controller_manager_msgs::srv::SwitchController::Request::STRICT){
        RCUTILS_LOG_ERROR("Could not stop controller with name '%s' because no controller with this name exists",
                  stop_controllers[i].c_str());
        stop_request_.clear();
        return false;
      }
      else{
        RCUTILS_LOG_DEBUG("Could not stop controller with name '%s' because no controller with this name exists",
                  stop_controllers[i].c_str());
      }
    }
    else{
      RCUTILS_LOG_DEBUG("Found controller '%s' that needs to be stopped in list of controllers",
                stop_controllers[i].c_str());
      stop_request_.push_back(ct);
    }
  }
  RCUTILS_LOG_DEBUG("Stop request vector has size %i", (int)stop_request_.size());

  // list all controllers to start
  for (unsigned int i=0; i<start_controllers.size(); i++)
  {
    ct = getControllerByName(start_controllers[i]);
    if (ct == NULL){
      if (strictness ==  controller_manager_msgs::srv::SwitchController::Request::STRICT){
        RCUTILS_LOG_ERROR("Could not start controller with name '%s' because no controller with this name exists",
                  start_controllers[i].c_str());
        stop_request_.clear();
        start_request_.clear();
        return false;
      }
      else{
        RCUTILS_LOG_DEBUG("Could not start controller with name '%s' because no controller with this name exists",
                  start_controllers[i].c_str());
      }
    }
    else{
      RCUTILS_LOG_DEBUG("Found controller '%s' that needs to be started in list of controllers",
                start_controllers[i].c_str());
      start_request_.push_back(ct);
    }
  }
  RCUTILS_LOG_DEBUG("Start request vector has size %i", (int)start_request_.size());

  // Do the resource management checking
  std::list<hardware_interface::ControllerInfo> info_list;
  switch_start_list_.clear();
  switch_stop_list_.clear();

  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    bool in_stop_list  = false;
    for(size_t j = 0; j < stop_request_.size(); j++)
    {
      if (stop_request_[j] == controllers[i].c.get())
      {
        in_stop_list = true;
        break;
      }
    }

    bool in_start_list = false;
    for(size_t j = 0; j < start_request_.size(); j++)
    {
      if (start_request_[j] == controllers[i].c.get())
      {
        in_start_list = true;
        break;
      }
    }

    const bool is_running = controllers[i].c->isRunning();
    hardware_interface::ControllerInfo &info = controllers[i].info;

    if(!is_running && in_stop_list){ // check for double stop
      if(strictness ==  controller_manager_msgs::srv::SwitchController::Request::STRICT){
        std::string message = "Could not stop controller '";
        message += info.name;
        message += "' since it is not running";
        RCUTILS_LOG_ERROR(message.c_str());
        stop_request_.clear();
        start_request_.clear();
        return false;
      } else {
        in_stop_list = false;
      }
    }

    if(is_running && !in_stop_list && in_start_list){ // check for doubled start
      if(strictness ==  controller_manager_msgs::srv::SwitchController::Request::STRICT){
        std::string message = "Controller '";
        message += info.name;
        message += "' is already running";
        RCUTILS_LOG_ERROR(message.c_str());
        stop_request_.clear();
        start_request_.clear();
        return false;
      } else {
        in_start_list = false;
      }
    }

    if(is_running && in_stop_list && !in_start_list){ // running and real stop
      switch_stop_list_.push_back(info);
    }
    else if(!is_running && !in_stop_list && in_start_list){ // start, but no restart
      switch_start_list_.push_back(info);
     }

    bool add_to_list = is_running;
    if (in_stop_list)
      add_to_list = false;
    if (in_start_list)
      add_to_list = true;

    if (add_to_list)
      info_list.push_back(info);
  }

  bool in_conflict = robot_hw_->checkForConflict(info_list);
  if (in_conflict)
  {
    RCUTILS_LOG_ERROR("Could not switch controllers, due to resource conflict");
    stop_request_.clear();
    start_request_.clear();
    return false;
  }

  if (!robot_hw_->prepareSwitch(switch_start_list_, switch_stop_list_))
  {
    RCUTILS_LOG_ERROR("Could not switch controllers. The hardware interface combination for the requested controllers is unfeasible.");
    stop_request_.clear();
    start_request_.clear();
    return false;
  }

  // start the atomic controller switching
  switch_strictness_ = strictness;
  please_switch_ = true;

  // wait until switch is finished
  RCUTILS_LOG_DEBUG("Request atomic controller switch from realtime loop");
  while (rclcpp::ok() && please_switch_)
  {
    if (!rclcpp::ok())
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  start_request_.clear();
  stop_request_.clear();

  RCUTILS_LOG_DEBUG("Successfully switched controllers");
  return true;
}

void ControllerManager::reloadControllerLibrariesSrv(
  const std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Request> req,
  std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Response> resp)
{
  // lock services
  RCUTILS_LOG_DEBUG("reload libraries service called");
  boost::mutex::scoped_lock guard(services_lock_);
  RCUTILS_LOG_DEBUG("reload libraries service locked");

  // only reload libraries if no controllers are running
  std::vector<std::string> controllers;
  getControllerNames(controllers);
  if (!controllers.empty() && !req->force_kill){
    RCUTILS_LOG_ERROR("Controller manager: Cannot reload controller libraries because there are still %i controllers running", (int)controllers.size());
    resp->ok = false;
    return;
  }

  // kill running controllers if requested
  if (!controllers.empty()){
    RCUTILS_LOG_INFO("Controller manager: Killing all running controllers");
    std::vector<std::string> empty;
    if (!switchController(empty,controllers, controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT)){
      RCUTILS_LOG_ERROR("Controller manager: Cannot reload controller libraries because failed to stop running controllers");
      resp->ok = false;
      return;
    }
    for (unsigned int i=0; i<controllers.size(); i++){
      if (!unloadController(controllers[i])){
        RCUTILS_LOG_ERROR("Controller manager: Cannot reload controller libraries because failed to unload controller '%s'",
                  controllers[i].c_str());
        resp->ok = false;
        return;
      }
    }
    getControllerNames(controllers);
  }
  assert(controllers.empty());

  // Force a reload on all the PluginLoaders (internally, this recreates the plugin loaders)
  for(std::list<ControllerLoaderInterfaceSharedPtr>::iterator it = controller_loaders_.begin(); it != controller_loaders_.end(); ++it)
  {
    (*it)->reload();
    RCUTILS_LOG_INFO("Controller manager: reloaded controller libraries for '%s'", (*it)->getName().c_str());
  }

  resp->ok = true;

  RCUTILS_LOG_DEBUG("reload libraries service finished");
  return;
}


void ControllerManager::listControllerTypesSrv(
  const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request> req,
  std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> resp)
{
  // pretend to use the request
  (void) req;

  // lock services
  RCUTILS_LOG_DEBUG("list types service called");
  boost::mutex::scoped_lock guard(services_lock_);
  RCUTILS_LOG_DEBUG("list types service locked");

  for(std::list<ControllerLoaderInterfaceSharedPtr>::iterator it = controller_loaders_.begin(); it != controller_loaders_.end(); ++it)
  {
    std::vector<std::string> cur_types = (*it)->getDeclaredClasses();
    for(size_t i=0; i < cur_types.size(); i++)
    {
      resp->types.push_back(cur_types[i]);
      resp->base_classes.push_back((*it)->getName());
    }
  }

  RCUTILS_LOG_DEBUG("list types service finished");
}


void ControllerManager::listControllersSrv(
  const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> req,
  std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> resp)
{
  // pretend to use the request
  (void) req;

  // lock services
  RCUTILS_LOG_DEBUG("list controller service called");
  boost::mutex::scoped_lock services_guard(services_lock_);
  RCUTILS_LOG_DEBUG("list controller service locked");

  // lock controllers to get all names/types/states
  boost::recursive_mutex::scoped_lock controller_guard(controllers_lock_);
  std::vector<ControllerSpec> &controllers = controllers_lists_[current_controllers_list_];
  resp->controller.resize(controllers.size());

  for (size_t i = 0; i < controllers.size(); ++i)
  {
    controller_manager_msgs::msg::ControllerState& cs = resp->controller[i];
    cs.name = controllers[i].info.name;
    cs.type = controllers[i].info.type;

    cs.claimed_resources.clear();
    typedef std::vector<hardware_interface::InterfaceResources> ClaimedResVec;
    typedef ClaimedResVec::const_iterator ClaimedResIt;
    const ClaimedResVec& c_res = controllers[i].info.claimed_resources;
    for (ClaimedResIt c_res_it = c_res.begin(); c_res_it != c_res.end(); ++c_res_it)
    {
      controller_manager_msgs::msg::HardwareInterfaceResources iface_res;
      iface_res.hardware_interface = c_res_it->hardware_interface;
      std::copy(c_res_it->resources.begin(), c_res_it->resources.end(), std::back_inserter(iface_res.resources));
      cs.claimed_resources.push_back(iface_res);
    }

    if (controllers[i].c->isRunning())
      cs.state = "running";
    else
      cs.state = "stopped";
  }

  RCUTILS_LOG_DEBUG("list controller service finished");
}


void ControllerManager::loadControllerSrv(
  const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> req,
  std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> resp)
{
  // lock services
  RCUTILS_LOG_DEBUG("loading service called for controller '%s' ",req->name.c_str());
  boost::mutex::scoped_lock guard(services_lock_);
  RCUTILS_LOG_DEBUG("loading service locked");

  resp->ok = loadController(req->name);

  RCUTILS_LOG_DEBUG("loading service finished for controller '%s' ",req->name.c_str());
}


void ControllerManager::unloadControllerSrv(
  const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> req,
  std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> resp)
{
  // lock services
  RCUTILS_LOG_DEBUG("unloading service called for controller '%s' ",req->name.c_str());
  boost::mutex::scoped_lock guard(services_lock_);
  RCUTILS_LOG_DEBUG("unloading service locked");

  resp->ok = unloadController(req->name);

  RCUTILS_LOG_DEBUG("unloading service finished for controller '%s' ",req->name.c_str());
}


void ControllerManager::switchControllerSrv(
  const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> req,
  std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> resp)
{
  // lock services
  RCUTILS_LOG_DEBUG("switching service called");
  boost::mutex::scoped_lock guard(services_lock_);
  RCUTILS_LOG_DEBUG("switching service locked");

  resp->ok = switchController(req->start_controllers, req->stop_controllers, req->strictness);

  RCUTILS_LOG_DEBUG("switching service finished");
}

void ControllerManager::registerControllerLoader(ControllerLoaderInterfaceSharedPtr controller_loader)
{
  controller_loaders_.push_back(controller_loader);
}

}
