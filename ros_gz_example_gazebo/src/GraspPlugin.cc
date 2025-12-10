#include <vector>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/common/Profiler.hh>

#include <sdf/Element.hh>

#include "ignition/gazebo/components/DetachableJoint.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/PoseCmd.hh"

#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Model.hh"

#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include <string>
#include <iostream>

#include "grasp_plugin/GraspPlugin.hh"
#include <ignition/gazebo/System.hh>

using namespace grasp_plugin;

// using namespace ignition;
// using namespace gazebo;
// using namespace systems;

////////////////////////////////////////////////

/////////////////////////////////////////////////
void GraspJoint::Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager & /*_eventMgr*/)
{
  std::cout << "Configuring GraspPlugin Plugin" << std::endl;
  /// Topics
  if (_sdf->HasElement("attachtopic"))
  {
    this->attachtopic = _sdf->Get<std::string>("attachtopic");
    std::cout << "'attachtopic' set to [" << this->attachtopic << "]" << std::endl;
  }
  else
  {
    this->attachtopic = "GraspPlugin";
    std::cout << "'attachtopic' is 'GraspPlugin' by default." << std::endl;
  }
  if (_sdf->HasElement("offset_x"))
  {
    this->offset_x = _sdf->Get<double>("offset_x");
    std::cout << "'offset_x' set to [" << this->offset_x << "]" << std::endl;
  }
  else
  {
    this->offset_x = 0.0;
    std::cout << "'offset_x' is 0.0 by default." << std::endl;
  }
  if (_sdf->HasElement("offset_y"))
  {
    this->offset_y = _sdf->Get<double>("offset_y");
    std::cout << "'offset_y' set to [" << this->offset_y << "]" << std::endl;
  }
  else
  {
    this->offset_y = 0.0;
    std::cout << "'offset_y' is 0.0 by default." << std::endl;
  }
  if (_sdf->HasElement("offset_z"))
  {
    this->offset_z = _sdf->Get<double>("offset_z");
    std::cout << "'offset_z' set to [" << this->offset_z << "]" << std::endl;
  }
  else
  {
    this->offset_z = 0.0;
    std::cout << "'offset_z' is 0.0 by default." << std::endl;
  }

  this->suppressChildWarning =
      _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning)
          .first;

  this->suppressParentWarning =
      _sdf->Get<bool>("suppress_parent_warning", this->suppressParentWarning)
          .first;
  this->validConfig = true;
}

bool GraspJoint::MoveChildToParent(ignition::gazebo::EntityComponentManager &_ecm)
{
  // move child link to parent link position plus offset
  // check if parent and child are close enough to attach
  auto pmodelEntity = _ecm.EntityByComponents(ignition::gazebo::components::Model(), ignition::gazebo::components::Name(this->parentModelName));
  auto cmodelEntity = _ecm.EntityByComponents(ignition::gazebo::components::Model(), ignition::gazebo::components::Name(this->childModelName));
  if (ignition::gazebo::kNullEntity == pmodelEntity || ignition::gazebo::kNullEntity == cmodelEntity)
  {
    std::cout << "Parent or child model not found, cannot process attach request." << std::endl;
    this->attachRequested = false;
    return false;
  }
  auto parentPoseComp = _ecm.Component<ignition::gazebo::components::Pose>(pmodelEntity);
  auto childPoseComp =_ecm.Component<ignition::gazebo::components::Pose>(cmodelEntity);
  if (!parentPoseComp || !childPoseComp)
  {
    std::cout << "Parent or child model pose component not found, cannot process attach request." << std::endl;
    this->attachRequested = false;
    return false;
  }

  // move child link to parent link position plus offset
 
    gz::math::Pose3d parentPose3d = parentPoseComp->Data();
    // auto box1PoseComp = _ecm.Component<gz::sim::components::Pose>(cmodelEntity);
    if (childPoseComp)
    {
      auto poseCmdComp =
          _ecm.Component<gz::sim::components::WorldPoseCmd>(cmodelEntity);
      if (!poseCmdComp)
      {
        std::cout << "Creating WorldPoseCmd component for child link entity." << std::endl;
        poseCmdComp =
            _ecm.CreateComponent(
                cmodelEntity, gz::sim::components::WorldPoseCmd(parentPose3d));
      }
      // gz::math::Pose3d newPos = box2Pose;
      // std::cout << "Current child link pose: " << childPoseComp->Data() << std::endl;
      parentPose3d.Set(parentPose3d.Pos() + gz::math::Vector3d(this->offset_x, this->offset_y, this->offset_z), parentPose3d.Rot());
      std::cout << "Setting child link pose to: " << parentPose3d << std::endl;
      std::cout << "addr parentPose3d: " << &parentPose3d << std::endl;
      auto state = poseCmdComp->SetData(parentPose3d, [](const gz::math::Pose3d &,
                                                     const gz::math::Pose3d &)
                                        { return true; });
      std::cout << "SetData returned state: " << static_cast<int>(state) << std::endl;
      _ecm.SetChanged(cmodelEntity, gz::sim::components::WorldPoseCmd::typeId,
                      gz::sim::ComponentState::OneTimeChange);
        
      std::cout << "Child link pose set to parent link pose plus offset." << std::endl;
      std::cout << "Parent link pose: " << parentPose3d << std::endl;

      this->need_move = false;
      this->move_timeout = true;
      return true;
    }
    else
    {
      std::cout << "No pose component found for child link entity." << std::endl;
    }

  return false;
}

//////////////////////////////////////////////////
void GraspJoint::PreUpdate(
    const ignition::gazebo::UpdateInfo & /*_info*/,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // ignmsg << "loop"<< std::endl;
  IGN_PROFILE("GraspJoint::PreUpdate");
  ignition::msgs::Int32 msg;
  msg.set_data(-1);

  if (this->not_initialized)
  {
    auto out = this->node.Subscribe(
        this->attachtopic, &GraspJoint::OnAttachRequest, this);
    std::cout << "GraspJoint subscribing to messages on "
              << "[" << this->attachtopic << "]" << std::endl;
    std::cout << "out: " << out << std::endl;
    this->not_initialized = false;

    ///////////////

    this->error_topic.reset();
    this->error_topic = this->node.Advertise<ignition::msgs::Int32>("GraspJoint/error");

    ///////////////
  }

  if (this->validConfig && this->attachRequested)
  {

    std::cout << "Processing grasp attach request." << std::endl;
    bool createNewGraspJoint = true;
    if(this->move_timeout)
    {
      std::cout << "Move timeout occurred, skipping attach this cycle." << std::endl;
      this->move_timeout = false;
      return;
    }
    if(this->need_move)
    {
    if (!this->MoveChildToParent(_ecm)) {
      this->attachRequested = false;
      msg.set_data(10);
      this->error_topic->Publish(msg);
    }
    // when moved, wait for next loop to create joint
    return;
  }
    // return;
    for (auto &item : this->GraspJointList)
    {
      if (item.second == this->GraspJointName)
      {
        createNewGraspJoint = false;
        break;
      }
    }
    if (createNewGraspJoint == true)
    {
      std::cout << "Creating new attachable joint." << std::endl;

      ignition::gazebo::Entity pmodelEntity{ignition::gazebo::kNullEntity};

      pmodelEntity = _ecm.EntityByComponents(ignition::gazebo::components::Model(), ignition::gazebo::components::Name(this->parentModelName));

      if (ignition::gazebo::kNullEntity != pmodelEntity)
      {
        this->parentLinkEntity = _ecm.EntityByComponents(
            ignition::gazebo::components::Link(), ignition::gazebo::components::ParentEntity(pmodelEntity),
            ignition::gazebo::components::Name(this->parentLinkName));

        if (ignition::gazebo::kNullEntity != this->parentLinkEntity)
        {
          std::cout << "Parent Link " << this->parentLinkName << " found." << std::endl;
          // Hacemos todo con el hijo
          ignition::gazebo::Entity cmodelEntity{ignition::gazebo::kNullEntity};

          cmodelEntity = _ecm.EntityByComponents(ignition::gazebo::components::Model(), ignition::gazebo::components::Name(this->childModelName));
          if (ignition::gazebo::kNullEntity != cmodelEntity)
          {
            this->childLinkEntity = _ecm.EntityByComponents(
                ignition::gazebo::components::Link(), ignition::gazebo::components::ParentEntity(cmodelEntity),
                ignition::gazebo::components::Name(this->childLinkName));

            if (ignition::gazebo::kNullEntity != this->childLinkEntity)
            {
              std::cout << "Child Link " << this->childLinkName << " found." << std::endl;
              // grasp the models, by moving the child link to the parent link position

              this->GraspJointEntity = _ecm.CreateEntity();
              auto out = _ecm.CreateComponent(
                  this->GraspJointEntity,
                  ignition::gazebo::components::DetachableJoint({this->parentLinkEntity,
                                                                 this->childLinkEntity, "fixed"}));
              std::cout << "CreateComponent returned: " << out << std::endl;
              _ecm.SetChanged(this->GraspJointEntity, ignition::gazebo::components::DetachableJoint::typeId,
                              ignition::gazebo::ComponentState::OneTimeChange);
              this->GraspJointList.push_back({this->GraspJointEntity, this->GraspJointName});
              this->initialized = true;
              this->attachRequested = false;
              msg.set_data(0);
            }
            else
            {
              this->attachRequested = false;
              std::cout << "Child Link " << this->childLinkName
                        << " could not be found." << std::endl;
              msg.set_data(1);
            }
          }
          else if (!this->suppressChildWarning)
          {
            this->attachRequested = false;
            std::cout << "Child Model " << this->childModelName
                      << " could not be found." << std::endl;
            msg.set_data(1);
          }
          // Hacemos todo con el hijo
        }
        else
        {
          this->attachRequested = false;
          std::cout << "Parent Link " << this->parentLinkName
                    << " could not be found." << std::endl;
          msg.set_data(1);
        }
      }
      else if (!this->suppressParentWarning)
      {
        std::cout << "Parent Model " << this->parentModelName
                  << " could not be found." << std::endl; // ignwarnignerr
        this->attachRequested = false;
        msg.set_data(1);
      }
    }
    else
    {
      std::cout << "Successful attach request, but the GraspJoint '"
                << this->GraspJointName << "' already exists." << std::endl;
      this->attachRequested = false;
      msg.set_data(2);
    }
    std::cout << "Attach request processed." << msg.data() << std::endl;
  }
  if (this->initialized)
  {
    if (this->detachRequested)
    {
      // Detach the models
      int i;
      msg.set_data(1);
      for (i = 0; i <= this->GraspJointList.size(); i++)
      {
        if (this->GraspJointList[i].second == this->GraspJointName)
        {
          msg.set_data(0);
          std::cout << "Removing entity: " << this->GraspJointList[i].first << std::endl;
          _ecm.RequestRemoveEntity(this->GraspJointList[i].first);

          this->GraspJointList.erase(this->GraspJointList.begin() + i);
          this->detachRequested = false;
          break;
        }
      }
    }
  }
  if (msg.data() > -1 && this->error_topic)
  {
    this->error_topic->Publish(msg);
  }
}

//////////////////////////////////////////////////
void GraspJoint::OnAttachRequest(const ignition::msgs::StringMsg &msg)
{
  std::cout << "Received attach request message." << std::endl;
  std::cout << "The message sent is: " << msg.data() << std::endl;
  // ignmsg << "El mensaje enviado es: " << msg.data() << std::endl;

  // [parentModel][ParentLink][ChildModel][ChildLink]
  // Now the Link must be nammed AttachableLink_Name or wont work

  std::string str = msg.data(); //"[box1][box_body][box2][box_body]";
  std::string request;
  std::cout << "debug:: line " << __LINE__ << std::endl;
  unsigned first = str.find('[');
  str = &str[first];
  unsigned last = str.find(']');
  this->parentModelName = str.substr(1, last - 1);
  std::cout << "parent model name: " << this->parentModelName << std::endl;
  str = &str[last];
  std::cout << "debug:: line " << __LINE__ << std::endl;
  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  this->parentLinkName = str.substr(1, last - 1);
  std::cout << "parent link name: " << this->parentLinkName << std::endl;
  str = &str[last];
  std::cout << "debug:: line " << __LINE__ << std::endl;
  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  this->childModelName = str.substr(1, last - 1);
  std::cout << "child model name: " << this->childModelName << std::endl;
  str = &str[last];
  std::cout << "debug:: line " << __LINE__ << std::endl;
  first = str.find('[');
  str = &str[first];
  last = str.find(']');
  this->childLinkName = str.substr(1, last - 1);
  std::cout << "child link name: " << this->childLinkName << std::endl;
  str = &str[last];
  std::cout << "debug:: line " << __LINE__ << std::endl;
  first = str.find('[');
  std::cout << "debug:: line " << __LINE__ << std::endl;
  std::cout << "first: " << first << std::endl;
  std::cout << "str before: " << str << std::endl;
  str = &str[first];
  std::cout << "debug:: line " << __LINE__ << std::endl;
  last = str.find(']');
  std::cout << "debug:: line " << __LINE__ << std::endl;
  std::cout << "debug:: line " << __LINE__ << std::endl;
  request = str.substr(1, last - 1);
  std::cout << "Request is: " << request << std::endl;
  std::cout << "Parent Model: " << this->parentModelName << std::endl;
  std::cout << "Parent Link: " << this->parentLinkName << std::endl;
  std::cout << "Child Model: " << this->childModelName << std::endl;
  std::cout << "Child Link: " << this->childLinkName << std::endl;
  std::cout << "debug:: line " << __LINE__ << std::endl;
  this->GraspJointName = this->parentModelName + "_" + this->parentLinkName + "_" + this->childModelName + "_" + this->childLinkName;

  if ("attach" == request)
  {
    this->attachRequested = true;
    this->need_move = true;
    this->move_timeout = false;
    std::cout << "PM: " << this->parentModelName << " PL: " << this->parentLinkName << " CM: " << this->childModelName << " CL: " << this->childLinkName
              << std::endl
              << "\n\n\n";
  }
  else if ("detach" == request)
  {
    if (false == this->not_initialized)
    {
      this->detachRequested = true;

      ignmsg << "PM: " << this->parentModelName << " PL: " << this->parentLinkName << " CM: " << this->childModelName << " CL: " << this->childLinkName
             << std::endl
             << "\n\n\n";
    }
    else
    {
      std::cout << "There is no GraspJoint created yet" << std::endl;
    }
  }

  /*
  if ( (this->parentLinkName.find("AttachableLink") != -1) && (this->childLinkName.find("AttachableLink") != -1) ) {

    this->attachRequested = true;
    ignmsg << "PM: " <<this->parentModelName <<" PL: "<< this->parentLinkName <<" CM: "<< this->childModelName <<" CL: "<< this->childLinkName
           << std::endl;
  }
  else {
      ignerr << "parent link or child link are not AttachableLinks"<< std::endl;
  }
  */
}

IGNITION_ADD_PLUGIN(grasp_plugin::GraspJoint,
                    ignition::gazebo::System,
                    grasp_plugin::GraspJoint::ISystemConfigure,
                    grasp_plugin::GraspJoint::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(GraspJoint, "grasp_plugin::AtachableJoint")
