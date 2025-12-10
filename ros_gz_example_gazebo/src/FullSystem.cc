/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "grasp_plugin/FullSystem.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    grasp_plugin::FullSystem,
    gz::sim::System,
    grasp_plugin::FullSystem::ISystemConfigure,
    grasp_plugin::FullSystem::ISystemPreUpdate,
    grasp_plugin::FullSystem::ISystemUpdate,
    grasp_plugin::FullSystem::ISystemPostUpdate
)

namespace grasp_plugin 
{

void FullSystem::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{
  igndbg << "grasp_plugin::FullSystem::Configure on entity: " << _entity << std::endl;
}

void FullSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  

  // // print location of box2
  // _ecm.Each<gz::sim::components::Name, gz::sim::components::Pose>(
  //   [](const gz::sim::Entity &/*_entity*/,
  //      const gz::sim::components::Name *_name,
  //      const gz::sim::components::Pose *_pose) -> bool
  //   {
  //     if (_name->Data() == "box2")
  //     {
  //       igndbg << "box2 position: " << _pose->Data().Pos() << std::endl;
  //     }
  //     return true;
  //   });
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
  
    // move box1 to box2 position
  gz::sim::Entity box1Entity = _ecm.EntityByComponents(
    gz::sim::components::Name("box1"));
  gz::sim::Entity box2Entity = _ecm.EntityByComponents(
    gz::sim::components::Name("box2"));
  if (box1Entity != gz::sim::kNullEntity &&
      box2Entity != gz::sim::kNullEntity)
  {
    auto box2PoseComp = _ecm.Component<gz::sim::components::Pose>(box2Entity);
    if (box2PoseComp)
    {
      gz::math::Pose3d box2Pose = box2PoseComp->Data();
      auto box1PoseComp = _ecm.Component<gz::sim::components::Pose>(box1Entity);
      if (box1PoseComp)
      {
        box1PoseComp->Data().Set(box2Pose.Pos() + gz::math::Vector3d(0, 0, 1), box1PoseComp->Data().Rot());
        std::cout << "Child link pose set to parent link pose plus offsethsfdsfdabhkjfhkas." << std::endl;
        std::cout << "Parent link pose: " << box2Pose << std::endl;
        std::cout << "Child link new pose: " << box1PoseComp->Data() << std::endl;
         _ecm.SetChanged(box1Entity, gz::sim::components::Pose::typeId,
        gz::sim::ComponentState::OneTimeChange);
      }
    }
  }


    std::cout << "grasp_plugin::FullSystem::PreUpdate" << std::endl;
  }
}

void FullSystem::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
   

    std::cout << "grasp_plugin::FullSystem::Update" << std::endl;
  }
}

void FullSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) 
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
   
    std::cout << "grasp_plugin::FullSystem::PostUpdate" << std::endl;
  }
}

}  // namespace grasp_plugin
