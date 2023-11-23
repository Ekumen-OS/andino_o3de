
#include "andino_o3deSampleComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <imgui/imgui.h>

#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace andino_o3de
{

    andino_o3deSampleComponent::andino_o3deSampleComponent()
    {
        m_goalTopicConfiguration.m_topic = "/goal_pose";
        m_goalTopicConfiguration.m_type = "geometry_msgs::msg::PoseStamped";
    }

    void andino_o3deSampleComponent::Activate()
    {
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        m_goalPublisher = ros2Node->create_publisher<geometry_msgs::msg::PoseStamped>(m_goalTopicConfiguration.m_topic.data(), m_goalTopicConfiguration.GetQoS());
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();

    }

    void andino_o3deSampleComponent::Deactivate()
    {
        m_goalPublisher.reset();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void andino_o3deSampleComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<andino_o3deSampleComponent, AZ::Component>()
                ->Version(1)
                ->Field("goals", &andino_o3deSampleComponent::m_goalEntities)
                ->Field("goalMessageTopic", &andino_o3deSampleComponent::m_goalTopicConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<andino_o3deSampleComponent>("andino_o3deSampleComponent", "A sample component that sends goal")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "andino_o3deSampleComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Sample ROS2 Project")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &andino_o3deSampleComponent::m_goalEntities, "Goals", "Entities goals")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &andino_o3deSampleComponent::m_goalTopicConfiguration,
                        "Topic for goal message",
                        "Configuration for ROS2 topic to send goal message to");
            }
        }
    }

    void andino_o3deSampleComponent::OnImGuiUpdate()
    {
        ImGui::Begin("andino_o3deSampleComponent");
        for (const auto& entityId : m_goalEntities)
        {
            AZStd::string entityName;
            AZ::ComponentApplicationBus::BroadcastResult(entityName, &AZ::ComponentApplicationRequests::GetEntityName, entityId);
            const AZStd::string buttonName = AZStd::string::format("Send goal %s", entityName.c_str());

            if (ImGui::Button(buttonName.c_str()))
            {
                AZ::Transform transform;
                AZ::TransformBus::EventResult(transform, entityId, &AZ::TransformBus::Events::GetWorldTM);
                geometry_msgs::msg::PoseStamped poseStamped;
                poseStamped.header.frame_id = "odom";
                poseStamped.pose = ROS2::ROS2Conversions::ToROS2Pose(transform);
                m_goalPublisher->publish(poseStamped);
            }
        }
        ImGui::End();
    }
} // namespace andino_o3de