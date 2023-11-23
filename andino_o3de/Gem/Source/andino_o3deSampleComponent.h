#pragma once

#include <AzCore/std/containers/vector.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <ImGuiBus.h>

#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <ROS2/Communication/TopicConfiguration.h>

#include <rclcpp/publisher.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace andino_o3de
{

    class andino_o3deSampleComponent
        : public AZ::Component
        , public ImGui::ImGuiUpdateListenerBus::Handler
    {
    public:
        AZ_COMPONENT(andino_o3deSampleComponent, "{26349EB7-DA67-4587-882B-C12B6F1DC272}", AZ::Component);
        andino_o3deSampleComponent();
        ~andino_o3deSampleComponent() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        void OnImGuiUpdate() override;

        AZStd::vector<AZ::EntityId> m_goalEntities;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> m_goalPublisher;
        geometry_msgs::msg::PoseStamped m_goalMessage;
        ROS2::TopicConfiguration m_goalTopicConfiguration;
    };
} // namespace andino_o3de
