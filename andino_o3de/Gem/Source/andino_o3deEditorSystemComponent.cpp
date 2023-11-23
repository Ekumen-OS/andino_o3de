
#include <AzCore/Serialization/SerializeContext.h>
#include "andino_o3deEditorSystemComponent.h"

namespace andino_o3de
{
    void andino_o3deEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<andino_o3deEditorSystemComponent, andino_o3deSystemComponent>()
                ->Version(0);
        }
    }

    andino_o3deEditorSystemComponent::andino_o3deEditorSystemComponent() = default;

    andino_o3deEditorSystemComponent::~andino_o3deEditorSystemComponent() = default;

    void andino_o3deEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("andino_o3deEditorService"));
    }

    void andino_o3deEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("andino_o3deEditorService"));
    }

    void andino_o3deEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void andino_o3deEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void andino_o3deEditorSystemComponent::Activate()
    {
        andino_o3deSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void andino_o3deEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        andino_o3deSystemComponent::Deactivate();
    }

} // namespace andino_o3de
