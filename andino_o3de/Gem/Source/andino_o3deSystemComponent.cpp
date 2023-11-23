

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "andino_o3deSystemComponent.h"

namespace andino_o3de
{
    void andino_o3deSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<andino_o3deSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<andino_o3deSystemComponent>("andino_o3de", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void andino_o3deSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("andino_o3deService"));
    }

    void andino_o3deSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("andino_o3deService"));
    }

    void andino_o3deSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void andino_o3deSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    andino_o3deSystemComponent::andino_o3deSystemComponent()
    {
        if (andino_o3deInterface::Get() == nullptr)
        {
            andino_o3deInterface::Register(this);
        }
    }

    andino_o3deSystemComponent::~andino_o3deSystemComponent()
    {
        if (andino_o3deInterface::Get() == this)
        {
            andino_o3deInterface::Unregister(this);
        }
    }

    void andino_o3deSystemComponent::Init()
    {
    }

    void andino_o3deSystemComponent::Activate()
    {
        andino_o3deRequestBus::Handler::BusConnect();
    }

    void andino_o3deSystemComponent::Deactivate()
    {
        andino_o3deRequestBus::Handler::BusDisconnect();
    }
}
