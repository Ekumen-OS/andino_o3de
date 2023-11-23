
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <andino_o3deSystemComponent.h>

namespace andino_o3de
{
    /// System component for andino_o3de editor
    class andino_o3deEditorSystemComponent
        : public andino_o3deSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = andino_o3deSystemComponent;
    public:
        AZ_COMPONENT(andino_o3deEditorSystemComponent, "{496683E3-4AD9-4CF0-B082-E3B39AB2BB39}", BaseSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        andino_o3deEditorSystemComponent();
        ~andino_o3deEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace andino_o3de
