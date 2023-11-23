
#include <andino_o3deModuleInterface.h>
#include "andino_o3deEditorSystemComponent.h"

#include "andino_o3deSampleComponent.h"
namespace andino_o3de
{
    class andino_o3deEditorModule
        : public andino_o3deModuleInterface
    {
    public:
        AZ_RTTI(andino_o3deEditorModule, "{4C810620-61C9-4284-BF45-95EA7F10E592}", andino_o3deModuleInterface);
        AZ_CLASS_ALLOCATOR(andino_o3deEditorModule, AZ::SystemAllocator, 0);

        andino_o3deEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                andino_o3deEditorSystemComponent::CreateDescriptor(),
                andino_o3deSampleComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<andino_o3deEditorSystemComponent>(),
            };
        }
    };
}// namespace andino_o3de

AZ_DECLARE_MODULE_CLASS(Gem_andino_o3de, andino_o3de::andino_o3deEditorModule)
