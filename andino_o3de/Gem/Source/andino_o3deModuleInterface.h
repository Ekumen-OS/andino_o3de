
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <andino_o3deSystemComponent.h>

namespace andino_o3de
{
    class andino_o3deModuleInterface
        : public AZ::Module
    {
    public:
        AZ_RTTI(andino_o3deModuleInterface, "{527A5C6D-923D-4C37-BFA1-6898BBE2722E}", AZ::Module);
        AZ_CLASS_ALLOCATOR(andino_o3deModuleInterface, AZ::SystemAllocator, 0);

        andino_o3deModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                andino_o3deSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<andino_o3deSystemComponent>(),
            };
        }
    };
}// namespace andino_o3de
