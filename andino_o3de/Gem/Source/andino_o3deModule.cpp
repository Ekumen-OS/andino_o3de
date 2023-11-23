
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "andino_o3deSystemComponent.h"
#include "andino_o3deSampleComponent.h"

namespace andino_o3de
{
    class andino_o3deModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(andino_o3deModule, "{4C810620-61C9-4284-BF45-95EA7F10E592}", AZ::Module);
        AZ_CLASS_ALLOCATOR(andino_o3deModule, AZ::SystemAllocator);

        andino_o3deModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                andino_o3deSystemComponent::CreateDescriptor(),
                andino_o3deSampleComponent::CreateDescriptor(),
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

AZ_DECLARE_MODULE_CLASS(Gem_andino_o3de, andino_o3de::andino_o3deModule)
