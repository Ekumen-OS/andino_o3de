/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>

#include <andino_o3de/andino_o3deBus.h>

namespace andino_o3de
{
    class andino_o3deSystemComponent
        : public AZ::Component
        , protected andino_o3deRequestBus::Handler
    {
    public:
        AZ_COMPONENT(andino_o3deSystemComponent, "{6222B8A6-BF9B-4495-95AA-913814DCDF8C}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        andino_o3deSystemComponent();
        ~andino_o3deSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // andino_o3deRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
