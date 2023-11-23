/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace andino_o3de
{
    class andino_o3deRequests
    {
    public:
        AZ_RTTI(andino_o3deRequests, "{B1DD0BFD-5EC2-47D5-9BEE-C2CACF60C5DD}");
        virtual ~andino_o3deRequests() = default;
        // Put your public methods here
    };

    class andino_o3deBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using andino_o3deRequestBus = AZ::EBus<andino_o3deRequests, andino_o3deBusTraits>;
    using andino_o3deInterface = AZ::Interface<andino_o3deRequests>;

} // namespace andino_o3de
