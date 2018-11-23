/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once

//*************************************************************************************************
#include "stdafx.h"
#include "aadc_structs.h"
#include <a_utils/core/a_utils_core.h>

#define CID_NETWORK_SWITCH_FILTER "network_switch_filter.filter.user.aadc.cid"
#define LABEL_NETWORK_SWITCH_FILTER "NetworkSwitch"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cNetworkSwitch : public cFilter
{
public:
    ADTF_CLASS_ID_NAME(cNetworkSwitch, CID_NETWORK_SWITCH_FILTER, LABEL_NETWORK_SWITCH_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;

    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;

    //PinReader
    cPinReader m_oInputDDSpeed;
    cPinReader m_oInputDDSteering;

    cPinReader m_oInputNetSpeed;
    cPinReader m_oInputNetSteering;

    cPinReader m_oInputSwitch;

    //PinWriter
    cPinWriter m_oWriterSpeed;
    cPinWriter m_oWriterSteering;

    //Runner
    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> m_oInputDDSpeedRunner;
    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> m_oInputDDSteeringRunner;
    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> m_oInputNetSpeedRunner;
    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> m_oInputNetSteeringRunner;
    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> m_oInputSwitchRunner;

    tBool m_bUseNetwork = tFalse;

    object_ptr<adtf::services::IReferenceClock> m_pClock;

public:

    /*! Default constructor. */
    cNetworkSwitch();

    /*! Destructor. */
    virtual ~cNetworkSwitch() = default;

    tResult  Init(tInitStage eStage) override;
    tResult  Shutdown(cFilterLevelmachine::tInitStage eStage) override;

    tResult ProcessDDSpeed(tTimeStamp tmTimeOfTrigger);
    tResult ProcessDDSteering(tTimeStamp tmTimeOfTrigger);
    tResult ProcessNetSpeed(tTimeStamp tmTimeOfTrigger);
    tResult ProcessNetSteering(tTimeStamp tmTimeOfTrigger);
    tResult ProcessSwitch(tTimeStamp tmTimeOfTrigger);

    tResult TransmitSpeed(tFloat32 value);
    tResult TransmitSteering(tFloat32 value);
};
