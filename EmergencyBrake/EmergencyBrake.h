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
#define CID_EMERGENCY_BRAKE_DATA_TRIGGERED_FILTER "EmergencyBrake.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cEmergencyBrake : public cTriggerFunction
{
private:
    /*! Media Descriptions. */
    struct tLaserScannerDataId
    {
        tSize ui32SizeId;
        tSize tScanArrayId;
    } m_ddlLaserDataId;

    struct tSignalValueId
    {
        tSize ui32ArduinoTimestamp;
        tSize value;
    } m_ddlSignalValueId;

    struct tBoolSignalValueId
    {
        tSize ui32ArduinoTimestamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;

    /*! The template data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_LaserDataSampleFactory;

    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;

    /*! Reader of an InPin. */
    cPinReader m_oSpeedReader;

    cPinReader m_oLaserReader;

    cPinReader m_oStearingReader;

    cPinReader m_oVPStopReader;

    cPinReader m_oRampReader;

    /*! Writer to an OutPin. */
    cPinWriter m_oSpeedWriter;

    cPinWriter m_oBrakeLightWriter;

    cPinWriter m_oOvertake;

    cPinWriter m_oRampDetector;

    tBool m_ramp = tFalse;

    tBool m_vpStop = tFalse;
    tBool m_initTimeSet;
    tTimeStamp m_initTime;

    tBool m_OvertakeInitTimeSet = tFalse;
    tTimeStamp m_OvertakeInitTime;
    tBool m_OvertakeSent = tFalse;

    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! Properties */
    adtf::base::property_variable<tFloat32> minDistance = 150.0f;
    adtf::base::property_variable<tFloat32> minDistanceExtra = 100.0f;
    adtf::base::property_variable<tFloat32> maxDistance = 500.0f;
    adtf::base::property_variable<tFloat32> minCount = 20.0f;
    adtf::base::property_variable<tFloat32> width = 150.0f;

    adtf::base::property_variable<tInt32> waitTimeUntilOvertake = 2;
    adtf::base::property_variable<tFloat32> m_factorMaxDistance = 0.5f;

public:

    /*! Default constructor. */
    cEmergencyBrake();

    /*! Destructor. */
    virtual ~cEmergencyBrake() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult transmitSpeed(tFloat32 value);
    tResult transmitBrakeLight(tBool bValue);
    tResult transmitOvertake(tBool bValue);
    tResult transmitRamp();

};


//*************************************************************************************************
