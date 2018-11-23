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
#define CID_SPEED_SCALER_DATA_TRIGGERED_FILTER "speed_scaler.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cSpeedScaler : public cTriggerFunction
{
private:
    cPinReader m_oSpeedIn;
	cPinReader m_oSteeringIn;
	cPinReader m_oRoadSignExtIn;
	cPinReader m_oWheelDataIn;
	cPinReader m_oToggleOnOff;

    cPinWriter m_oSpeedOut;

	cSampleCodecFactory m_oSignalValueSampleCodecFactory;
	cSampleCodecFactory m_oRoadSignExtSampleCodecFactory;
	cSampleCodecFactory m_oWheelDataSampleCodecFactory;
	cSampleCodecFactory m_oBoolSignalValueSampleCodecFactory;

	struct
	{
		tSize ts;
		tSize f32Value;
	} m_ddlSignalValueId;

	struct
	{
		tSize identifier;
		tSize tVec;
		tSize rVec;
	} m_ddlRoadSignExtId;

	struct
	{
		tSize ArduinoTimestamp;
		tSize WheelTach;
		tSize WheelDir;
	} m_ddlWheelDataId;

	struct
	{
		tSize bValue;
	} m_ddlBoolSignalValueId;

	tFloat32 m_dist = 0.f;
	tFloat32 m_multiplier = 1.f;
    tUInt32 m_tach = 0;
    tBool m_bSteering = tFalse;
	tBool m_bActive = tTrue;

    std::vector<tInt16> m_regularRoadSignIds = { 0, 1, 2, 3, 4, 5, 6 };

	property_variable<tFloat32> m_maxMultiplierStraight = 2.f;
	property_variable<tFloat32> m_maxMultiplierCurve = 1.5f;
	property_variable<tFloat32> m_accelerationOffset = 0.5f;
	property_variable<tFloat32> m_accelerationDist = 0.5f;
	property_variable<tFloat32> m_decelerationDist = .5f;
	property_variable<tFloat32> m_decelerationOffset = .5f;
	property_variable<tFloat32> m_steeringOffset = 0.f;
	property_variable<tFloat32> m_steeringDeadband = 10.f;

	object_ptr<adtf::services::IReferenceClock> m_pClock;

public:
    cSpeedScaler();
    virtual ~cSpeedScaler() = default;

	tResult Configure() override;
	tResult Process(tTimeStamp tmTimeOfTrigger) override;
    tResult ProcessToggleOnOff();
    tResult ProcessWheel();
    tResult ProcessSteering();
    tResult ProcessRoadSign();
};


//*************************************************************************************************
