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
#include "stdafx.h"
#include "SpeedScaler.h"
#include "aadc_structs.h"
#include "ADTF3_helper.h"

#define CLAMP(X, MIN, MAX) ((X < MIN) ? MIN : ((X > MAX) ? MAX : X))

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_SPEED_SCALER_DATA_TRIGGERED_FILTER,"SpeedScaler",cSpeedScaler,adtf::filter::pin_trigger({ "SpeedIn", "Steering", "RoadSignExt", "WheelData", "ToggleOnOff" }));

cSpeedScaler::cSpeedScaler()
{
	object_ptr<IStreamType> pTypeSignalValue;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_oSignalValueSampleCodecFactory))
	{
		adtf_ddl::access_element::find_index(m_oSignalValueSampleCodecFactory, cString("f32Value"), m_ddlSignalValueId.f32Value);
	}
	else
	{
		LOG_WARNING("No mediadescription for tSignalValue found!");
	}

	object_ptr<IStreamType> pTypeRoadSignExt;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tRoadSignExt", pTypeRoadSignExt, m_oRoadSignExtSampleCodecFactory))
	{
		adtf_ddl::access_element::find_array_index(m_oRoadSignExtSampleCodecFactory, cString("af32TVec"), m_ddlRoadSignExtId.tVec);
		adtf_ddl::access_element::find_array_index(m_oRoadSignExtSampleCodecFactory, cString("af32RVec"), m_ddlRoadSignExtId.rVec);
		adtf_ddl::access_element::find_index(m_oRoadSignExtSampleCodecFactory, cString("i16Identifier"), m_ddlRoadSignExtId.identifier);
	}
	else
	{
		LOG_WARNING("No mediadescription for tRoadSignExt found!");
	}

	object_ptr<IStreamType> pTypeWheelData;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData, m_oWheelDataSampleCodecFactory))
	{
		adtf_ddl::access_element::find_index(m_oWheelDataSampleCodecFactory, "ui32ArduinoTimestamp", m_ddlWheelDataId.ArduinoTimestamp);
		adtf_ddl::access_element::find_index(m_oWheelDataSampleCodecFactory, "ui32WheelTach", m_ddlWheelDataId.WheelTach);
		adtf_ddl::access_element::find_index(m_oWheelDataSampleCodecFactory, "i8WheelDir", m_ddlWheelDataId.WheelDir);
	}
	else
	{
		LOG_INFO("No mediadescription for tWheelData found!");
	}

	object_ptr<IStreamType> pTypeBoolSignalValue;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_oBoolSignalValueSampleCodecFactory))
	{
		adtf_ddl::access_element::find_index(m_oBoolSignalValueSampleCodecFactory, "bValue", m_ddlBoolSignalValueId.bValue);
	}
	else
	{
		LOG_INFO("No mediadescription for tBoolSignalValue found!");
	}

    Register(m_oSpeedIn, "SpeedIn", pTypeSignalValue);
	Register(m_oSteeringIn, "Steering", pTypeSignalValue);
	Register(m_oRoadSignExtIn, "RoadSignExt", pTypeRoadSignExt);
	Register(m_oWheelDataIn, "WheelData", pTypeWheelData);
	Register(m_oToggleOnOff, "ToggleOnOff", pTypeBoolSignalValue);

    Register(m_oSpeedOut, "SpeedOut", pTypeSignalValue);

    RegisterPropertyVariable("Max Multiplier Straight", m_maxMultiplierStraight);
    RegisterPropertyVariable("Max Multiplier Curve", m_maxMultiplierCurve);
    RegisterPropertyVariable("Acceleration Offset", m_accelerationOffset);
    RegisterPropertyVariable("Acceleration Distance", m_accelerationDist);
    RegisterPropertyVariable("Deceleration Distance", m_decelerationDist);
	RegisterPropertyVariable("Deceleration Offset", m_decelerationOffset);
	RegisterPropertyVariable("Steering Offset", m_steeringOffset);
    RegisterPropertyVariable("Steering Deadband", m_steeringDeadband);
}

tResult cSpeedScaler::Configure()
{
	RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
	RETURN_NOERROR;
}

tResult cSpeedScaler::Process(tTimeStamp tmTimeOfTrigger)
{
    ProcessWheel();
    ProcessToggleOnOff();
    ProcessSteering();
    ProcessRoadSign();

	object_ptr<const ISample> pSpeedSample;
	if (IS_OK(m_oSpeedIn.GetNextSample(pSpeedSample)))
	{
        if (m_bActive && m_dist > m_accelerationOffset)
        {
            auto oDecoder = m_oSignalValueSampleCodecFactory.MakeDecoderFor(*pSpeedSample);
            RETURN_IF_FAILED(oDecoder.IsValid());

            tFloat32 speed;
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.f32Value, &speed));

            if (speed > 0)
            {
                if (m_bSteering)
                {
                    speed *= m_maxMultiplierCurve;
                }
                else
                {
                    speed *= m_maxMultiplierStraight;
                }

                object_ptr<ISample> pNewSpeedSample;
                if (IS_OK(alloc_sample(pNewSpeedSample)))
                {
                    auto oCodec = m_oSignalValueSampleCodecFactory.MakeCodecFor(pNewSpeedSample);
                    RETURN_IF_FAILED(oCodec.IsValid());
                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.f32Value, speed));
                    m_oSpeedOut << pNewSpeedSample << flush;
                }
            }
            else
            {
                m_oSpeedOut << pSpeedSample << flush;
            }
		}
		else
		{
			m_oSpeedOut << pSpeedSample << flush;
		}
	}
	RETURN_NOERROR;
}

tResult cSpeedScaler::ProcessToggleOnOff()
{
    object_ptr<const ISample> pOnOffSample;
    if (IS_OK(m_oToggleOnOff.GetNextSample(pOnOffSample)))
    {
        auto oDecoder = m_oBoolSignalValueSampleCodecFactory.MakeDecoderFor(*pOnOffSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &m_bActive));
        if (m_bActive)
        {
            m_dist = 0;
            m_tach = 0;
        }
    }
    RETURN_NOERROR;
}

tResult cSpeedScaler::ProcessWheel()
{
    object_ptr<const ISample> pWheelSample;
    if (IS_OK(m_oWheelDataIn.GetNextSample(pWheelSample)))
    {
        auto oDecoder = m_oWheelDataSampleCodecFactory.MakeDecoderFor(*pWheelSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        tUInt32 wheelTach;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataId.WheelTach, &wheelTach));

        // only init m_tach with first sample
        if (m_tach != 0)
        {
            m_dist += ((wheelTach - m_tach) / 60.f) * 0.34f;
        }
        m_tach = wheelTach;
    }
    RETURN_NOERROR;
}

tResult cSpeedScaler::ProcessSteering()
{
    object_ptr<const ISample> pSteeringSample;
    if (IS_OK(m_oSteeringIn.GetNextSample(pSteeringSample)))
    {
        auto oDecoder = m_oSignalValueSampleCodecFactory.MakeDecoderFor(*pSteeringSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        tFloat32 steering;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.f32Value, &steering));
        m_bSteering = std::abs(steering - m_steeringOffset) > m_steeringDeadband;
    }
    RETURN_NOERROR;
}

tResult cSpeedScaler::ProcessRoadSign()
{
    object_ptr<const ISample> pRoadSignExtSample;
    if (IS_OK(m_oRoadSignExtIn.GetNextSample(pRoadSignExtSample)))
    {
        auto oDecoder = m_oRoadSignExtSampleCodecFactory.MakeDecoderFor(*pRoadSignExtSample);
        RETURN_IF_FAILED(oDecoder.IsValid());

        tInt16 roadSignId;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlRoadSignExtId.identifier, &roadSignId));

        // ignore aadc helper signs
        if (std::find(m_regularRoadSignIds.begin(), m_regularRoadSignIds.end(), roadSignId) == m_regularRoadSignIds.end())
        {
            RETURN_NOERROR;
        }

        const tFloat32* tVec = static_cast<const tFloat32 *>(oDecoder.GetElementAddress(m_ddlRoadSignExtId.tVec));
        const tFloat32* rVec = static_cast<const tFloat32 *>(oDecoder.GetElementAddress(m_ddlRoadSignExtId.rVec));
        if (tVec == nullptr || rVec == nullptr)
        {
            return ERR_INVALID_INDEX;
        }

        tFloat32 distance = std::sqrt(tVec[0] * tVec[0] + (tVec[2] - 0.2f)*(tVec[2] - 0.2f));
        tFloat32 orientation = rVec[2];
        if (std::abs(orientation) < 1.f && distance < m_decelerationOffset)
        {
            m_dist = 0;
        }
    }
    RETURN_NOERROR;
}
