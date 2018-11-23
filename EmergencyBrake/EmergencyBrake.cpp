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
#include "EmergencyBrake.h"
#include "aadc_structs.h"
#include <cmath>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_EMERGENCY_BRAKE_DATA_TRIGGERED_FILTER,
                                    "EmergencyBrakeFilter",
                                    cEmergencyBrake,
                                    adtf::filter::pin_trigger({"laser"}));


cEmergencyBrake::cEmergencyBrake()
{
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeLaserData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pTypeLaserData, m_LaserDataSampleFactory))
    {
        adtf_ddl::access_element::find_array_index(m_LaserDataSampleFactory, cString("tScanArray"), m_ddlLaserDataId.tScanArrayId);
        adtf_ddl::access_element::find_index(m_LaserDataSampleFactory, cString("ui32Size"), m_ddlLaserDataId.ui32SizeId);
    }
    else
    {
        LOG_WARNING("No mediadescription for tLaserScanData found!");
    }

    Register(m_oLaserReader, "laser" , pTypeLaserData);

    object_ptr<IStreamType> pType;
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pType, m_SignalValueSampleFactory)))
    {
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.ui32ArduinoTimestamp);
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    object_ptr<IStreamType> pTypeBool;
    if( IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBool, m_BoolSignalValueSampleFactory)))
    {
        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory,   cString("bValue"), m_ddlBoolSignalValueId.bValue);
    }
    else
    {
        LOG_WARNING("No mediadescription for tBoolSignalValue found!");
    }

    Register(m_oSpeedReader, "speedIn", pType);
    Register(m_oSpeedWriter, "speedOut", pType);
    Register(m_oStearingReader, "steering", pType);

    Register(m_oVPStopReader, "VPStopBool", pTypeBool);
    Register(m_oBrakeLightWriter, "brakeLightToDD", pTypeBool);
    Register(m_oOvertake, "overtake", pTypeBool);
    Register(m_oRampReader, "ramp_switch", pTypeBool);
    Register(m_oRampDetector, "ramp_detector", pTypeBool);

    //Register Properties

    RegisterPropertyVariable("Min Distance of Stop Area", minDistance);
    RegisterPropertyVariable("Max Distance of Stop Area", maxDistance);
    RegisterPropertyVariable("Width of Detection Cone in [0,250]", width);
    RegisterPropertyVariable("min number of Laserpoints to trigger", minCount);
    RegisterPropertyVariable("Extra length added to Min Distance to the front", minDistanceExtra);
    RegisterPropertyVariable("Multiplication Factor for maxDistance increased by speed", m_factorMaxDistance);
}


//implement the Configure function to read ALL Properties
tResult cEmergencyBrake::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cEmergencyBrake::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSampleBool;

    if(IS_OK(m_oVPStopReader.GetNextSample(pReadSampleBool)))
    {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pReadSampleBool);

        tBool stop = false;
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &stop));

        if(stop && !m_vpStop)
        {
            m_initTimeSet = tFalse;
        }
        m_vpStop = stop;
    }

    object_ptr<const ISample> pReadSampleRamp;
    if(IS_OK(m_oRampReader.GetNextSample(pReadSampleRamp)))
    {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pReadSampleRamp);

        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &m_ramp));
    }

    if(m_vpStop)
    {
        transmitSpeed(0.f);
        if(!m_initTimeSet)
        {
            m_initTime = m_pClock->GetStreamTime();
            m_initTimeSet = tTrue;
        }
        else if ((m_pClock->GetStreamTime()-m_initTime) < 500000)
        {
            transmitBrakeLight(tTrue);
        }
        else
        {
            transmitBrakeLight(tFalse);
        }
        RETURN_NOERROR;
    }

    object_ptr<const ISample> pReadSampleS;

    if (IS_OK(m_oSpeedReader.GetLastSample(pReadSampleS)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSampleS);

        RETURN_IF_FAILED(oDecoder.IsValid());

        tFloat32 speed;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &speed));

        if(speed <= 0.f)
        {
            transmitSpeed(speed);
            RETURN_NOERROR;
        }

        tFloat32 oSpeed = speed;
        tBool brakeLight = tFalse;

        object_ptr<const ISample> pReadSampleSt;

        if (IS_OK(m_oStearingReader.GetLastSample(pReadSampleSt)))
        {
            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSampleSt);

            RETURN_IF_FAILED(oDecoder.IsValid());

            tFloat32 steering;
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &steering));

            object_ptr<const ISample> pReadSample;

            if (IS_OK(m_oLaserReader.GetLastSample(pReadSample)))
            {
                auto oDecoder = m_LaserDataSampleFactory.MakeDecoderFor(*pReadSample);

                RETURN_IF_FAILED(oDecoder.IsValid());

                tUInt32 size;
                RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaserDataId.ui32SizeId, &size));


                const tPolarCoordiante* inputData = static_cast<const tPolarCoordiante *>(oDecoder.GetElementAddress(m_ddlLaserDataId.tScanArrayId));
                if (inputData == nullptr)
                {
                    return ERR_INVALID_INDEX;
                }


                int count = 0;
                tBool stop = tFalse;
                // inputData[i].f32Radius in [mm]
                for (tUInt32 i = 0; i < size; i++)
                {
                    if ((inputData[i].f32Angle > 300.0f || inputData[i].f32Angle < 60.0f) && inputData[i].f32Radius < minDistance + minDistanceExtra * (1.f - (std::min(inputData[i].f32Angle, 360.f - inputData[i].f32Angle))/60.f))
                    {
                        if(inputData[i].f32Radius > 10.f)
                        {
                            count++;
                            //stop = tTrue;
                            //break;
                        }

                    }
                }

                if(stop || count >= minCount)
                {
                    oSpeed = 0.0f;
                    transmitSpeed(oSpeed);

                    if(!m_OvertakeInitTimeSet)
                    {
                        m_OvertakeInitTime = m_pClock->GetStreamTime();
                        m_OvertakeInitTimeSet = tTrue;
                        transmitBrakeLight(tTrue);
                    }
                    else if ((m_pClock->GetStreamTime()-m_OvertakeInitTime) > 500000 && (m_pClock->GetStreamTime()-m_OvertakeInitTime) <1000000)
                    {
                        transmitBrakeLight(tFalse);
                    }
                    else if ((m_pClock->GetStreamTime()-m_OvertakeInitTime) > 1000000 * waitTimeUntilOvertake && !m_OvertakeSent)
                    {
                        transmitOvertake(tTrue);
                        m_OvertakeSent = tTrue;
                    }
                    RETURN_NOERROR;
                }

                if(m_OvertakeSent) {
                    transmitOvertake(tFalse);
                    m_OvertakeSent = tFalse;
                }

                //LOG_INFO(cString("Nothing in the way!"));
                m_OvertakeInitTimeSet = tFalse;


                tFloat32 minRadius = 100000.0f;

                tFloat32 x_peak = width * cbrtf(steering/100);

                for (tUInt32 i = 0; i < size; i++)
                {
                    if ((inputData[i].f32Angle > 300.0f || inputData[i].f32Angle < 60.0f) && inputData[i].f32Radius > minDistance - 50.0f )
                    {
                        tFloat32 x = inputData[i].f32Radius * sin(inputData[i].f32Angle * 3.141f / 180.0f);

                        if(-width < x && x < x_peak)
                        {
                            tFloat32 y = inputData[i].f32Radius * cos(inputData[i].f32Angle * 3.141f / 180.0f);

                            if(y < 100.0f + (x+width)/(x_peak+width)*(maxDistance-100.0f))
                            {
                                minRadius = min(minRadius, inputData[i].f32Radius);
                            }

                        }
                        else if (x_peak <= x && x < width)
                        {
                            tFloat32 y = inputData[i].f32Radius * cos(inputData[i].f32Angle * 3.141f / 180.0f);

                            if(y < 100.0f + (x-width)/(x_peak-width)*(maxDistance-100.0f))
                            {
                                minRadius = min(minRadius, inputData[i].f32Radius);
                            }
                        }
                    }
                }


                if(!m_ramp && minRadius < maxDistance * (1.f + max(0.f, (speed - 0.7f)) * m_factorMaxDistance))
                {
                    oSpeed = std::max(0.01f, speed * fabs((minRadius - minDistance) / (maxDistance - minDistance)));
                    brakeLight = tTrue;
                }
                else if(m_ramp && minRadius < 300.f)
                {
                    transmitRamp();
                }


                transmitSpeed(oSpeed);
                transmitBrakeLight(brakeLight);
            }
        }
    }
    RETURN_NOERROR;
}

tResult cEmergencyBrake::transmitSpeed(tFloat32 value)
{
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, value));

    }
    m_oSpeedWriter << pWriteSample << flush;// << trigger;
    //LOG_INFO(cString::Format("Transmitted speed value %f", value));


    RETURN_NOERROR;
}

tResult cEmergencyBrake::transmitBrakeLight(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_BoolSignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, bValue));

        m_oBrakeLightWriter << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cEmergencyBrake::transmitOvertake(tBool bValue)
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_BoolSignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, bValue));

        m_oOvertake << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cEmergencyBrake::transmitRamp()
{
    object_ptr< ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_BoolSignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, tTrue));

        m_oRampDetector << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}
