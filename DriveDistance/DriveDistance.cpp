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
#include "DriveDistance.h"
#include "aadc_structs.h"

ADTF_PLUGIN(LABEL_DRIVE_DISTANCE_FILTER, cDriveDistance)

cDriveDistance::cDriveDistance()
{
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeDriveDistanceCom;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriveDistanceCom", pTypeDriveDistanceCom, m_DriveDistanceComSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("comID"), m_ddlDriveDistanceComId.comID);

        adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("distanceToDrive"), m_ddlDriveDistanceComId.distance);

        adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("speedToDrive"), m_ddlDriveDistanceComId.speed);

        adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("stopAfter"), m_ddlDriveDistanceComId.stopAfter);

        adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("steeringToDrive"), m_ddlDriveDistanceComId.steering);
    }
    else
    {
        LOG_WARNING("No mediadescription for tDriveDistanceCom found!");
    }

    filter_create_pin(*this, m_oInputDriveDistanceCom, "DriveDistanceCom", pTypeDriveDistanceCom);
    m_oInputDriveDistanceComRunner = runnable<IRunnable::RUN_TRIGGER >(ADTF_RUN_FUNCTION(ProcessDriveDistanceCom));
    RegisterRunner("drive_distance_com_runner", m_oInputDriveDistanceComRunner);
    ConfigureDataInTrigger("drive_distance_com_runner", "DriveDistanceCom");


    object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData, m_WheelDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataId.ArduinoTimestamp);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataId.WheelTach);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataId.WheelDir);
    }
    else
    {
        LOG_INFO("No mediadescription for tWheelData found!");
    }

    filter_create_pin(*this, m_oInputCurrentDistance, "WheelLeft" , pTypeWheelData);
    m_oInputWheelDataRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessWheelData));
    RegisterRunner("wheel_data_runner", m_oInputWheelDataRunner);
    ConfigureDataInTrigger("wheel_data_runner", "WheelLeft");


    object_ptr<IStreamType> pTypeSignalValue;
    if( IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)))
    {
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    filter_create_pin(*this, m_oInputLanekeeping, "Lanekeeping", pTypeSignalValue);
    m_oInputLanekeepingRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessLanekeeping));
    RegisterRunner("lanekeeping_runner", m_oInputLanekeepingRunner);
    ConfigureDataInTrigger("lanekeeping_runner", "Lanekeeping");

    filter_create_pin(*this, m_oInputSpeedScalar, "speed_scalar", pTypeSignalValue);
    m_oInputSpeedScalarRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessSpeedScalar));
    RegisterRunner("speed_scalar_runner", m_oInputSpeedScalarRunner);
    ConfigureDataInTrigger("speed_scalar_runner", "speed_scalar");

    filter_create_pin(*this, m_oWriterSpeed, "speed" , pTypeSignalValue);
    filter_create_pin(*this, m_oWriterSteering, "steering", pTypeSignalValue);


    object_ptr<IStreamType> pTypeInt;
    if( IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tIntSignalValue", pTypeInt, m_IntSignalValueSampleFactory)))
    {
        adtf_ddl::access_element::find_index(m_IntSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlIntSignalValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_IntSignalValueSampleFactory, cString("value"), m_ddlIntSignalValueId.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tIntSignalValue found!");
    }

    filter_create_pin(*this, m_oWriterInt, "comID", pTypeInt);

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue);
    }
    else
    {
        LOG_WARNING("No mediadescription for tBoolSignalValue found!");
    }

    filter_create_pin(*this, m_oWriterBrakeLight, "brake_light", pTypeBoolSignalValue);
    filter_create_pin(*this, m_oInputBrakeLightEB, "brake_light_EB", pTypeBoolSignalValue);
    filter_create_pin(*this, m_oInputBrakeLightCore, "brake_light_Core", pTypeBoolSignalValue);

    //init private attributes

    m_comID = -1;

    m_distanceToDrive = 1000.0f; //[m]

    m_speedToDrive = 0.0f;

    m_stopAfter = tTrue;

    m_steeringToDrive = -150.0f;

    m_overallDistanceAtStart = 0.0f;

    m_overallDistanceSet = tTrue;

    m_comFinished = tTrue;

    //Register Properties

    RegisterPropertyVariable("Distance to stop", m_stopDistance);
}




tResult cDriveDistance::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cDriveDistance::Shutdown(cFilterLevelmachine::tInitStage eStage)
{
    RETURN_NOERROR;
}

tResult cDriveDistance::ProcessDriveDistanceCom(tTimeStamp tmTimeOfTrigger)
{
    std::lock_guard<mutex> lock(m_oMutex);
    object_ptr<const ISample> pReadSample;

    if (IS_OK(m_oInputDriveDistanceCom.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_DriveDistanceComSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        tBool oldStopAfter = m_stopAfter;
        tFloat32 oldSpeed = m_speedToDrive;

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriveDistanceComId.comID, &m_comID));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriveDistanceComId.distance, &m_distanceToDrive));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriveDistanceComId.speed, &m_speedToDrive));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriveDistanceComId.stopAfter, &m_stopAfter));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriveDistanceComId.steering, &m_steeringToDrive));

        if(!m_comFinished)
        {
            m_overallDistanceSet = tFalse;
        }
        m_comFinished = tFalse;

        //LOG_INFO(cString::Format("received comID: %d", m_comID));

        if(m_speedToDrive == 0.f && !oldStopAfter && oldSpeed != 0.f)
        {
            m_bBrakeLight = tTrue;
            m_initTimeSet = tFalse;
        }

        TransmitSpeed(m_speedToDrive);
    }
    RETURN_NOERROR;
}

tResult cDriveDistance::ProcessWheelData(tTimeStamp tmTimeOfTrigger)
{
    std::lock_guard<mutex> lock(m_oMutex);
    if(!m_comFinished)
    {
        object_ptr<const ISample> pReadSample;
        if (IS_OK(m_oInputCurrentDistance.GetLastSample(pReadSample)))
        {
            auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pReadSample);

            RETURN_IF_FAILED(oDecoder.IsValid());
            tUInt32 valueI;
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataId.WheelTach, &valueI));

            tFloat32 valueM = ConvertWheelToMeter(valueI);

            tFloat32 speed = m_speedScalar * m_speedToDrive;
            tBool lightOn = tFalse;

            if(!m_overallDistanceSet)
            {
                m_overallDistanceAtStart = valueM;
                m_overallDistanceSet = tTrue;
            }
            else if (valueM-m_overallDistanceAtStart<m_distanceToDrive)
            {
                if (m_stopAfter && valueM-m_overallDistanceAtStart>m_distanceToDrive-m_stopDistance)
                {
                    speed = m_speedToDrive * ((m_distanceToDrive-(valueM-m_overallDistanceAtStart))/m_stopDistance);
                    lightOn = tTrue;
                }
            }
            else
            {
                if(m_stopAfter)
                {
                    speed = 0.0f;
                }
                //LOG_INFO(cString::Format("sent comID: %d", m_comID));
                m_comFinished = tTrue;
                m_overallDistanceAtStart = valueM;
                TransmitInt(m_comID);
            }

            TransmitSpeed(speed);

            if(m_bBrakeLight)
            {
                if(!m_initTimeSet)
                {
                    m_initTime = m_pClock->GetStreamTime();
                    m_initTimeSet = tTrue;
                    TransmitBrakeLight(tTrue);
                }
                else if ((m_pClock->GetStreamTime()-m_initTime) > 500000)
                {
                    TransmitBrakeLight(tFalse);
                    m_bBrakeLight = tFalse;
                }
                RETURN_NOERROR;
            }

            tBool bValueEB = tFalse;
            tBool bValueC = tFalse;
            object_ptr<const ISample> pReadSampleB;
            if(IS_OK(m_oInputBrakeLightEB.GetLastSample(pReadSampleB)))
            {
                auto oDecoderB = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pReadSampleB);
                RETURN_IF_FAILED(oDecoderB.IsValid());
                RETURN_IF_FAILED(oDecoderB.GetElementValue(m_ddlBoolSignalValueId.bValue, &bValueEB));
            }

            object_ptr<const ISample> pReadSampleC;
            if(IS_OK(m_oInputBrakeLightCore.GetLastSample(pReadSampleC)))
            {
                auto oDecoderC = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pReadSampleC);
                RETURN_IF_FAILED(oDecoderC.IsValid());
                RETURN_IF_FAILED(oDecoderC.GetElementValue(m_ddlBoolSignalValueId.bValue, &bValueC));
            }

            TransmitBrakeLight((lightOn || bValueEB || bValueC));
        }
    }
    RETURN_NOERROR;
}



tResult cDriveDistance::ProcessLanekeeping(tTimeStamp tmTimeOfTrigger)
{
    std::lock_guard<mutex> lock(m_oMutex);
    if (!m_comFinished)
    {
        tFloat32 steering = m_steeringToDrive;
        if(steering > 100.0f || steering < -100.0f)
        {
            object_ptr<const ISample> pReadSample;
            if(IS_OK(m_oInputLanekeeping.GetLastSample(pReadSample)))
            {
                m_oWriterSteering << pReadSample << flush << trigger;
            }
        }
        else
        {

            object_ptr<ISample> pWriteSample;

            if(IS_OK(alloc_sample(pWriteSample)))
            {
                auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);
                RETURN_IF_FAILED(oCodec.IsValid());
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, steering));
            }

            m_oWriterSteering << pWriteSample << flush << trigger;
        }
    }
    RETURN_NOERROR;
}

tResult cDriveDistance::ProcessSpeedScalar(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSample;
    if(IS_OK(m_oInputSpeedScalar.GetNextSample(pSample)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &m_speedScalar));
    }
    RETURN_NOERROR;
}

tResult cDriveDistance::TransmitSpeed(tFloat32 value)
{
    // value = value*30.0f;
    object_ptr<ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, value));
        m_oWriterSpeed << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cDriveDistance::TransmitBrakeLight(tBool bValue)
{
    object_ptr<ISample> pWriteSample;
    if(IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_BoolSignalValueSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, bValue));
        m_oWriterBrakeLight << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}


tResult cDriveDistance::TransmitInt(tInt32 value)
{
    object_ptr<ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_IntSignalValueSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlIntSignalValueId.value, value));
        m_oWriterInt << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tFloat32 cDriveDistance::ConvertWheelToMeter(tUInt32 value)
{
    tFloat32 result = (value/60.0f)*0.34f;
    return result;
}
