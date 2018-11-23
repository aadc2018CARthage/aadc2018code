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
#include "NetworkSwitch.h"
#include "aadc_structs.h"

ADTF_PLUGIN(LABEL_NETWORK_SWITCH_FILTER, cNetworkSwitch)

cNetworkSwitch::cNetworkSwitch()
{
    object_ptr<IStreamType> pTypeSignalValue;
    if( IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)))
    {
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    filter_create_pin(*this, m_oInputDDSpeed, "DDSpeed", pTypeSignalValue);
    m_oInputDDSpeedRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessDDSpeed));
    RegisterRunner("ddspeed_runner", m_oInputDDSpeedRunner);
    ConfigureDataInTrigger("ddspeed_runner", "DDSpeed");

    filter_create_pin(*this, m_oInputDDSteering, "DDSteering", pTypeSignalValue);
    m_oInputDDSteeringRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessDDSteering));
    RegisterRunner("ddsteering_runner", m_oInputDDSteeringRunner);
    ConfigureDataInTrigger("ddsteering_runner", "DDSteering");

    filter_create_pin(*this, m_oInputNetSpeed, "NetSpeed", pTypeSignalValue);
    m_oInputNetSpeedRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessNetSpeed));
    RegisterRunner("netspeed_runner", m_oInputNetSpeedRunner);
    ConfigureDataInTrigger("netspeed_runner", "NetSpeed");

    filter_create_pin(*this, m_oInputNetSteering, "NetSteering", pTypeSignalValue);
    m_oInputNetSteeringRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessNetSteering));
    RegisterRunner("netsteering_runner", m_oInputNetSteeringRunner);
    ConfigureDataInTrigger("netsteering_runner", "NetSteering");

    filter_create_pin(*this, m_oWriterSpeed, "speed" , pTypeSignalValue);
    filter_create_pin(*this, m_oWriterSteering, "steering", pTypeSignalValue);

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue);
    }
    else
    {
        LOG_WARNING("No mediadescription for tBoolSignalValue found!");
    }

    filter_create_pin(*this, m_oInputSwitch, "switch", pTypeBoolSignalValue);
    m_oInputSwitchRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessSwitch));
    RegisterRunner("switch_runner", m_oInputSwitchRunner);
    ConfigureDataInTrigger("switch_runner", "switch");
}

tResult cNetworkSwitch::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cNetworkSwitch::Shutdown(cFilterLevelmachine::tInitStage eStage)
{
    RETURN_NOERROR;
}

tResult cNetworkSwitch::ProcessSwitch(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oInputSwitch.GetNextSample(pReadSample)))
    {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pReadSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &m_bUseNetwork));
    }
    RETURN_NOERROR;
}

tResult cNetworkSwitch::ProcessDDSpeed(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oInputDDSpeed.GetNextSample(pReadSample)) && !m_bUseNetwork)
    {
        m_oWriterSpeed << pReadSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cNetworkSwitch::ProcessNetSpeed(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oInputNetSpeed.GetNextSample(pReadSample)) && m_bUseNetwork)
    {
        m_oWriterSpeed << pReadSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cNetworkSwitch::ProcessDDSteering(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oInputDDSteering.GetNextSample(pReadSample)) && !m_bUseNetwork)
    {
        m_oWriterSteering << pReadSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cNetworkSwitch::ProcessNetSteering(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oInputNetSteering.GetNextSample(pReadSample)) && m_bUseNetwork)
    {
        m_oWriterSteering << pReadSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cNetworkSwitch::TransmitSpeed(tFloat32 value)
{
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

tResult cNetworkSwitch::TransmitSteering(tFloat32 value)
{
    object_ptr<ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, value));
        m_oWriterSteering << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}
