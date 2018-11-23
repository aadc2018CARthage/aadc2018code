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
#include "DriveListDummy.h"
#include "aadc_structs.h"
#include <vector>

ADTF_PLUGIN(LABEL_DRIVE_LIST_DUMMY , cDriveListDummy)


cDriveListDummy::cDriveListDummy()
{
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeDriveDistanceCom;
    if( IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriveDistanceCom", pTypeDriveDistanceCom, m_DriveDistanceComSampleFactory)))
    {
        adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("comID"), m_ddlDriveDistanceComId.comID);

       adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("distanceToDrive"), m_ddlDriveDistanceComId.distance);

       adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("speedToDrive"), m_ddlDriveDistanceComId.speed);

       adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("stopAfter"), m_ddlDriveDistanceComId.stopAfter);

       adtf_ddl::access_element::find_index(m_DriveDistanceComSampleFactory,   cString("steeringToDrive"), m_ddlDriveDistanceComId.steering);
    }
    else
    {
        LOG_WARNING("No mediadescription for DriveDistanceCom found!");
    }

    filter_create_pin(*this, m_oWriterDriveDistanceCom, "DriveDistanceCom", pTypeDriveDistanceCom);

    object_ptr<IStreamType> pTypeInt;
    if( IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tIntSignalValue", pTypeInt, m_IntSignalValueSampleFactory)))
    {
        adtf_ddl::access_element::find_index(m_IntSignalValueSampleFactory,   cString("value"), m_ddlIntSignalValueId.value);
    }
    else
    {
        LOG_WARNING("No mediadescription for tIntSignalValue found!");
    }

    filter_create_pin(*this, m_oInputComID, "comID", pTypeInt);
    m_oInputIntRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessComID));
    RegisterRunner("int_runner", m_oInputIntRunner);
    ConfigureDataInTrigger("int_runner", "comID");

    object_ptr<IStreamType> pTypeBool;
    if( IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBool, m_BoolSignalValueSampleFactory)))
    {
        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory,   cString("bValue"), m_ddlBoolSignalValueId.bValue);
    }
    else
    {
        LOG_WARNING("No mediadescription for tBoolSignalValue found!");
    }

    filter_create_pin(*this, m_oInputStart , "startBool", pTypeBool);
    m_oInputBoolRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessInit));
    RegisterRunner("start_runner", m_oInputBoolRunner);
    ConfigureDataInTrigger("start_runner", "startBool");

    //initialize your DriveList here!!!
    //
    //

    tDriveDistanceComStruct com1 {1, 2.0f, 0.5f, tFalse, 200.0f};
    tDriveDistanceComStruct com2 {2, 1.0f, 0.3f, tFalse, 200.0f};
    tDriveDistanceComStruct com3 {3, 4.0f, 0.5f, tFalse, 200.0f};
    tDriveDistanceComStruct com4 {4, 1.0f, 0.2f, tTrue, -30.0f};

    DriveList.push_back(com1);
    DriveList.push_back(com2);
    DriveList.push_back(com3);
    DriveList.push_back(com4);

    //
    //
    m_hasStarted = tFalse;

}

tResult cDriveListDummy::Init(tInitStage eStage)
{
    RETURN_NOERROR;
}

tResult cDriveListDummy::Shutdown(cFilterLevelmachine::tInitStage eStage)
{
    RETURN_NOERROR;
}

tResult cDriveListDummy::ProcessInit(tTimeStamp tmTimeOfTrigger)
{
    if(!m_hasStarted)
    {
        object_ptr<const ISample> pReadSample;
        if IS_OK(m_oInputStart.GetLastSample(pReadSample))
        {
            m_hasStarted = tTrue;
            std::lock_guard<std::mutex> lock(m_oMutex);
            if(!DriveList.empty())
            {
                tDriveDistanceComStruct com = DriveList.front();
                DriveList.erase(DriveList.begin());

                object_ptr< ISample> pWriteSample;
                if (IS_OK(alloc_sample(pWriteSample)))
                {
                    auto oCodec = m_DriveDistanceComSampleFactory.MakeCodecFor(pWriteSample);

                    RETURN_IF_FAILED(oCodec.IsValid());

                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.comID, com.comID));
                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.distance, com.distanceToDrive));
                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.speed, com.speedToDrive));
                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.stopAfter, com.stopAfter));
                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.steering,
                                                            com.steeringtoDrive));
                    m_oWriterDriveDistanceCom << pWriteSample << flush << trigger;
                }
            }
        }
    }
    RETURN_NOERROR;
}

tResult cDriveListDummy::ProcessComID(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oInputComID.GetLastSample(pReadSample)))
    {
        tInt32 comID = -1;
        auto oDecoder = m_IntSignalValueSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlIntSignalValueId.value , &comID));

        std::lock_guard<std::mutex> lock(m_oMutex);
        if(!DriveList.empty())
        {
            tDriveDistanceComStruct com = DriveList.front();
            DriveList.erase(DriveList.begin());

            object_ptr< ISample> pWriteSample;
            if (IS_OK(alloc_sample(pWriteSample)))
            {
                auto oCodec = m_DriveDistanceComSampleFactory.MakeCodecFor(pWriteSample);

                RETURN_IF_FAILED(oCodec.IsValid());

                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.comID, com.comID));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.distance, com.distanceToDrive));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.speed, com.speedToDrive));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.stopAfter, com.stopAfter));
                RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriveDistanceComId.steering, com.steeringtoDrive));
            }

            m_oWriterDriveDistanceCom << pWriteSample << flush << trigger;
        }
    }
    RETURN_NOERROR;
}
