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

#include "stdafx.h"
#include "aadc_structs.h"
#include <a_utils/core/a_utils_core.h>
//*************************************************************************************************
#define CID_DRIVELISTDUMMY_FILTER "drivelistdummy_filter.filter.user.aadc.cid"
#define LABEL_DRIVE_LIST_DUMMY "DriveListDummy"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cDriveListDummy : public cFilter
{
public:
    ADTF_CLASS_ID_NAME(cDriveListDummy, CID_DRIVELISTDUMMY_FILTER, LABEL_DRIVE_LIST_DUMMY);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:
    /*! Media Descriptions. */
    struct tDriveDistanceComId
    {
        tSize comID;
        tSize distance;
        tSize speed;
        tSize stopAfter;
        tSize steering;
    } m_ddlDriveDistanceComId;

    adtf::mediadescription::cSampleCodecFactory m_DriveDistanceComSampleFactory;

    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;

    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;

    struct tIntSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlIntSignalValueId;

    adtf::mediadescription::cSampleCodecFactory m_IntSignalValueSampleFactory;

    cPinWriter m_oWriterDriveDistanceCom;

    cPinReader m_oInputComID;
    cPinReader m_oInputStart;
    tBool m_hasStarted;

    //DriveList
    typedef struct {
        tInt32    comID;
        tFloat32  distanceToDrive; //[m]
        tFloat32  speedToDrive;
        tBool     stopAfter;
        tFloat32  steeringtoDrive;
    } tDriveDistanceComStruct;

    vector<tDriveDistanceComStruct> DriveList;

    std::mutex m_oMutex;

    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER>
    m_oInputIntRunner;

    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER>
    m_oInputBoolRunner;

    object_ptr<adtf::services::IReferenceClock> m_pClock;

public:

    /*! Default constructor. */
    cDriveListDummy();

    /*! Destructor. */
    virtual ~cDriveListDummy() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    //tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult  Init(tInitStage eStage) override;
    tResult  Shutdown(cFilterLevelmachine::tInitStage eStage) override;

    tResult ProcessComID(tTimeStamp tmTimeOfTrigger);

    tResult ProcessInit(tTimeStamp tmTimeOfTrigger);



};


//*************************************************************************************************
