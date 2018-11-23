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

#define CID_ImgCollector_FILTER "ImgCollector.filter.user.aadc.cid"
#define LABEL_ImgCollector_FILTER  "ImgCollector"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

/*! This is the main class for the filter for object detection with caffe. */
class cImgCollectorDetection : public cFilter
{
public:
    ADTF_CLASS_ID_NAME(cImgCollectorDetection, CID_ImgCollector_FILTER, LABEL_ImgCollector_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    /*! The reader for image data*/
    cPinReader m_oReader;
    cPinReader m_oActivateIn;

    /*! The writer for output image */
    cPinWriter m_oWriter;

    tBool m_active;

    property_variable<cFilename> m_directory = cFilename("home/aadc/Desktop/Bilder");

    property_variable<tInt32> m_collectionRate = 1;

    tTimeStamp m_lastTimeStamp = 0;

    adtf::base::ant::runnable<> m_oThreadRunner;

    struct tROI
    {
        adtf::base::property_variable<tInt> offsetX;
        adtf::base::property_variable<tInt> offsetY;
        adtf::base::property_variable<tInt> width;
        adtf::base::property_variable<tInt> height;
    };


    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;

    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;





    /*! The codec factory */
    cSampleCodecFactory m_oCodecFactory;

    /*! The current stream imageformat */
    adtf::streaming::tStreamImageFormat m_sCurrentFormat;

    property_variable<tBool> m_showDebugVideo = tTrue;
    property_variable<tBool> m_usePNG = tTrue;


    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

public:

    /*! Default constructor. */
    cImgCollectorDetection();

    /*! Destructor. */
    virtual ~cImgCollectorDetection() = default;

    tResult  Init(tInitStage eStage) override;
    tResult  Shutdown(cFilterLevelmachine::tInitStage eStage) override;

    /*!
     * Process the last Sample.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     *
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger);

};
