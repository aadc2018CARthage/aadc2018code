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
#define CID_ZebraDETECTOR_DATA_TRIGGERED_FILTER "ZebraDetector.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;
//#include "aadc_structs.h"

/*! the main class of the open cv template. */
class cZebraDetector : public cTriggerFunction
{
private:

    //Pins
    /*! Reader of an InPin. */	/*! DDL identifier for signal value */
    //   struct tZebraSituationId
    // {
    //       tSize situation;
    //       tSize distance;
    //   } m_ddlZebraSituationId;
    cPinReader m_oVideoIn;
    cPinReader m_oActivateIn;
    /*! Writer to an OutPin. */
    cPinWriter m_oVideoOut;
    // Writer for Situation Out
    cPinWriter m_oSituationOut;


    tBool m_SearchActive;

#pragma pack(push,1)
    typedef struct
    {
        tBool Situation;
        tFloat32 Distance;
    } tZebraSituation;
#pragma pack(pop)

    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;

    struct tZebraSituationId
    {
        tSize situation;
        tSize distance; // distance to Zebra, might not be needed.
    } m_ddlZebraSituationId;

    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_ZebraSituationSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;
    //Stream Formats	/*! DDL identifier for signal value */
    //   struct tZebraSituationId
    // {
    //       tSize situation;
    //       tSize distance;
    //   } m_ddlZebraSituationId;
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    struct tROI
    {
        adtf::base::property_variable<tInt> offsetX;
        adtf::base::property_variable<tInt> offsetY;
        adtf::base::property_variable<tInt> width;
        adtf::base::property_variable<tInt> height;
    };

    struct tLineWidth
    {
        adtf::base::property_variable<tInt> minWidth;
        adtf::base::property_variable<tInt> maxWidth;
    };

    struct
	{
        adtf::base::property_variable<tBool> showDebug = tTrue;
        adtf::base::property_variable<tFloat32> binaryThreshold = 190.0f;
        adtf::base::property_variable<tInt> numLines = 5;
        adtf::base::property_variable<tInt> lineCount = 5;
        tROI roi = { 540, 450, 250, 100};
        tLineWidth lineWidth = {20,50};
        adtf::base::property_variable<tInt> lineCountTol = 2;

	} m_Properties;

    std::mutex m_oLaneImageMutex;
    std::mutex m_oLaneChangeMutex;

    tResult searchForZebra(const cv::Mat* image);
    tBool processROI(const tROI& roiVars, const cv::Mat* image, cv::Mat* laneImage);
    tResult setupProperties();
    tResult TransmitZebraSituation(tZebraSituation ZebraSituation);
    tInt lineCounter(const uchar* linePtr, const int cols);


public:
	/*! Default constructor. */
    cZebraDetector();

	/*! Destructor. */
    virtual ~cZebraDetector() = default;

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
};


//*************************************************************************************************
