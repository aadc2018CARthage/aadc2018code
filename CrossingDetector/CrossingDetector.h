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

#include "stdafx.h"
#include "aadc_structs.h"

#define CID_CROSSING_DETECTOR_FILTER "crossingDetector.filter.user.aadc.cid"
#define LABEL_CROSSING_DETECTOR "CrossingDetector"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

/*! the main class of the open cv template. */
class cCrossingDetector : public cFilter
{
public:
    ADTF_CLASS_ID_NAME(cCrossingDetector, CID_CROSSING_DETECTOR_FILTER, LABEL_CROSSING_DETECTOR);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

	//Pins
	/*! Reader of an InPin. */
	cPinReader m_oVideoIn;
    cPinReader m_oActivateIn;
    cPinReader m_SteeringIn;
    cPinReader m_SpeedIn;
	/*! Writer to an OutPin. */
	cPinWriter m_oVideoOut;
	// Writer for Steering Out
    cPinWriter m_oSituationOut;

    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> m_oVideo_runner;
	/*! DDL identifier for signal value */
    struct tCrossingSituationId
	{
        tSize situation;
        tSize distance;
    } m_ddlCrossingSituationId;

    struct tSignalValueId
    {
        tSize timeStamp;
        tSize f32Value;
    } m_ddlSignalValueId;

    struct tIntSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlIntSignalValueId;


	/*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_IntSignalValueSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_CrossingSituationSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

	//Stream Formats
	/*! The input format */
	adtf::streaming::tStreamImageFormat m_sImageFormat;

	/*! The clock */
	object_ptr<adtf::services::IReferenceClock> m_pClock;

    /* Steering and SPeed Values */
    tFloat32 m_Steering, m_Speed;

    //cases: 0: off, 1: crossing, 2: parking
    tInt32 m_active = 0;

	struct tROI
	{
		adtf::base::property_variable<tInt> offsetX;
		adtf::base::property_variable<tInt> offsetY;
		adtf::base::property_variable<tInt> width;
        adtf::base::property_variable<tInt> height;
        adtf::base::property_variable<tInt> houghAccThreshold;
        adtf::base::property_variable<tInt> houghAccThresholdParking;
	};
    struct tCanny
    {
        adtf::base::property_variable<tInt> thresholdOne;
        adtf::base::property_variable<tInt> thresholdTwo;
    };
    struct tCorner
    {
        adtf::base::property_variable<tFloat32> qualityMeasure;
        adtf::base::property_variable<tFloat32> maxDistance;
        adtf::base::property_variable<tInt> numberOfCorners;
        adtf::base::property_variable<tFloat32> yCorrFactor;
        adtf::base::property_variable<tFloat32> xCorrFactor;
        adtf::base::property_variable<tFloat32> lowDistance;
        adtf::base::property_variable<tFloat32> highDistance;
        adtf::base::property_variable<tInt> maxOffset;
        adtf::base::property_variable<tFloat32> offsetFactor;
    };

	struct
	{
        adtf::base::property_variable<tBool> showDebug = tTrue;
        adtf::base::property_variable<tFloat32> CrossingAngleTolerance = 0.05;
        tROI roiRight = { 979, 555, 300, 30, 180, 70};
        tROI roiLeft = { 0, 555, 200, 30, 100, 100};
        tROI cornerROI = { 600, 450, 200, 200, 200};
        tCanny CannyParam = {100, 130};
        tCorner CornerParam = {0.6f, 0.04f, 8, 0.25f, 0.005f, 0.36f, 1.2f, 300, 5.0f};

		property_variable<tFloat32> steeringDeadband = 100.f;
		property_variable<tFloat32> curveFactor = 0.f;
		property_variable<tFloat32> curveConstant = 0.1f;
	} m_Properties;

    tFloat32 m_CrossingAngle;
    tFloat32 m_CrossingRadius;

    std::mutex m_oLaneImageMutex;
    std::mutex m_oLaneChangeMutex;

public:
	/*! Default constructor. */
    cCrossingDetector();

	/*! Destructor. */
    virtual ~cCrossingDetector() = default;

    tResult Init(tInitStage eStage) override;

    tResult Shutdown(cFilterLevelmachine::tInitStage eStage) override;
	/**
	 * Overwrites the Process
	 * You need to implement the Reading and Writing of Samples within this function
	 * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
	 * This FUnction will be called if the Run() of the TriggerFunction was called.
	 */
    tResult Process(tTimeStamp tmTimeOfTrigger);
    tResult searchForCrossing(const cv::Mat* image);
    tBool processROI(const tROI& roiVars, const cv::Mat* image, cv::Mat* laneImage);
    tFloat32 getDistance(const tROI& roiVars, const cv::Mat* image, cv::Mat* crossingImage);
    tResult setupProperties();
    tResult TransmitCrossingSituation(tCrossingSituation);
};


//*************************************************************************************************
