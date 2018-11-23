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
#define CID_LANEKEEPING_DATA_TRIGGERED_FILTER "laneKeeping.filter.user.aadc.cid"

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
class cLanekeeping : public cTriggerFunction
{
private:

	//Pins
	/*! Reader of an InPin. */
	cPinReader m_oVideoIn;
    cPinReader m_oWheelLeftIn;
    cPinReader m_oLaneChangeIn;
    cPinReader m_SetRoiIn;
    cPinReader m_oUseRoiLeftIn;
    cPinReader m_oUseRoiRightIn;
    cPinReader m_oStopLaneChange;
	cPinReader m_oLastSteering;
	/*! Writer to an OutPin. */
	cPinWriter m_oVideoOut;
	// Writer for Steering Out
	cPinWriter m_oSteeringOut;

	/*! DDL identifier for signal value */
	struct tSignalValueId
	{
		tSize timeStamp;
	    tSize value;
	} m_ddlSignalValueId;


    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;
    struct tWheelDataId
    {
        tSize ArduinoTimestamp;
        tSize WheelTach;
        tSize WheelDir;
    } m_ddlWheelDataId;




	/*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_WheelDataSampleFactory;
	adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;
	//Stream Formats
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

		adtf::base::property_variable<tInt> refUpperX;
		adtf::base::property_variable<tInt> refLowerX;
		adtf::base::property_variable<tInt> refUpperY;
		adtf::base::property_variable<tInt> refLowerY;

		adtf::base::property_variable<tInt> maxDynWidth;
		tInt dynWidth;
	};

	struct tLaneSwitch
	{
		adtf::base::property_variable<tFloat32> distance;
		adtf::base::property_variable<tFloat32> offset;
		adtf::base::property_variable<tInt> offsetROI;
	};

	struct
	{
		adtf::base::property_variable<tBool> showLanekeeping = tTrue;

		tROI roiRight = { 710, 685, 300, 30, 835, 865, 685, 720, 240, 0 };
		tROI roiLeft = { 300, 685, 200, 30, 410, 375, 685, 720, 240, 0 };

		adtf::base::property_variable<tInt> detectionLines = 28;
		adtf::base::property_variable<tInt> minLineWidth = 8;
		adtf::base::property_variable<tInt> maxLineWidth = 50;
		adtf::base::property_variable<tInt> minLineContrast = 20;

		adtf::base::property_variable<tFloat32> upperBound = 200;
		adtf::base::property_variable<tInt> dynWidthMultiplier = 5;
		adtf::base::property_variable<tFloat32> steeringMultiplier = 1;
		adtf::base::property_variable<tFloat32> steeringOffset = 0;

		tLaneSwitch laneSwitchLeftToRight = { 1.5f, -60.f, 60 };
		tLaneSwitch laneSwitchRightToLeft = { 1.5f, 75.f, -60 };

		adtf::base::property_variable<tBool> errorCorrection = tTrue;
        adtf::base::property_variable<tFloat32> errorCorrThreshold = 25;
	} m_Properties;


	tFloat32 steeringValue;
	tFloat32 lastSteeringValue;
	tFloat32 distanceDriven;

    tBool m_useRoiLeft;
    tBool m_useRoiRigth;

    tBool m_laneChangeActive;
    tBool m_laneSwitchdistanceAtStartSet;
    tFloat32 m_laneSwitchdistanceAtStart;
	const tInt LANE_LEFT = -1;
	const tInt LANE_RIGHT = 1;
    tInt targetLane;

    std::mutex m_oLaneImageMutex;
    std::mutex m_oLaneChangeMutex;

    std::pair<tFloat32, tInt> processROI(const tROI& roiVars, const cv::Mat* image, cv::Mat* laneImage);
    std::vector<Point> getLinePoints(const tROI& roiVars, const cv::Mat* image);
	tVoid discardBadPoints(std::vector<Point>& points);
	tVoid switchLane();
	tVoid switchLane(const tLaneSwitch& laneSwitch);

	tResult setupProperties();
	tResult CalculateSteering(const cv::Mat* image, tTimeStamp inputtimestamp);
	tResult TransmitSteering(tFloat32 steeringangle, tTimeStamp timestamp);
    tFloat32 getDistance();
    tFloat32 ConvertWheelToMeter(tUInt32 value);


public:
	/*! Default constructor. */
	cLanekeeping();

	/*! Destructor. */
	virtual ~cLanekeeping() = default;

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
