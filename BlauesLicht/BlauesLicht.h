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
#define CID_BlauesLicht_DATA_TRIGGERED_FILTER "BlauesLicht.filter.user.aadc.cid"

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
class cBlauesLicht : public cTriggerFunction
{
public:
    struct tROI
    {
        adtf::base::property_variable<tInt> offsetX;
        adtf::base::property_variable<tInt> offsetY;
        adtf::base::property_variable<tInt> width;
        adtf::base::property_variable<tInt> height;
    };
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
    cPinWriter m_oVideoOutHough;
    cPinWriter m_oLastVideo;
    // Writer for Situation Out
    cPinWriter m_oBoolOut;


    tBool m_SearchActive;


    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;


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




    struct tHoughParam
    {
        adtf::base::property_variable<tInt> dp;
        adtf::base::property_variable<tInt> minDist;
        adtf::base::property_variable<tInt> param1;
        adtf::base::property_variable<tInt> param2;
        adtf::base::property_variable<tInt> minRadius;
        adtf::base::property_variable<tInt> maxRadius;
    };

    struct tCntParam
    {
        adtf::base::property_variable<tInt> CannyThr1;
        adtf::base::property_variable<tInt> CannyThr2;
        adtf::base::property_variable<tFloat32> periError;
        adtf::base::property_variable<tFloat32> minArea;
        adtf::base::property_variable<tFloat32> maxAreaDev;
        adtf::base::property_variable<tInt> sigmaColor;

    };

    struct tBlurrParam
    {
        adtf::base::property_variable<tInt> kSize;
        adtf::base::property_variable<tInt> sigma;
    };

    struct tLineWidth
	{
        adtf::base::property_variable<tInt> minWidth;
        adtf::base::property_variable<tInt> maxWidth;
	};

    struct
    {
       //daily reminder: this stuff is "optimized" only for the front camera of car 004
       adtf::base::property_variable<tBool> showDebug = tTrue;
       adtf::base::property_variable<tFloat32> w = -0.45;
       adtf::base::property_variable<tBool> FrontCamera = tTrue; //True: FrontCam|False: RearCam|QuickTipp: only car 003 has a working rear cam

       tBlurrParam blurr = {7,5};
       tROI roi = { 540, 450, 250, 100};
       tHoughParam hough = {1, 100, 60, 30, 0, 20};
       tCntParam Cnt ={40,60, 0.02, 50, 4, 9}; //sigmaColor 12 sieht auch nice aus
	} m_Properties;

    std::mutex m_oLaneImageMutex;
    std::mutex m_oLaneChangeMutex;


    tBool blauesLichtFinderCnt(const tROI& roiVars, const cv::Mat* image, cv::Mat* laneImage);
    tBool blauesLichtFinderHo(const tROI& roiVars, const cv::Mat* image, cv::Mat* laneImage);
    tResult searchForBlauesLicht (const Mat *image);
    tResult setupProperties();
    double calculateAspect(vector<Point> contours);
    float calculateRadius(tFloat area);



public:
	/*! Default constructor. */
    cBlauesLicht();

	/*! Destructor. */
    virtual ~cBlauesLicht() = default;

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
