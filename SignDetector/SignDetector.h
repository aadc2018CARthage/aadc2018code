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

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

#include "stdafx.h"
#include "aadc_structs.h"
#include "aruco_helpers.h"

#define CID_SIGN_DETECTOR "sign_detector.filter.user.aadc.cid"
#define LABEL_SIGN_DETECTOR "Sign Detector"

/*! The main class for the marker detector module */
class cSignDetector : public cFilter
{
public:
    ADTF_CLASS_ID_NAME(cSignDetector, CID_SIGN_DETECTOR, LABEL_SIGN_DETECTOR);
private:
    //Properties
    /*! Where the Camera Calib File is */
    adtf::base::property_variable<cFilename> m_calibFile = cFilename(cString("basler_fisheye_intrinsic_calib.yml"));
    /*! Where the Detector Parameter File is */
    adtf::base::property_variable<cFilename> m_fileDetectorParameter = cFilename(cString("basler_fisheye_intrinsic_calib.yml"));
    /*! size of the markers*/
    adtf::base::property_variable<tFloat32> m_f32MarkerSize = 0.117f;
    /*! The marker x coordinate */
    adtf::base::property_variable<tFloat32> m_MarkerX;
    /*! The marker y coordinate */
    adtf::base::property_variable<tFloat32> m_MarkerY;
    /*! Width of the marker */
    adtf::base::property_variable<tFloat32> m_MarkerWidth = 10;
    /*! Height of the marker */
    adtf::base::property_variable<tFloat32> m_MarkerHeight = 10;
    /*! true if the roi bounding rectangle is valid and should be used. */
    adtf::base::property_variable<tBool> m_MarkerRoiUsed = tFalse;
    /*! true if marker roi should be shown in the gcl output. */
    adtf::base::property_variable<tBool> m_ShowMarkerRoi = tFalse;
    /* Multiplier that controls ROI-offset depending on Steering input */
    adtf::base::property_variable<tFloat32> m_SteeringMultiplier = 5.0f;
    adtf::base::property_variable<tInt> m_MaxOffset = 300;

    adtf::base::property_variable<tFloat32> m_steeringOffset = -20.f;
    /*! The image pin reader */
    cPinReader m_oReader;
    //Steering Input
    cPinReader m_oSteeringIn;
    // Current Steering Value
    tFloat32 m_SteeringValue;

    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> basler_rgb_runner;
    adtf::base::ant::runnable<adtf::base::ant::IRunnable::RUN_TRIGGER> steering_runner;


    // sample factory for tSignalValue
    adtf::mediadescription::cSampleCodecFactory m_FloatValueFactory;
    struct
    {
        tSize timestamp;
        tSize floatValue;
    } m_ddlFloatIds;

    cPinReader m_oSearchForStartSign;

    tBool m_bSearchForStartSign = tFalse;
    adtf::mediadescription::cSampleCodecFactory m_BoolSampleValueFactory;

    struct
    {
        tSize timestamp;
        tSize bValue;
    } m_ddlBoolIds;

    /*! The image pin writer */
    cPinWriter m_oImagePinWriter;

    /*! The output pose sample factory for DDL access*/
    adtf::mediadescription::cSampleCodecFactory m_outputPoseSampleFactory;

    /*! A ddl road sign extent identifiers. */
    struct
    {
        tSize tVec;
        tSize rVec;
        tSize id;
    } m_ddlRoadSignExtIds;

    /*! The vector pin writer */
    cPinWriter m_oPosePinWriter;

    //Stream Formats
    
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_sInputFormat;
    /*! The output format */
    adtf::streaming::tStreamImageFormat m_sOutputFormat;

    //OpenCV Variables
    
    /*! camera matrix from calibration file */
    cv::Mat m_Intrinsics;
    /*! distorsion coefficients from calibration file */
    cv::Mat m_Distorsion;
    /*! indicates wheter the camara parameters are loaded or not */
    tBool m_bCamaraParamsLoaded = tFalse;

    //Aruco
    /*! the aruco detector for the markers*/
    Ptr<aruco::DetectorParameters> m_detectorParams;
    /*! marker roi bounding rectangle */
    cv::Rect m_MarkerRoi = cv::Rect();
    /*! function to check if the current roi is valid and limit it to the input image size.
    * \return true if roi is still valid, false otherwise
    */
    tBool checkRoi(void);
    /*! the dictionary for the aruco lib*/
    Ptr<aruco::Dictionary> m_Dictionary;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

public:

    /*! Default constructor. */
    cSignDetector();

    /*! Destructor. */
    virtual ~cSignDetector() = default;

    tResult  Init(tInitStage eStage) override;
    tResult  Shutdown(cFilterLevelmachine::tInitStage eStage) override;

    tResult Process(tTimeStamp tmTimeOfTrigger);
    tResult ProcessSteering(tTimeStamp tmTimeOfTrigger);


};
