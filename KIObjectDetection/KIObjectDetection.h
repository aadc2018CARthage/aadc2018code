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

#define CID_KI_OBJECT_DETECTION_FILTER "ki_object_detection.filter.user.aadc.cid"
#define LABEL_KI_OBJECT_DETECTION_FILTER  "KI Object Detection"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

/*! This is the main class for the filter for object detection with caffe. */
class cKIObjectDetection : public cFilter
{
public:
    ADTF_CLASS_ID_NAME(cKIObjectDetection, CID_KI_OBJECT_DETECTION_FILTER, LABEL_KI_OBJECT_DETECTION_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    enum class Objects : tInt32 { Car = 3, Person = 1, Child = 2, Other = 0}; //have to be assigned according to the trained neural network

    /*! The writer for output image */
    cPinWriter m_oWriter;
    /*! The reader for image data*/
    cPinReader m_oReader;
    /*! The classification writer */

    cPinWriter m_oObjectWriter;

    cPinWriter m_oBlueLightWriter;

    adtf::base::ant::runnable<> m_oThreadRunner;

    struct ObjectDetectionDataID
    {
        tSize ui32SizeId;
        tSize tObjectarrayId;
    } m_ObjectDetectionDataIDs;

    struct tBoolSignalValueId
    {
        tSize timeStamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;

    cSampleCodecFactory m_BoolSignalValueSampleFactory;

    struct blobFromImageParams
    {
        adtf::base::property_variable<tFloat> scale;
        adtf::base::property_variable<tFloat> meanR;
        adtf::base::property_variable<tFloat> meanG;
        adtf::base::property_variable<tFloat> meanB;
        adtf::base::property_variable<tBool>  channelSwap;
    };
    blobFromImageParams m_blobParams = { 2.0f/255.0f, 127.5f, 127.5f, 127.5f, tTrue};

    struct roi
    {
        adtf::base::property_variable<tInt32> roiX;
        adtf::base::property_variable<tInt32> roiY;
        adtf::base::property_variable<tInt32> width;
        adtf::base::property_variable<tInt32> height;
    };
    roi m_roiParams = {0, 200, 1080, 550};

    property_variable<tBool> m_useRoi = tTrue;

    /*! The codec factory */
    cSampleCodecFactory m_oCodecFactory;

    /*! The current stream imageformat */
    adtf::streaming::tStreamImageFormat m_sCurrentFormat;

    /*! The caffe model net */
    Net m_oNet;

    /*! List of names of the class */
    std::vector<cString> m_classNames;


    /*! Confidence Threshold */
    adtf::base::property_variable<tFloat32> m_confThr = 0.5;

    /*! The open cl flag */
    property_variable<tBool> m_bOpenCL = tTrue;

    /*! The prototype text property */
    property_variable<cFilename> m_strProtoTxt = cFilename("bvlc_googlenet.prototxt");

    /*! The model property */
    property_variable<cFilename> m_strModel = cFilename("bvlc_googlenet.caffemodel");

    /*! List of names of the class property */
    property_variable<cFilename> m_strClassNames = cFilename("synset_words.txt");

    property_variable<tInt32> m_lowerThreshold = 500;

    property_variable<tInt32> m_upperThreshold = 1000;

    property_variable<tBool> m_showDebugVideo = tTrue;

    property_variable<tBool> m_takePicture = tFalse;

    property_variable<tInt32> m_minHeightOfCar = 0;

    property_variable<tInt32> m_minHeightOfBarbie = 0;

    property_variable<tInt32> m_maxHeightOfBarbie = 1080;

    property_variable<tInt32> m_minHeightOfBaby = 0;

    property_variable<tInt32> m_maxHeightOfBaby = 1080;

    property_variable<tBool> m_showFPS = tTrue;

    //properties blaues licht

    struct tHoughParam
    {
        adtf::base::property_variable<tInt> dp;
        adtf::base::property_variable<tInt> minDist;
        adtf::base::property_variable<tInt> param1;
        adtf::base::property_variable<tInt> param2;
        adtf::base::property_variable<tInt> minRadius;
        adtf::base::property_variable<tInt> maxRadius;
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
       property_variable<tFloat32> w = 0.333;
       tBlurrParam blurr = {7,5};
       tHoughParam hough = {1, 100, 60, 30, 0, 20};
       property_variable<tFloat32> factor = 0.5f;
       property_variable<tFloat32> heightFactor = 0.66;
    } m_Properties;

    std::vector<cString> readClassNames(const char *filename);

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

public:

    /*! Default constructor. */
    cKIObjectDetection();

    /*! Destructor. */
    virtual ~cKIObjectDetection() = default;

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
