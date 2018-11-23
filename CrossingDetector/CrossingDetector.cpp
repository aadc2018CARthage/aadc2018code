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
#include <vector>
#include <future>
#include <algorithm>
#include "CrossingDetector.h"
#include "aadc_structs.h"
#include "ADTF3_helper.h"
#include "ADTF3_OpenCV_helper.h"

#define CLAMP(X, MIN, MAX) ((X < MIN) ? MIN : ((X > MAX) ? MAX : X))

ADTF_PLUGIN(LABEL_CROSSING_DETECTOR, cCrossingDetector)

cCrossingDetector::cCrossingDetector() :
    m_Steering(0.0f),
    m_Speed(0.0f),
    m_CrossingAngle(0.0f),
    m_CrossingRadius(40)
{
    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    filter_create_pin(*this, m_oVideoIn, "VideoIn", pType);
    m_oVideo_runner = runnable<IRunnable::RUN_TRIGGER >(ADTF_RUN_FUNCTION(Process));
    RegisterRunner("video_runner", m_oVideo_runner);
    ConfigureDataInTrigger("video_runner", "VideoIn");
    //Register output pin
    filter_create_pin(*this, m_oVideoOut, "VideoOut", pType);

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.f32Value);
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    filter_create_pin(*this, m_SteeringIn, "Steering", pTypeSignalValue);
    filter_create_pin(*this, m_SpeedIn, "Speed", pTypeSignalValue);



    object_ptr<IStreamType> pTypeCrossingSituation;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tCrossingSituation", pTypeCrossingSituation, m_CrossingSituationSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_CrossingSituationSampleFactory, cString("situation"), m_ddlCrossingSituationId.situation);
        adtf_ddl::access_element::find_index(m_CrossingSituationSampleFactory, cString("distanceToCrossing"), m_ddlCrossingSituationId.distance);
    }
    else
    {
        LOG_INFO("No mediadescription for tCrossingSituation found!");
    }
    filter_create_pin(*this, m_oSituationOut, "Situation", pTypeCrossingSituation);

    object_ptr<IStreamType> pTypeIntSignalValue;
    if(IS_OK(create_adtf_default_stream_type_from_service("tIntSignalValue", pTypeIntSignalValue, m_IntSignalValueSampleFactory)))
    {
        access_element::find_index(m_IntSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlIntSignalValueId.timeStamp);
        access_element::find_index(m_IntSignalValueSampleFactory, cString("value"), m_ddlIntSignalValueId.value);
    }
    else
    {
        LOG_INFO("No mediadescription for tIntSignalValue found!");
    }
    filter_create_pin(*this, m_oActivateIn,"Activate",pTypeIntSignalValue);

    //register callback for type changes
    m_oVideoIn.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oVideoIn, m_sImageFormat, *pType.Get(), m_oVideoOut);
    });

    //setup filter properties
    setupProperties();
}

tResult cCrossingDetector::setupProperties()
{
    RegisterPropertyVariable("Show DebugVideo", m_Properties.showDebug);
    RegisterPropertyVariable("ROI Right Offset X", m_Properties.roiRight.offsetX);
    RegisterPropertyVariable("ROI Right Offset Y", m_Properties.roiRight.offsetY);
    RegisterPropertyVariable("ROI Right Width", m_Properties.roiRight.width);
    RegisterPropertyVariable("ROI Right Height", m_Properties.roiRight.height);

    RegisterPropertyVariable("ROI Left Offset X", m_Properties.roiLeft.offsetX);
    RegisterPropertyVariable("ROI Left Offset Y", m_Properties.roiLeft.offsetY);
    RegisterPropertyVariable("ROI Left Width", m_Properties.roiLeft.width);
    RegisterPropertyVariable("ROI Left Height", m_Properties.roiLeft.height);

    RegisterPropertyVariable("ROI Corner Offset X", m_Properties.cornerROI.offsetX);
    RegisterPropertyVariable("ROI Corner Offset Y", m_Properties.cornerROI.offsetY);
    RegisterPropertyVariable("ROI Corner Width", m_Properties.cornerROI.width);
    RegisterPropertyVariable("ROI Corner Height", m_Properties.cornerROI.height);

    RegisterPropertyVariable("Corner Quality Measure", m_Properties.CornerParam.qualityMeasure);
    RegisterPropertyVariable("Corner Max Distance", m_Properties.CornerParam.maxDistance);
    RegisterPropertyVariable("Corner Number of Corners", m_Properties.CornerParam.numberOfCorners);
    RegisterPropertyVariable("Corner Y Factor", m_Properties.CornerParam.yCorrFactor);
    RegisterPropertyVariable("Corner X Factor", m_Properties.CornerParam.xCorrFactor);
    RegisterPropertyVariable("Corner ROI Offset Factor", m_Properties.CornerParam.offsetFactor);
    RegisterPropertyVariable("Corner ROI Max Offset", m_Properties.CornerParam.maxOffset);
    RegisterPropertyVariable("Corner High Crossing DIstance", m_Properties.CornerParam.highDistance);
    RegisterPropertyVariable("Corner Low Crossing DIstance", m_Properties.CornerParam.lowDistance);

    RegisterPropertyVariable("Canny Threshold 1", m_Properties.CannyParam.thresholdOne);
    RegisterPropertyVariable("Canny Threshold 2", m_Properties.CannyParam.thresholdTwo);
    RegisterPropertyVariable("Hough Accumulator Threshold", m_Properties.roiLeft.houghAccThreshold);
    RegisterPropertyVariable("Hough Accumulator Threshold R", m_Properties.roiRight.houghAccThreshold);
    RegisterPropertyVariable("Hough Accumulator Threshold P", m_Properties.roiRight.houghAccThresholdParking);

    RegisterPropertyVariable("Crossing Angle Tolerance", m_Properties.CrossingAngleTolerance);

    RegisterPropertyVariable("Steering Deadband", m_Properties.steeringDeadband);
    RegisterPropertyVariable("Curve Factor", m_Properties.curveFactor);
    RegisterPropertyVariable("Curve Constant", m_Properties.curveConstant);
    RETURN_NOERROR;
}

tResult cCrossingDetector::Init(tInitStage eStage)
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cCrossingDetector::Shutdown(cFilterLevelmachine::tInitStage eStage)
{
    RETURN_NOERROR;
}

tResult cCrossingDetector::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oVideoIn.GetNextSample(pReadSample)))
    {
        object_ptr<const ISample> pActivateSample;
        if (IS_OK(m_oActivateIn.GetNextSample(pActivateSample)))
        {
            auto oDecoder = m_IntSignalValueSampleFactory.MakeDecoderFor(*pActivateSample);

            RETURN_IF_FAILED(oDecoder.IsValid());
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlIntSignalValueId.value, &m_active));
        }

        if (m_active)
        {
            object_ptr<const ISample> pSteeringSample;
            if (IS_OK(m_SteeringIn.GetLastSample(pSteeringSample)))
            {
                auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSteeringSample);

                RETURN_IF_FAILED(oDecoder.IsValid());
                RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.f32Value, &m_Steering));
            }
            object_ptr<const ISample> pSpeedSample;
            if (IS_OK(m_SpeedIn.GetLastSample(pSpeedSample)))
            {
                auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSpeedSample);

                RETURN_IF_FAILED(oDecoder.IsValid());
                RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.f32Value, &m_Speed));
            }

            object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
            //lock read buffer
            if (IS_OK(pReadSample->Lock(pReadBuffer)))
            {
                //create a opencv matrix from the media sample buffer
                Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                     CV_8UC3, (uchar*) pReadBuffer->GetPtr());

                searchForCrossing(&inputImage);
            }
            //m_oVideoIn.Clear();
        }
    }
    RETURN_NOERROR;
}

tBool cCrossingDetector::processROI(const tROI& roiVars, const cv::Mat* image, cv::Mat* crossingImage)
{
    vector<Vec2f> CrossingLines;
    Mat roi = (*image)(Rect(roiVars.offsetX, roiVars.offsetY, roiVars.width, roiVars.height));
    cvtColor(roi, roi, CV_RGB2GRAY);
    Canny(roi, roi,m_Properties.CannyParam.thresholdOne, m_Properties.CannyParam.thresholdTwo, 3, tFalse);
    tInt threshold;
    switch (m_active)
    {
    case 1: //Crossings
        threshold = roiVars.houghAccThreshold;
        break;
    case 2: //Parking
        threshold = roiVars.houghAccThresholdParking;
        break;
    default:
        return tFalse;
    }
    HoughLines(roi, CrossingLines, 1, CV_PI/180, threshold);
    float rho, theta;
    tBool Crossingfound = tFalse;
    for( size_t i = 0; i < CrossingLines.size(); i++ )
    {
        rho = CrossingLines[i][0], theta = CrossingLines[i][1];
        //LOG_INFO(cString::Format("WInkel: %f", theta));
        //LOG_INFO(cString::Format("Current angle %f:", theta));
        if ((CV_PI/2 - theta < m_Properties.CrossingAngleTolerance) || theta > CV_PI/2 || rho < 0)
        {
            if (m_Properties.showDebug)
            {
                //LOG_INFO(cString::Format("CrossingFound with angle %f:", theta));
                // rectangle(*crossingImage, { roiVars.offsetX, roiVars.offsetY }, { roiVars.offsetX+ roiVars.width, roiVars.offsetY + roiVars.height }, { 255, 255, 0 }, 2);
                Point pt1, pt2;
                double a = cos(theta), b = sin(theta);
                double x0 = a*rho, y0 = b*rho;
                pt1.x = cvRound(x0 + roiVars.offsetX + 1000*(-b));
                pt1.y = cvRound(y0 + roiVars.offsetY + 1000*(a));
                pt2.x = cvRound(x0 + roiVars.offsetX - 1000*(-b));
                pt2.y = cvRound(y0 + roiVars.offsetY - 1000*(a));
                //line(*crossingImage, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
            }
            Crossingfound = tTrue;
            m_CrossingAngle = theta;
            m_CrossingRadius = rho;
            break;
        }

    }
    if (m_Properties.showDebug)
    {
        //LOG_INFO(cString::Format("CrossingFound with angle %f:", theta));
        rectangle(*crossingImage, { roiVars.offsetX, roiVars.offsetY }, { roiVars.offsetX+ roiVars.width, roiVars.offsetY + roiVars.height }, { 255, 255, 0 }, 2);
        cvtColor(roi, roi, CV_GRAY2RGB);
        roi.copyTo((*crossingImage)(Rect(roiVars.offsetX, roiVars.offsetY, roiVars.width, roiVars.height)));
    }
    return Crossingfound;
}

tResult cCrossingDetector::searchForCrossing(const Mat *image)
{
    cv::Mat* crossingImage = nullptr;
    tFloat32 distance = 0.0f;
    if (m_Properties.showDebug)
    {
        crossingImage = new cv::Mat(*image);
    }

    tBool leftPossible{tFalse}, rightPossible{tFalse};

    rightPossible = processROI(m_Properties.roiRight, image, crossingImage);
    leftPossible = processROI(m_Properties.roiLeft, image, crossingImage);

    tInt32 situation = 0;

    if (rightPossible)
    {
        situation += 2;
        distance = 0.4*(1.0f + m_Properties.CornerParam.yCorrFactor*(m_CrossingAngle- CV_PI/2.0f) + m_Properties.CornerParam.xCorrFactor*m_CrossingRadius);
        if (fabs(m_Steering) > m_Properties.steeringDeadband) distance = 0.0f;
        //LOG_INFO(cString::Format("Distance to crossing: %f",distance));
        if (m_Properties.showDebug)
        {
            LOG_INFO(cString("Crossing Right found!"));
        }
    }
    if (leftPossible && (fabs(m_Steering) < 20.0f))
    {
        situation += 1;
        distance = 0.1f;
        if (m_Properties.showDebug)
        {
            LOG_INFO(cString("Crossing Left found!"));
        }
    }

    if (situation != 0)
    {
        TransmitCrossingSituation({situation, distance});
    }
    // distance =  getDistance(m_Properties.cornerROI, image, crossingImage);

    if (m_Properties.showDebug)
    {

        //update output format if matrix size does not fit to
        if (crossingImage->total() * crossingImage->elemSize() != m_sImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oVideoOut, *crossingImage);
        }
        // write to pin
        writeMatToPin(m_oVideoOut, *crossingImage, m_pClock->GetStreamTime());
        delete crossingImage;

    }
    RETURN_NOERROR;
}

tResult cCrossingDetector::TransmitCrossingSituation(tCrossingSituation Situation)
{
    object_ptr<ISample> pWriteCrossingSample;
    if (IS_OK(alloc_sample(pWriteCrossingSample)))
    {
        auto oCodec = m_CrossingSituationSampleFactory.MakeCodecFor(pWriteCrossingSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlCrossingSituationId.situation, Situation.situation));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlCrossingSituationId.distance, Situation.distance));
        m_oSituationOut << pWriteCrossingSample << flush << trigger;
    }

    RETURN_NOERROR;
}

tFloat32 cCrossingDetector::getDistance(const tROI& roiVars, const cv::Mat* image, cv::Mat* crossingImage)
{
    vector<Point2f> corners;
    tInt Offset = CLAMP(tInt(m_Properties.CornerParam.offsetFactor*m_Steering), tInt(-m_Properties.CornerParam.maxOffset), tInt(m_Properties.CornerParam.maxOffset));
    Mat roi = (*image)(Rect(roiVars.offsetX + Offset, roiVars.offsetY, roiVars.width, roiVars.height));
    if (m_Properties.showDebug) rectangle(*crossingImage, { roiVars.offsetX + Offset, roiVars.offsetY }, { roiVars.offsetX + Offset + roiVars.width, roiVars.offsetY + roiVars.height }, { 255, 0, 255 }, 4);

    cvtColor(roi, roi, CV_RGB2GRAY);
    Mat mask;
    tFloat32 distance = 0;
    //mask =Scalar(0);
    //mask(mask,RoiImage) = Scalar(1);
    //tInt cornerX = 0;
    goodFeaturesToTrack(roi, corners, m_Properties.CornerParam.numberOfCorners, m_Properties.CornerParam.qualityMeasure, m_Properties.CornerParam.maxDistance, mask, 3, false, 0.04);


    if (!corners.empty())
    {
        tInt lowestY = corners.begin()->y; // lowestY ist der im Bild "höchste!" Eckenwert.
        for (vector<Point2f>::iterator cornerIt = corners.begin(); cornerIt != corners.end(); cornerIt++)
        {
            if (cornerIt->y < lowestY)
            {
                lowestY = cornerIt->y;
                //cornerX = corners.at(i).x;
            }
            cornerIt->x += m_Properties.cornerROI.offsetX + Offset;
            cornerIt->y += m_Properties.cornerROI.offsetY;
            //cout << "(" << corners.at(i).x << "," << corners.at(i).y << ")" << endl;
            if (m_Properties.showDebug) circle(*crossingImage,*cornerIt,3, Scalar(255,255,0),5);

        }
        distance = m_Properties.CornerParam.lowDistance + (1 - lowestY/(tFloat32)m_Properties.cornerROI.height)*(m_Properties.CornerParam.highDistance - m_Properties.CornerParam.lowDistance);
        distance -= m_Properties.CornerParam.yCorrFactor*m_Speed - m_Properties.CornerParam.xCorrFactor*(m_Steering);
        LOG_INFO(cString::Format("Distance to Crossing: %f", distance));
        //TransmitStopline(tTrue,GetTime());


    }
    return distance;
}



