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
#include "Lanekeeping.h"
#include "aadc_structs.h"
#include "ADTF3_helper.h"
#include "ADTF3_OpenCV_helper.h"

#define CLAMP(X, MIN, MAX) ((X < MIN) ? MIN : ((X > MAX) ? MAX : X))

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_LANEKEEPING_DATA_TRIGGERED_FILTER,"Lanekeeping",cLanekeeping,adtf::filter::pin_trigger({ "VideoIn", "LaneChangeIn", "useRoiLeft", "useRoiRight" }));

cLanekeeping::cLanekeeping() :
    steeringValue(0.f),
    lastSteeringValue(0.f),
    distanceDriven(0.f),
    targetLane(LANE_RIGHT),
    m_laneSwitchdistanceAtStartSet(tTrue),
    m_laneChangeActive(tFalse),
    m_useRoiLeft(tTrue),
    m_useRoiRigth(tTrue)
{
    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oVideoIn, "VideoIn", pType);
    //Register output pin
    Register(m_oVideoOut, "VideoOut", pType);

    object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData, m_WheelDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataId.ArduinoTimestamp);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataId.WheelTach);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataId.WheelDir);
    }
    else
    {
        LOG_INFO("No mediadescription for tWheelData found!");
    }

    Register(m_oWheelLeftIn, "WheelLeft" , pTypeWheelData);

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
        adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value);
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oSteeringOut, "SteeringOut",pTypeSignalValue);
    Register(m_SetRoiIn, "Set ROI", pTypeSignalValue);
	Register(m_oLastSteering, "SteeringIn", pTypeSignalValue);

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if (IS_OK(create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory)))
    {
        access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.timeStamp);
        access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue);
    }
    else
    {
        LOG_INFO("No mediadescription for tBoolSignalValue found!");
    }
    Register(m_oLaneChangeIn, "LaneChangeIn",pTypeBoolSignalValue);
    Register(m_oUseRoiLeftIn, "useRoiLeft", pTypeBoolSignalValue);
    Register(m_oUseRoiRightIn, "useRoiRight", pTypeBoolSignalValue);
    Register(m_oStopLaneChange, "stopLaneChange", pTypeBoolSignalValue);

    //register callback for type changes
    m_oVideoIn.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oVideoIn, m_sImageFormat, *pType.Get(), m_oVideoOut);
    });

    //setup filter properties
    setupProperties();
}

tResult cLanekeeping::setupProperties()
{
    RegisterPropertyVariable("Show Lanekeeping Video", m_Properties.showLanekeeping);

    RegisterPropertyVariable("ROI Right Offset X", m_Properties.roiRight.offsetX);
    RegisterPropertyVariable("ROI Right Offset Y", m_Properties.roiRight.offsetY);
    RegisterPropertyVariable("ROI Right Width", m_Properties.roiRight.width);
    RegisterPropertyVariable("ROI Right Height", m_Properties.roiRight.height);
    RegisterPropertyVariable("ROI Right Reference Upper X", m_Properties.roiRight.refUpperX);
    RegisterPropertyVariable("ROI Right Reference Lower X", m_Properties.roiRight.refLowerX);
    RegisterPropertyVariable("ROI Right Reference Upper Y", m_Properties.roiRight.refUpperY);
    RegisterPropertyVariable("ROI Right Reference Lower Y", m_Properties.roiRight.refLowerY);
    RegisterPropertyVariable("ROI Right Maximal Dynamic Width", m_Properties.roiRight.maxDynWidth);

    RegisterPropertyVariable("ROI Left Offset X", m_Properties.roiLeft.offsetX);
    RegisterPropertyVariable("ROI Left Offset Y", m_Properties.roiLeft.offsetY);
    RegisterPropertyVariable("ROI Left Width", m_Properties.roiLeft.width);
    RegisterPropertyVariable("ROI Left Height", m_Properties.roiLeft.height);
    RegisterPropertyVariable("ROI Left Reference Upper X", m_Properties.roiLeft.refUpperX);
    RegisterPropertyVariable("ROI Left Reference Lower X", m_Properties.roiLeft.refLowerX);
    RegisterPropertyVariable("ROI Left Reference Upper Y", m_Properties.roiLeft.refUpperY);
    RegisterPropertyVariable("ROI Left Reference Lower Y", m_Properties.roiLeft.refLowerY);
    RegisterPropertyVariable("ROI Left Maximal Dynamic Width", m_Properties.roiLeft.maxDynWidth);

    RegisterPropertyVariable("# Detection Lines", m_Properties.detectionLines);
    RegisterPropertyVariable("Minimal Line Width", m_Properties.minLineWidth);
    RegisterPropertyVariable("Maximal Line Width", m_Properties.maxLineWidth);
    RegisterPropertyVariable("Minimal Line Contrast", m_Properties.minLineContrast);
    RegisterPropertyVariable("Upper Bound", m_Properties.upperBound);
    RegisterPropertyVariable("Dynamic Width Multiplier", m_Properties.dynWidthMultiplier);
    RegisterPropertyVariable("Steering Multiplier", m_Properties.steeringMultiplier);
    RegisterPropertyVariable("Steering Offset", m_Properties.steeringOffset);

    RegisterPropertyVariable("Lane Switch Left to Right Distance", m_Properties.laneSwitchLeftToRight.distance);
    RegisterPropertyVariable("Lane Switch Left to Right Offset", m_Properties.laneSwitchLeftToRight.offset);
    RegisterPropertyVariable("Lane Switch Left to Right Offset ROI", m_Properties.laneSwitchLeftToRight.offsetROI);

    RegisterPropertyVariable("Lane Switch Right to Left Distance", m_Properties.laneSwitchRightToLeft.distance);
    RegisterPropertyVariable("Lane Switch Right to Left Offset", m_Properties.laneSwitchRightToLeft.offset);
    RegisterPropertyVariable("Lane Switch Right to Left Offset ROI", m_Properties.laneSwitchRightToLeft.offsetROI);

    RegisterPropertyVariable("Error Correction Threshold", m_Properties.errorCorrThreshold);
    RegisterPropertyVariable("Error Correction Enabled", m_Properties.errorCorrection);

    RETURN_NOERROR;
}

tResult cLanekeeping::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cLanekeeping::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSteeringSample;
    if (IS_OK(m_oLastSteering.GetNextSample(pSteeringSample)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSteeringSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &lastSteeringValue));
    }

    object_ptr<const ISample> pLaneChangeSample;
    if (IS_OK(m_oLaneChangeIn.GetNextSample(pLaneChangeSample)))
    {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pLaneChangeSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        tBool changeLane;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &changeLane));

        targetLane = changeLane ? LANE_LEFT : LANE_RIGHT;
        m_laneChangeActive = tTrue;
        m_laneSwitchdistanceAtStartSet = tFalse;
    }
    object_ptr<const ISample> pStopLaneChangeSample;
    if(IS_OK(m_oStopLaneChange.GetNextSample(pStopLaneChangeSample)))
    {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pLaneChangeSample);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tBool stop;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &stop));
        if(stop)
        {
            m_laneChangeActive = tFalse;
            m_Properties.roiRight.dynWidth = 0;
            m_Properties.roiLeft.dynWidth = 0;
        }
    }
    object_ptr<const ISample> pSetRoiSample;
    if (IS_OK(m_SetRoiIn.GetNextSample(pSetRoiSample)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSetRoiSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        tFloat32 roi;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &roi));
        m_Properties.roiRight.dynWidth = (tInt)roi;
        m_Properties.roiLeft.dynWidth = (tInt)roi;
    }

    object_ptr<const ISample> pUseRoiLeftSample;
    if (IS_OK(m_oUseRoiLeftIn.GetNextSample(pUseRoiLeftSample)))
    {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pUseRoiLeftSample);

        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &m_useRoiLeft));
    }

    object_ptr<const ISample> pUseRoiRightSample;
    if (IS_OK(m_oUseRoiRightIn.GetNextSample(pUseRoiRightSample)))
    {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pUseRoiRightSample);

        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &m_useRoiRigth));
    }

    if(m_laneChangeActive)
    {
        if(!m_laneSwitchdistanceAtStartSet)
        {
            m_laneSwitchdistanceAtStart = getDistance();
            m_laneSwitchdistanceAtStartSet = m_laneSwitchdistanceAtStart > 0 ? tTrue : tFalse;
        }
        else
        {
            distanceDriven = getDistance()-m_laneSwitchdistanceAtStart;
        }
    }

    object_ptr<const ISample> pReadSample;
    while (IS_OK(m_oVideoIn.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                 CV_8UC3, (uchar*) pReadBuffer->GetPtr());

            CalculateSteering(&inputImage, tmTimeOfTrigger);
        }
    }
    RETURN_NOERROR;
}

std::pair<tFloat32, tInt> cLanekeeping::processROI(const tROI& roiVars, const cv::Mat* image, cv::Mat* laneImage)
{
    tFloat32 cumulatedDifference = 0.f;

    tInt32 x1 = CLAMP(roiVars.offsetX + roiVars.dynWidth, 0, image->cols -1);

    Mat roi = (*image)(Rect(x1, roiVars.offsetY, CLAMP((image->cols - x1), 0, std::min(*roiVars.width, image->cols - 1)), roiVars.height));
    cvtColor(roi, roi, CV_RGB2GRAY);

    std::vector<Point> linePoints = getLinePoints(roiVars, &roi);

    if (m_Properties.errorCorrection)
    {
        discardBadPoints(linePoints);
    }

    for (const auto& p : linePoints)
    {
        tFloat32 lambda = (p.y - roiVars.refUpperY) / (roiVars.refLowerY - roiVars.refUpperY);
        tFloat32 refx = roiVars.refUpperX + lambda * (roiVars.refLowerX - roiVars.refUpperX);
        cumulatedDifference += refx - p.x;
    }

    if (laneImage != nullptr)
    {
        std::lock_guard<std::mutex> lock(m_oLaneImageMutex);

        circle(*laneImage, { roiVars.refUpperX, roiVars.refUpperY }, 7, { 0, 255, 255 }, FILLED);
        circle(*laneImage, { roiVars.refLowerX, roiVars.refLowerY }, 7, { 0, 255, 255 }, FILLED);

        for (const auto& p : linePoints)
        {
            circle(*laneImage, p, 7, { 0, 255, 0 }, FILLED);
        }

        /*
        for (vector<Point>::iterator it = discardedPointsL.begin(); it != discardedPointsL.end(); it++)
        {
            circle(laneImage, *it, 7, Scalar(255, 0, 0), FILLED);
        }
        */

        // show detection ROIs
        rectangle(*laneImage, { roiVars.offsetX + roiVars.dynWidth, roiVars.offsetY }, { roiVars.offsetX + roiVars.dynWidth + roiVars.width, roiVars.offsetY + roiVars.height }, { 255, 255, 0 }, 2);
        // show maximum / minimum line width
        rectangle(*laneImage, { roiVars.offsetX, roiVars.offsetY + roiVars.height }, { roiVars.offsetX + m_Properties.maxLineWidth, roiVars.offsetY + roiVars.height + 10 }, { 255, 0, 255 }, 2);
        rectangle(*laneImage, { roiVars.offsetX, roiVars.offsetY + roiVars.height + 10 }, { roiVars.offsetX + m_Properties.minLineWidth, roiVars.offsetY + roiVars.height + 20 }, { 255, 0, 255 }, 2);
    }

    return { cumulatedDifference, linePoints.size() };
}

tVoid cLanekeeping::discardBadPoints(std::vector<Point>& points)
{
    if (points.size() < 3)
    {
        return;
    }

    std::vector<tInt> badIdx;
    for (tInt i = points.size() - 2; i > 0; i--)
    {
        if (std::abs(points[i].x - points[i - 1].x) > m_Properties.errorCorrThreshold
                || std::abs(points[i].x - points[i + 1].x) > m_Properties.errorCorrThreshold)
        {
            badIdx.push_back(i);
        }
    }

    for (tInt idx : badIdx)
    {
        points.erase(points.begin() + idx);
    }
    points.erase(points.begin());
    points.erase(points.end() - 1);
}

std::vector<Point> cLanekeeping::getLinePoints(const tROI& roiVars, const cv::Mat* image)
{
    std::vector<Point> linePoints;

    //tInt lineDistance = roiVars.height / (m_Properties.detectionLines + 1);
    tInt roiEnd = std::min(*roiVars.width, image->cols);

    for (tInt nline = 0; nline < std::min(*roiVars.height, image->rows); nline += 1 /*lineDistance*/)
    {
        const uchar* linePtr = image->ptr<uchar>(nline, 0);

        tInt startIdx = 1;
        for (; startIdx < roiEnd; startIdx++)
        {
            if (linePtr[startIdx] - linePtr[startIdx - 1] > m_Properties.minLineContrast)
            {
                break;
            }
        }

        if (roiEnd - startIdx < m_Properties.minLineWidth)
        {
            continue;
        }

        tInt endIdx = startIdx + 1;
        for (; endIdx < roiEnd; endIdx++)
        {
            if (linePtr[endIdx - 1] - linePtr[endIdx] > m_Properties.minLineContrast)
            {
                break;
            }
        }

        tInt lineWidth = endIdx - startIdx;
        if (lineWidth >= m_Properties.minLineWidth && lineWidth <= m_Properties.maxLineWidth)
        {
            linePoints.push_back({ startIdx + lineWidth / 2 + roiVars.offsetX + roiVars.dynWidth, nline + roiVars.offsetY });
        }
    }

    return linePoints;
}

tResult cLanekeeping::CalculateSteering(const cv::Mat* image, tTimeStamp inputTimestamp)
{
    cv::Mat* laneImage = nullptr;
    if (m_Properties.showLanekeeping)
    {
        laneImage = new cv::Mat(*image);
    }

    std::pair<tFloat32, tInt> diffRight = { 0, 0 };
    std::pair<tFloat32, tInt> diffLeft = { 0, 0 };

    if (m_useRoiRigth)
    {
        diffRight = processROI(m_Properties.roiRight, image, laneImage);
    }

    if (m_useRoiLeft)
    {
        diffLeft = processROI(m_Properties.roiLeft, image, laneImage);
    }

    tFloat32 cumulatedDifference;
    if (diffRight.second == 0 && diffLeft.second == 0)
    {
        cumulatedDifference = lastSteeringValue;
    }
    else
    {
        cumulatedDifference = (diffRight.first + diffLeft.first) / (diffRight.second + diffLeft.second);
    }

    steeringValue = CLAMP(-m_Properties.steeringMultiplier * cumulatedDifference / m_Properties.upperBound * 50, -100.f, 100.f);

    m_Properties.roiRight.dynWidth = CLAMP(m_Properties.dynWidthMultiplier * steeringValue, -m_Properties.roiRight.maxDynWidth, *m_Properties.roiRight.maxDynWidth);
    m_Properties.roiLeft.dynWidth = CLAMP(m_Properties.dynWidthMultiplier * steeringValue, -m_Properties.roiLeft.maxDynWidth, *m_Properties.roiRight.maxDynWidth); // intended!

    // ?
    //steeringValue += m_Properties.steeringOffset;
    //steeringValue = abs(steeringValue + m_Properties.steeringOffset) > 100 ? steeringValue : steeringValue + m_Properties.steeringOffset;
    steeringValue = CLAMP(steeringValue + m_Properties.steeringOffset, -100.f, 100.f);

    {
        std::lock_guard<std::mutex> lock(m_oLaneChangeMutex);
        if (m_laneChangeActive)
        {
            switchLane();
        }
    }

    TransmitSteering(steeringValue, inputTimestamp);

    if (m_Properties.showLanekeeping)
    {
        //update output format if matrix size does not fit to
        if (laneImage->total() * laneImage->elemSize() != m_sImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oVideoOut, *laneImage);
        }
        // write to pin
        writeMatToPin(m_oVideoOut, *laneImage, m_pClock->GetStreamTime());

        delete laneImage;
    }
    RETURN_NOERROR;
}

tVoid cLanekeeping::switchLane()
{
    if (targetLane == LANE_LEFT)
    {
        switchLane(m_Properties.laneSwitchRightToLeft);
    }
    else if (targetLane == LANE_RIGHT)
    {
        switchLane(m_Properties.laneSwitchLeftToRight);
    }
}

tVoid cLanekeeping::switchLane(const tLaneSwitch& laneSwitch)
{
    if (distanceDriven < laneSwitch.distance / 2.f)
    {
        steeringValue -= laneSwitch.offset * 2.f * distanceDriven / laneSwitch.distance;
    }
    else if (distanceDriven < laneSwitch.distance)
    {
        steeringValue -= laneSwitch.offset;
    }
    else
    {
        steeringValue -= laneSwitch.offset;

        m_Properties.roiRight.dynWidth = laneSwitch.offsetROI;
        m_Properties.roiLeft.dynWidth = -laneSwitch.offsetROI;
        distanceDriven = 0.f;
        m_laneChangeActive = tFalse;
    }
}

tResult cLanekeeping::TransmitSteering(tFloat32 steeringValue, tTimeStamp timestamp)
{
    object_ptr<ISample> pWriteSteeringSample;
    if (IS_OK(alloc_sample(pWriteSteeringSample)))
    {
        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSteeringSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, timestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, steeringValue));
    }
    m_oSteeringOut << pWriteSteeringSample << flush;
    RETURN_NOERROR;
}


tFloat32 cLanekeeping::getDistance()
{
    tFloat32 valueM = -1.f;
    object_ptr<const ISample> pReadSample;

    if (IS_OK(m_oWheelLeftIn.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pReadSample);

        tUInt32 valueI;

        oDecoder.GetElementValue(m_ddlWheelDataId.WheelTach, &valueI);

        valueM = ConvertWheelToMeter(valueI);
    }

    return valueM;
}

tFloat32 cLanekeeping::ConvertWheelToMeter(tUInt32 value)
{
    tFloat32 result = (value/60.0f)*0.34f;
    return result;
}
