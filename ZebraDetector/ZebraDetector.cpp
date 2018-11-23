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
#include "ZebraDetector.h"
#include "aadc_structs.h"
#include "ADTF3_helper.h"
#include "ADTF3_OpenCV_helper.h"

#define CLAMP(X, MIN, MAX) ((X < MIN) ? MIN : ((X > MAX) ? MAX : X))

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ZebraDETECTOR_DATA_TRIGGERED_FILTER,"ZebraDetector",cZebraDetector,adtf::filter::pin_trigger({ "VideoIn", "Activate" }));

cZebraDetector::cZebraDetector() :
    m_SearchActive(tFalse)
{



    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oVideoIn, "VideoIn", pType);
    //Register output pin
    Register(m_oVideoOut, "VideoOut", pType);

    object_ptr<IStreamType> pTypeZebraSituation;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tZebraSituation", pTypeZebraSituation, m_ZebraSituationSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_ZebraSituationSampleFactory, cString("Situation"), m_ddlZebraSituationId.situation);
        adtf_ddl::access_element::find_index(m_ZebraSituationSampleFactory, cString("Distance"), m_ddlZebraSituationId.distance);
    }
    else
    {
        LOG_INFO("No mediadescription for tZebraSituation found!");
    }
    Register(m_oSituationOut, "Situation",pTypeZebraSituation);


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
     Register(m_oActivateIn, "Activate",pTypeBoolSignalValue);

    //register callback for type changes
    m_oVideoIn.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oVideoIn, m_sImageFormat, *pType.Get(), m_oVideoOut);
    });

    //setup filter properties
    setupProperties();
}

tResult cZebraDetector::setupProperties()
{
    RegisterPropertyVariable("Show DebugVideo", m_Properties.showDebug);
    RegisterPropertyVariable("ROI Offset X", m_Properties.roi.offsetX);
    RegisterPropertyVariable("ROI Offset Y", m_Properties.roi.offsetY);
    RegisterPropertyVariable("ROI Width", m_Properties.roi.width);
    RegisterPropertyVariable("ROI Height", m_Properties.roi.height);

    RegisterPropertyVariable("Line Min Width", m_Properties.lineWidth.minWidth);
    RegisterPropertyVariable("Line Max Width", m_Properties.lineWidth.maxWidth);


    RegisterPropertyVariable("binaryThreshold", m_Properties.binaryThreshold);
    RegisterPropertyVariable("Number of Lines", m_Properties.numLines);
    RegisterPropertyVariable("Counts Per Lines", m_Properties.lineCount);
    RegisterPropertyVariable("Counts Tol", m_Properties.lineCountTol);


    RETURN_NOERROR;
}

tResult cZebraDetector::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cZebraDetector::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pActivateSample;
    if (IS_OK(m_oActivateIn.GetNextSample(pActivateSample)))
    {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pActivateSample);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tBool active = tFalse;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &active));
        if (active) m_SearchActive = tTrue;

    }

    object_ptr<const ISample> pReadSample;
    //while (IS_OK(m_oVideoIn.GetNextSample(pReadSample)))
    if (IS_OK(m_oVideoIn.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                 CV_8UC3, (uchar*) pReadBuffer->GetPtr());

            if (m_SearchActive)
            {
                searchForZebra(&inputImage);
            }
        }
    }
    RETURN_NOERROR;


}

tInt cZebraDetector::lineCounter(const uchar* linePtr, const int cols)
{
    tInt c = 0;
    tBool white = tFalse;
    tInt Start = 0, Len = 0;

    for(int j = 0; j < cols; j++)
    {
      if (linePtr[j] >= 200 and white == tFalse)
      {
        white = tTrue;
        Start = j;
      }

      if(linePtr[j] <= 100 and white == tTrue)
      {
        white = tFalse;
        Len = j - Start;
        if (Len >= m_Properties.lineWidth.minWidth and Len <= m_Properties.lineWidth.maxWidth)
        {
          c++;
        }
      }
     }
     return c;
}


tBool cZebraDetector::processROI(const tROI& roiVars, const cv::Mat* image, cv::Mat* ZebraImage)
{
    Mat roi = (*image)(Rect(roiVars.offsetX, roiVars.offsetY, roiVars.width, roiVars.height));
    cvtColor(roi, roi, CV_RGB2GRAY);
    threshold(roi, roi,m_Properties.binaryThreshold,255, CV_THRESH_TOZERO);

    tInt lineDistance = ceil(1.f/(m_Properties.numLines+1)*m_Properties.roi.height);

    vector<tInt> counts;



    for (tInt nline = lineDistance; nline <= lineDistance * m_Properties.numLines; nline += lineDistance)
      {
        const uchar* linePtr = roi.ptr<uchar>(nline, 0);

        tInt c = lineCounter(linePtr, roi.cols);
        counts.push_back(c);
      }

      // This block of code may not be necessary,

//      vector<tInt>::iterator min;
//      min = std::min_element(counts.begin(), counts.end());

//      vector<tInt>::iterator max;
//      max = std::max_element(counts.begin(), counts.end());

//      counts.erase(min);
//      counts.erase(max);

      //


      tBool DetectionGood =  std::equal(counts.begin() + 1, counts.end(), counts.begin()); // room for improvement here

      tBool Zebrafound = false;



      if(DetectionGood and abs(counts[0] - m_Properties.lineCount) <= m_Properties.lineCountTol )
      {
        Zebrafound = true;

      }







      if (m_Properties.showDebug)
    {
        LOG_INFO(cString::Format("Count %d:", counts[0]));
        LOG_INFO(cString::Format("DetectionGood %d:", DetectionGood));
        rectangle(*ZebraImage, { roiVars.offsetX, roiVars.offsetY }, { roiVars.offsetX+ roiVars.width, roiVars.offsetY + roiVars.height }, { 255, 255, 0 }, 2);
        cvtColor(roi, roi, CV_GRAY2RGB);


        for (tInt nline = lineDistance; nline <= lineDistance * m_Properties.numLines; nline += lineDistance)
          {
           Point pt1(0,nline);
           Point pt2(roi.cols,nline);
           line(roi, pt1, pt2, {255,0,0}, 2);
          }
        roi.copyTo((*ZebraImage)(Rect(roiVars.offsetX, roiVars.offsetY, roiVars.width, roiVars.height)));

    }
    return Zebrafound;
}


tResult cZebraDetector::searchForZebra (const Mat *image)
{
    cv::Mat* ZebraImage = nullptr;
    if (m_Properties.showDebug)
    {
        ZebraImage = new cv::Mat(*image);
    }

    tBool Zebrafound{tFalse};

    Zebrafound = processROI(m_Properties.roi, image, ZebraImage);
    if (Zebrafound)
    {
        if (m_Properties.showDebug)
        {
            LOG_INFO(cString("Zebra found!"));
        }
        tZebraSituation value = {tTrue,0.0f};
        TransmitZebraSituation(value);
    }
    else
    {
//        if (m_Properties.showDebug)
//        {
//            LOG_INFO(cString("Zebra not found!"));
//        }
        tZebraSituation value = {tFalse,0.0f};
        TransmitZebraSituation(value);
    }

    if (m_Properties.showDebug && Zebrafound)
    {

        //update output format if matrix size does not fit to
        if (ZebraImage->total() * ZebraImage->elemSize() != m_sImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oVideoOut, *ZebraImage);
        }
        // write to pin
        writeMatToPin(m_oVideoOut, *ZebraImage, m_pClock->GetStreamTime());
        delete ZebraImage;

    }
    RETURN_NOERROR;
}

tResult cZebraDetector::TransmitZebraSituation(tZebraSituation ZebraSituation)
{
    object_ptr<ISample> pWriteZebraSample;    
    if (IS_OK(alloc_sample(pWriteZebraSample)))
    {
        auto oCodec = m_ZebraSituationSampleFactory.MakeCodecFor(pWriteZebraSample);        
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlZebraSituationId.situation, ZebraSituation.Situation));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlZebraSituationId.distance, ZebraSituation.Distance));
         LOG_INFO(cString("Situation initialized"));
    }
    m_oSituationOut << pWriteZebraSample << flush;// << trigger;
    RETURN_NOERROR;
}
