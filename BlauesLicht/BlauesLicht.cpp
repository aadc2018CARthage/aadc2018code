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
#include "BlauesLicht.h"
#include "ADTF3_helper.h"
#include "ADTF3_OpenCV_helper.h"



#define CLAMP(X, MIN, MAX) ((X < MIN) ? MIN : ((X > MAX) ? MAX : X))

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_BlauesLicht_DATA_TRIGGERED_FILTER,"BlauesLicht",cBlauesLicht,adtf::filter::pin_trigger({ "VideoIn", "Activate" }));

cBlauesLicht::cBlauesLicht() :
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
    Register(m_oLastVideo, "LastVideo", pType);
    Register(m_oVideoOutHough, "VideoOutHough", pType);



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

    Register(m_oBoolOut, "BlauLichtBool", pTypeBoolSignalValue);



    //register callback for type changes
    m_oVideoIn.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        RETURN_IF_FAILED(ChangeType(m_oVideoIn, m_sImageFormat, *pType.Get(), m_oVideoOut));
        RETURN_IF_FAILED(ChangeType(m_oVideoIn, m_sImageFormat, *pType.Get(), m_oLastVideo));
        return ChangeType(m_oVideoIn, m_sImageFormat, *pType.Get(), m_oVideoOutHough);
    });

    //setup filter properties
    setupProperties();
}

tResult cBlauesLicht::setupProperties()
{
    RegisterPropertyVariable("Show DebugVideo", m_Properties.showDebug);
    RegisterPropertyVariable("Use Front Camera", m_Properties.FrontCamera); // true = front camera; false = rear camera


    RegisterPropertyVariable("ROI Offset X", m_Properties.roi.offsetX);
    RegisterPropertyVariable("ROI Offset Y", m_Properties.roi.offsetY);
    RegisterPropertyVariable("ROI Width", m_Properties.roi.width);
    RegisterPropertyVariable("ROI Height", m_Properties.roi.height);

    RegisterPropertyVariable("minus R and G", m_Properties.w);

    RegisterPropertyVariable("GaussianBlur KernelSize", m_Properties.blurr.kSize);
    RegisterPropertyVariable("GaussianBlur sigma", m_Properties.blurr.sigma);

    RegisterPropertyVariable("Hough: dp", m_Properties.hough.dp);
    RegisterPropertyVariable("Hough: minDistance", m_Properties.hough.minDist);
    RegisterPropertyVariable("Hough: Param1", m_Properties.hough.param1);
    RegisterPropertyVariable("Hough: Param2", m_Properties.hough.param2);
    RegisterPropertyVariable("Hough: minRadius", m_Properties.hough.minRadius);
    RegisterPropertyVariable("Hough: maxRadius", m_Properties.hough.maxRadius);

    RegisterPropertyVariable("Cnt: Canny Thr 1",m_Properties.Cnt.CannyThr1);
    RegisterPropertyVariable("Cnt: Canny Thr 2",m_Properties.Cnt.CannyThr2);
    RegisterPropertyVariable("Cnt: Max Area Deviation",m_Properties.Cnt.maxAreaDev);
    RegisterPropertyVariable("Cnt: Min area of circle",m_Properties.Cnt.minArea);
    RegisterPropertyVariable("Cnt: Max Area Deviation",m_Properties.Cnt.maxAreaDev);
    RegisterPropertyVariable("Cnt: Max Peri error", m_Properties.Cnt.periError);

    RegisterPropertyVariable("bilateralFilter sigmaColor", m_Properties.Cnt.sigmaColor);


    RETURN_NOERROR;
}

tResult cBlauesLicht::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cBlauesLicht::Process(tTimeStamp tmTimeOfTrigger)
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
            //imwrite("/home/aadc/Desktop/BlauLicht.png", inputImage);

            if (m_SearchActive)
            {
                searchForBlauesLicht(&inputImage);
            }
        }
    }
    RETURN_NOERROR;


}



tBool cBlauesLicht::blauesLichtFinderHo(const tROI& roiVars, const cv::Mat* image, cv::Mat* ShowImage)
{
    Mat roi;
    ((*image)(Rect(roiVars.offsetX, roiVars.offsetY, roiVars.width, roiVars.height))).copyTo(roi);

    vector<Vec3f> circles;

    tInt red, blue;
    if(m_Properties.FrontCamera)
    {
        red = 0;
        blue = 2;
    }
    else //rear cam
    {
        blue = 0;
        red = 2;
    }


    Mat rgb[3];  //bgr if rearcam
    split(roi, rgb);

    addWeighted(rgb[blue], 1.0, rgb[red], m_Properties.w, 0, rgb[blue]);
    addWeighted(rgb[blue], 1.0, rgb[1], m_Properties.w, 0, rgb[blue]);


    Mat Mblue;
//    bilateralFilter(rgb[blue], Mblue, 9,75,75);

//    Mat morph;
//    Mat kernel = getStructuringElement(MORPH_RECT,Size{5,5});
//    morphologyEx(Mblue,morph,MORPH_OPEN,kernel,Point{-1,-1},15);
//    subtract(Mblue,morph,Mblue);

    Size kSize(m_Properties.blurr.kSize, m_Properties.blurr.kSize);
    GaussianBlur(rgb[blue], Mblue, kSize, m_Properties.blurr.sigma);


    HoughCircles(Mblue, circles, CV_HOUGH_GRADIENT,
                 m_Properties.hough.dp, m_Properties.hough.minDist,
                 m_Properties.hough.param1, m_Properties.hough.param2,
                 m_Properties.hough.minRadius, m_Properties.hough.maxRadius);





    tBool BlauesLichtFound = false;

    if(!circles.empty()) BlauesLichtFound = true;


    if (m_Properties.showDebug)
    {
        rectangle(*ShowImage, { roiVars.offsetX, roiVars.offsetY }, { roiVars.offsetX+ roiVars.width, roiVars.offsetY + roiVars.height }, { 255, 255, 0 }, 2);
        cvtColor(Mblue, Mblue, CV_GRAY2RGB);


        for( size_t i = 0; i < circles.size(); i++ )
        {
            Vec3i c = circles[i];
            Point center = Point(c[0], c[1]);
            // circle center
            circle( Mblue, center, 1, Scalar(0,0,0), 3, LINE_AA);
            // circle outline
            int radius = c[2];
            circle( Mblue, center, radius + 1, Scalar(0,0,0), 3, LINE_AA);
            LOG_INFO(cString::Format("Radius: %d:",radius));
        }

        (Mblue).copyTo((*ShowImage)(Rect(roiVars.offsetX, roiVars.offsetY, roiVars.width, roiVars.height)));

    }
    return BlauesLichtFound;
}



tBool cBlauesLicht::blauesLichtFinderCnt(const tROI& roiVars, const cv::Mat* image, cv::Mat* ShowImage)
{
    Mat roi;
    ((*image)(Rect(roiVars.offsetX, roiVars.offsetY, roiVars.width, roiVars.height))).copyTo(roi);

    //vector<Vec3f> circles;

    tInt red, blue;
    if(m_Properties.FrontCamera)
    {
        red = 0;
        blue = 2;
    }
    else //rear cam
    {
        blue = 0;
        red = 2;
    }


    Mat rgb[3];  //bgr if rearcam
    split(roi, rgb);

    addWeighted(rgb[blue], 1.0, rgb[red], m_Properties.w, 0, rgb[blue]);
    addWeighted(rgb[blue], 1.0, rgb[1], m_Properties.w, 0, rgb[blue]);

    Mat Mblue;
    bilateralFilter(rgb[blue], Mblue, 9,75,75);

    Mat morph;
    Mat kernel = getStructuringElement(MORPH_RECT,Size{5,5});
    morphologyEx(Mblue,morph,MORPH_OPEN,kernel,Point{-1,-1},15);
    subtract(Mblue,morph,Mblue);

    Canny(Mblue, Mblue, m_Properties.Cnt.CannyThr1 , m_Properties.Cnt.CannyThr2);

    vector<vector<Point>> contours, drawcunts;
    findContours(Mblue,contours,RETR_TREE,CHAIN_APPROX_SIMPLE);

    sort(contours.begin(),contours.end(),[](const vector<Point> &a, const vector<Point> &b) -> bool {return contourArea(a) > contourArea(b);});
    vector<Point> ScrnCnt;
    if (!contours.empty())
    {
        for (int i = 0; i < min((int)contours.size(),10); i++)
        {
            double peri = arcLength(contours.at(i),true);
            approxPolyDP(contours.at(i),ScrnCnt,m_Properties.Cnt.periError*peri,true);
            //double aspect = calculateAspect(ScrnCnt);

            tFloat32 radius;
            Point2f center;
            minEnclosingCircle(ScrnCnt, center, radius);

            if(isContourConvex(ScrnCnt) && contourArea(ScrnCnt) >= m_Properties.Cnt.minArea && std::abs(radius-calculateRadius(contourArea(ScrnCnt)) <= m_Properties.Cnt.maxAreaDev) )
            {
                drawcunts.push_back(ScrnCnt);

                //break;

            }

        }
        //drawContours(objectimage,drawcunts,0,Scalar(0,255,0),3);
        //objectimage.copyTo(outputImage(object));
    }


    tBool BlauesLichtFound = false;

    if(!drawcunts.empty()) BlauesLichtFound = true;


    if (m_Properties.showDebug)
    {
        rectangle(*ShowImage, { roiVars.offsetX, roiVars.offsetY }, { roiVars.offsetX+ roiVars.width, roiVars.offsetY + roiVars.height }, { 255, 255, 0 }, 2);
        cvtColor(Mblue, Mblue, CV_GRAY2RGB);


        for( size_t i = 0; i < drawcunts.size(); i++ )
        {
            drawContours(Mblue, drawcunts, i, Scalar(255,0,0), 3);

        }

        (Mblue).copyTo((*ShowImage)(Rect(roiVars.offsetX, roiVars.offsetY, roiVars.width, roiVars.height)));

    }
    return BlauesLichtFound;
}


tResult cBlauesLicht::searchForBlauesLicht (const Mat *image)
{
    cv::Mat* ShowImage = nullptr;
    cv::Mat* ShowImage2 = nullptr;
    if (m_Properties.showDebug)
    {
        ShowImage = new cv::Mat();
        image->copyTo(*ShowImage);

        ShowImage2 = new cv::Mat();
        image->copyTo(*ShowImage2);

    }

    tBool BlauesLichtFoundCnt{tFalse}, BlauesLichtFoundHo{tFalse};



    BlauesLichtFoundCnt = blauesLichtFinderCnt(m_Properties.roi, image, ShowImage);
    if (BlauesLichtFoundCnt)
    {
        if (m_Properties.showDebug)
        {
            LOG_INFO(cString("Die Cnt hat was gefunden!!!"));
        }

//        transmitBoolSignalValue(m_oBoolOut,m_pClock->GetStreamTime(),m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.timeStamp
//                                ,0, m_ddlBoolSignalValueId.bValue, BlauesLichtFound);
    }


    BlauesLichtFoundHo = blauesLichtFinderHo(m_Properties.roi, image, ShowImage2);

    if(BlauesLichtFoundHo)
    {
        if(m_Properties.showDebug)
        {
             LOG_INFO(cString("Die Ho hat was gefunden!!!"));
        }
    }




    if (m_Properties.showDebug)
    {

        //update output format if matrix size does not fit to
        if (ShowImage->total() * ShowImage->elemSize() != m_sImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oVideoOut, *ShowImage);
        }
        // write to pin
        writeMatToPin(m_oVideoOut, *ShowImage, m_pClock->GetStreamTime());

        //update output format if matrix size does not fit to
//        if (ShowImage2->total() * ShowImage2->elemSize() != m_sImageFormat.m_szMaxByteSize)
//        {
//            setTypeFromMat(m_oLastVideo, *ShowImage2);
//        }
        // write to pin
        writeMatToPin(m_oLastVideo, *ShowImage2, m_pClock->GetStreamTime());


        delete ShowImage2;
        delete ShowImage;
    }
    RETURN_NOERROR;
}


double cBlauesLicht::calculateAspect(vector<Point> contours)
{
    tInt32 xmin,xmax,ymin,ymax;
    xmin = 2000;
    xmax = 0;
    ymin = 2000;
    ymax = 0;
    for(vector<Point>::iterator pit = contours.begin(); pit != contours.end(); pit++)
    {
        xmin = pit->x <  xmin ? pit->x : xmin;
        xmax = pit->x >  xmax ? pit->x : xmax;

        ymin = pit->y <  ymin ? pit->y : ymin;
        ymax = pit->y >  ymax ? pit->y : ymax;
    }
    return (static_cast<double>(xmax - xmin))/(static_cast<double>(ymax -ymin));
}

float cBlauesLicht::calculateRadius(tFloat area)
{
    return (sqrt(area/CV_PI));

}
