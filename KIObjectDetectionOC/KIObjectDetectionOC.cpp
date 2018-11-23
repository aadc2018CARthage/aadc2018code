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

//using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;
using namespace adtf::filter;
using namespace adtf::mediadescription;
using namespace adtf_ddl;

//using namespace cv;
//using namespace cv::dnn;
//using namespace std;

using cv::Mat;
using cv::Rect;
using cv::Scalar;
using cv::Size;
using cv::Point;
using cv::String;
using cv::Vec3f;
using cv::Vec3i;

#include "KIObjectDetectionOC.h"
#include <ADTF3_OpenCV_helper.h>

#include <iostream>
#include <chrono>

#define CLAMP(X, MIN, MAX) ((X < MIN) ? MIN : ((X > MAX) ? MAX : X))

ADTF_PLUGIN(LABEL_KI_OBJECT_DETECTION_FILTER, cKIObjectDetection)

std::vector<cString> cKIObjectDetection::readClassNames(const char *filename)
{
    std::vector<cString> classNames;
    std::ifstream fp(filename);
    if (!fp.is_open())
    {
        LOG_ERROR("File with classes labels not found: %s", filename);
        return std::vector<cString>();
    }
    std::string name;
    while (!fp.eof())
    {
        std::getline(fp, name);
        if (name.length())
        {
            classNames.push_back(name.substr(name.find(' ') + 1).c_str());
        }
    }
    fp.close();
    return classNames;
}

cKIObjectDetection::cKIObjectDetection() :
    SteeringValue(0.0f)
{
    //create image pins
    m_sCurrentFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    m_sCurrentFormat.m_ui32Width = 1920;
    m_sCurrentFormat.m_ui32Height = 1080;
    m_sCurrentFormat.m_szMaxByteSize = 0;
    m_sCurrentFormat.m_ui8DataEndianess = PLATFORM_BYTEORDER;
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());

    set_stream_type_image_format(*pType, m_sCurrentFormat);

    filter_create_pin(*this, m_oReader, "basler_rgb", ucom_object_ptr_cast<const IStreamType>(pType));
    filter_create_pin(*this, m_oWriter, "debugVideo", ucom_object_ptr_cast<const IStreamType>(pType));

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sCurrentFormat, *pType.Get(), m_oWriter);
    });

    m_oThreadRunner = runnable<>(ADTF_RUN_FUNCTION(Process));
    RegisterRunner("thread_runner", m_oThreadRunner);
    ConfigureThreadTrigger("thread_runner", tTrue);

    //get the media description for the classification
    object_ptr<IStreamType> pTypeClassifaction;

    if IS_FAILED(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tObjectDetectionData",                                                                                           pTypeClassifaction, m_oCodecFactory))
    {
        LOG_ERROR("Could not load media description for output pin classification");
    }
    else
    {

        adtf_ddl::access_element::find_index(m_oCodecFactory, "ui32Size", m_ObjectDetectionDataIDs.ui32SizeId);
        adtf_ddl::access_element::find_array_index(m_oCodecFactory, "tObjectArray", m_ObjectDetectionDataIDs.tObjectarrayId);
    }
    //create output pin for classifacation
    filter_create_pin(*this, m_oObjectWriter, "Objects", ucom_object_ptr_cast<const IStreamType>(pTypeClassifaction));

    object_ptr<IStreamType> pTypeSignalValue;
    if (IS_OK(create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)))
    {
        access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp);
        access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.f32Value);
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    //create speed and steering output pins
    filter_create_pin(*this, m_oSteeringWriter, "steering", pTypeSignalValue);
    filter_create_pin(*this, m_oSpeedWriter, "speed", pTypeSignalValue);

    filter_create_pin(*this, m_oSpeedReader, "current_speed", pTypeSignalValue);
    m_oSpeedRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessCurrentSpeed));
    RegisterRunner("speed_runner", m_oSpeedRunner);
    ConfigureDataInTrigger("speed_runner", "current_speed");

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue);
    }
    else
    {
        LOG_WARNING("No mediadescription for tBoolSignalValue found!");
    }

    filter_create_pin(*this, m_oActivateReader, "activateOutput", pTypeBoolSignalValue);
    m_oActivateRunner = runnable<IRunnable::RUN_TRIGGER>(ADTF_RUN_FUNCTION(ProcessActivate));
    RegisterRunner("activateRunner", m_oActivateRunner);
    ConfigureDataInTrigger("activateRunner", "activateOutput");

    //register properties
    RegisterPropertyVariable("opencl", m_bOpenCL);
    RegisterPropertyVariable("proto", m_strProtoTxt);
    RegisterPropertyVariable("model", m_strModel);
    RegisterPropertyVariable("classNames", m_strClassNames);
    RegisterPropertyVariable("show DebugVideo", m_showDebugVideo);

    RegisterPropertyVariable("blobFromImage Scale", m_blobParams.scale);
    RegisterPropertyVariable("blobFromImage meanR", m_blobParams.meanR);
    RegisterPropertyVariable("blobFromImage meanG", m_blobParams.meanG);
    RegisterPropertyVariable("blobFromImage meanB", m_blobParams.meanB);
    RegisterPropertyVariable("blobFromImage channelSwap", m_blobParams.channelSwap);

    RegisterPropertyVariable("ROI - x", m_roiParams.roiX);
    RegisterPropertyVariable("ROI - y", m_roiParams.roiY);
    RegisterPropertyVariable("ROI - width", m_roiParams.width);
    RegisterPropertyVariable("ROI - height", m_roiParams.height);
    RegisterPropertyVariable("use ROI", m_useRoi);

    RegisterPropertyVariable("Decision - Confidence Threshhold", m_confThr);
    RegisterPropertyVariable("Decision - min Height of Cars", m_minHeightOfCar);
    RegisterPropertyVariable("Decision - min Height of Barbies", m_minHeightOfBarbie);
    RegisterPropertyVariable("Decision - min Height of Babys", m_minHeightOfBaby);
    RegisterPropertyVariable("Decision - max Height of Barbies", m_maxHeightOfBarbie);
    RegisterPropertyVariable("Decision - max Height of Babys", m_maxHeightOfBaby);
    RegisterPropertyVariable("Decision - ignore all Objects over this threshold", m_lowerThreshold);
    RegisterPropertyVariable("Decision - ignore all Objects under this threshold", m_upperThreshold);
    //params properties
    RegisterPropertyVariable("numberplate - lower rel area Threshold gyay", m_Properties.params.areaLow);
    RegisterPropertyVariable("numberplate - higher rel area Threshold gyay", m_Properties.params.areaHigh);
    RegisterPropertyVariable("numberplate - minAspectRatio", m_Properties.params.minAspectRatio);
    RegisterPropertyVariable("numberplate - steering prop factor", m_Properties.params.steeringPropFactor);

    RegisterPropertyVariable("speed - height threshold for desired distance", m_Properties.params.heightThresholdSpeed);
    RegisterPropertyVariable("speed - scaling factor", m_Properties.params.factorSpeed);
    RegisterPropertyVariable("Speed - max offset", m_Properties.params.maxOffset);
    RegisterPropertyVariable("Speed - NEW scaling factor offset", m_speedMultDist);
    RegisterPropertyVariable("Speed - NEW scaling factor total", m_speedMultTotal);

}

tResult cKIObjectDetection::Init(tInitStage eStage)
{
    // check if all necessary files exists
    cFilename filenameProtoTxt = m_strProtoTxt;
    adtf::services::ant::adtf_resolve_macros(filenameProtoTxt);
    if (!cFileSystem::Exists(filenameProtoTxt))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Prototext file could not be loaded from %s", filenameProtoTxt.GetPtr()));
    }

    cFilename filenameModel = m_strModel; ;
    adtf::services::ant::adtf_resolve_macros(filenameModel);
    if (!cFileSystem::Exists(filenameModel))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Network file could not be loaded from %s", filenameModel.GetPtr()));
    }

    cFilename filenameClassnames = m_strClassNames; ;
    adtf::services::ant::adtf_resolve_macros(filenameClassnames);
    if (!cFileSystem::Exists(filenameClassnames))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Class names file could not be loaded from %s", filenameClassnames.GetPtr()));
    }
    //read model
//    m_oNet = dnn::readNetFromCaffe(filenameProtoTxt.GetPtr(),
//                                   filenameModel.GetPtr());

    //read model
    m_pNet.reset(new Net<float>(filenameProtoTxt.GetPtr(), TEST));
    m_pNet->CopyTrainedLayersFrom(filenameModel.GetPtr());

    m_pInputBlob = m_pNet->input_blobs()[0];
    m_pInputBlob->Reshape(1, 3, m_pInputBlob->width(), m_pInputBlob->height());  //height und width vllt vertauschen
    m_pNet->Reshape();
    m_pOutputBlob = m_pNet->output_blobs()[0];

//    if (m_oNet.empty())
//    {
//        LOG_ERROR("Can't load network");
//        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Network could not be loaded"));
//    }
//    else
//    {
//        LOG_INFO(cString::Format("Loaded Caffe model sucessfully from %s", filenameModel.GetPtr()));
//    }
    //read class names
    m_classNames = readClassNames(filenameClassnames.GetPtr());

//    if (m_bOpenCL)
//    {
//        m_oNet.setPreferableTarget(DNN_TARGET_OPENCL);
//    }

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cKIObjectDetection::Shutdown(tInitStage eStage)
{
    RETURN_NOERROR;
}

tResult cKIObjectDetection::Process(tTimeStamp tmTimeOfTrigger)
{
    Caffe::set_mode(Caffe::GPU);

    auto t1 = chrono::high_resolution_clock::now();

    object_ptr<const ISample> pReadSample;

    //the result for classification
    Mat outputImage;
    Mat inputImage;
//    Mat detections;
    Mat detectionMat;
    vector<Point> ScrnCunt;
    tFloat32 speed = m_currentSpeed;

    if (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer

        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            inputImage = Mat(cv::Size(m_sCurrentFormat.m_ui32Width, m_sCurrentFormat.m_ui32Height),
                             CV_8UC3, (uchar*)pReadBuffer->GetPtr());
        }

    }  else {

        //LOG_INFO("KI OpenChallenge : Video Input is not ok!");
        RETURN_NOERROR;
    }


    Mat roiImg = m_useRoi ? inputImage(Rect(m_roiParams.roiX, m_roiParams.roiY, m_roiParams.width, m_roiParams.height)) : inputImage;
    if (m_showDebugVideo) roiImg.copyTo(outputImage);

    try
    {
        //MobileNet-SSD: scale:2.0/255 size:Size(300, 300)  mean:Scalar(127.5, 127.5, 127.5)
        Mat oBlob = cv::dnn::blobFromImage(roiImg, m_blobParams.scale, Size(300, 300),
                                  Scalar(m_blobParams.meanR, m_blobParams.meanG, m_blobParams.meanB),m_blobParams.channelSwap, false);

        memcpy(m_pInputBlob->mutable_cpu_data(), oBlob.data, oBlob.total()*sizeof(float));
        m_pNet->Forward();
        detectionMat.create(m_pOutputBlob->shape(2), m_pOutputBlob->shape(3), CV_32F);
        memcpy(detectionMat.data, m_pOutputBlob->cpu_data(), m_pOutputBlob->shape(2) * m_pOutputBlob->shape(3) * sizeof(float));

//        if (!m_oNet.empty() && oBlob.size > 0)
//        {
//            m_oNet.setInput(oBlob, "data");        //set the network input
//            detections = m_oNet.forward("detection_out");
//            // Mobile-SSD detections [BatchID] [Class] [Confidence] [xLeftBottom] [yLeftBottom] [xRightTop] [yRightTop]

//        }
    }
    catch (cv::Exception ex)
    {
        const char* err_msg = ex.what();
        LOG_ERROR(cString("OpenCV exception caught: ") + err_msg);
    }


//    Mat detectionMat(detections.size[2], detections.size[3], CV_32F, detections.ptr<float>());

    tObject DetectedObjects[10];
    memset(DetectedObjects, 0, 10*sizeof(tObject));
    tSize n = 0;

    for(int i = 0; i < min(detectionMat.rows, 10); i++)
    {
        float confidence = detectionMat.at<float>(i, 2);

        if(confidence >= m_confThr)
        {
            tInt32 objectClass = (tInt32)(detectionMat.at<float>(i, 1));

            tInt32 xLeftBottom = CLAMP(detectionMat.at<float>(i, 3),0.0f,1.0f) * (roiImg.cols - 1);
            tInt32 yLeftBottom = CLAMP(detectionMat.at<float>(i, 4),0.0f,1.0f) * (roiImg.rows - 1);
            tInt32 xRightTop = CLAMP(detectionMat.at<float>(i, 5),0.0f,1.0f) * (roiImg.cols - 1);
            tInt32 yRightTop = CLAMP(detectionMat.at<float>(i, 6),0.0f,1.0f) * (roiImg.rows - 1);

            tBool shallBeAdded = tFalse;
            tInt32 orientation = 0;

            tInt32 low = m_useRoi ? static_cast<tInt32>(m_lowerThreshold-(m_roiParams.roiY)) : static_cast<tInt32>(m_lowerThreshold);
            tInt32 up = m_useRoi ? static_cast<tInt32>(m_upperThreshold-(m_roiParams.roiY)) : static_cast<tInt32>(m_upperThreshold);

            if(yRightTop > low && yRightTop < up)
            {
                tInt32 height = std::abs(yLeftBottom-yRightTop);
                switch(Objects(objectClass))
                {
                case Objects::Car:
                    if(height>m_minHeightOfCar) shallBeAdded = tTrue;
                    break;
                case Objects::Child:
                    if(height > m_minHeightOfBaby && height < m_maxHeightOfBaby) shallBeAdded = tTrue;
                    break;
                case Objects::Person:
                    if(height > m_minHeightOfBarbie && height < m_maxHeightOfBarbie) shallBeAdded = tTrue;
                    break;
                default:
                    break;
                }
            }

            // Number plate detection ----------------------------------
            if(objectClass == 3)
            {
                //compute the speed

                tFloat32 outputSpeed;
                if((m_Properties.params.heightThresholdSpeed - static_cast<tFloat32>(yRightTop)) > 50.f)
                {
                LOG_INFO(cString("oc - too far"));
                    outputSpeed = m_speedMultTotal * m_currentSpeed + m_speedMultDist * m_speedMultTotal;
                }
                else //if((static_cast<tFloat32>(yRightTop) - m_Properties.params.heightThresholdSpeed) > 50.f)
                {
                                LOG_INFO(cString("oc - too close"));
                    outputSpeed = m_speedMultTotal * m_currentSpeed - m_speedMultDist * m_speedMultTotal;
                }
//                else
//                {
//                                LOG_INFO(cString("oc - alrighty"));
//                    outputSpeed = m_speedMultTotal * m_currentSpeed;
//                }

                TransmitSpeed(outputSpeed);

                //compute the steering
                Rect object(xLeftBottom, yLeftBottom+std::abs(yRightTop-yLeftBottom)/2, std::abs(xRightTop-xLeftBottom), std::abs(yRightTop-yLeftBottom)/2);
                Mat plateImg, plateImgOld, objectimage;
                plateImgOld = roiImg(object);
                plateImgOld.copyTo(objectimage);
                cvtColor(plateImgOld, plateImgOld, cv::COLOR_RGB2GRAY);
                Mat histImg;
                equalizeHist(plateImgOld,histImg);
                bilateralFilter(histImg,plateImg,9,75,75);
                Mat kernel = getStructuringElement(cv::MORPH_RECT,Size{5,5});
                morphologyEx(plateImg,plateImg,cv::MORPH_OPEN,kernel,Point{-1,-1},15);
                subtract(histImg,plateImg,plateImg);
                threshold(plateImg,plateImg,0,255,CV_THRESH_OTSU);
                Canny(plateImg,plateImg,250,255);
                convertScaleAbs(plateImg,plateImg);
                Mat kernelNew = Mat::eye(Size{3,3},CV_8UC1);
                dilate(plateImg,plateImg,kernelNew);
                vector<vector<Point> > contours,drawcunts;
                findContours(plateImg,contours,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);

                sort(contours.begin(),contours.end(),[](const vector<Point> &a, const vector<Point> &b) -> bool {return contourArea(a) > contourArea(b);});

                if (!contours.empty())
                {
                    for (int i = 0; i < min((int)contours.size(),20); i++)
                    {
                        double peri = arcLength(contours.at(i),true);
                        approxPolyDP(contours.at(i),ScrnCunt,0.06*peri,true);
                        double aspect = calculateAspect(ScrnCunt);


                        if(ScrnCunt.size() > 3 && ScrnCunt.size() <= 5 && contourArea(ScrnCunt)/object.area() >= m_Properties.params.areaLow && contourArea(ScrnCunt)/object.area() <= m_Properties.params.areaHigh && aspect > m_Properties.params.minAspectRatio && isContourConvex(ScrnCunt)) // && fabs(contourArea(ScrnCunt)/contourArea(contours.at(i)) - 1) < 0.5 && isContourConvex(ScrnCunt)
                        {
                            drawcunts.push_back(ScrnCunt);
                            LOG_INFO(cString::Format("Aspect ratio is: %f",aspect));

                            double meanX = calculateMeanX(ScrnCunt);
                            SteeringValue = m_Properties.params.steeringPropFactor*(meanX - static_cast<tFloat32>((object.width -1)/2.0f))/static_cast<tFloat32>(object.width);
                            TransmitSteering(SteeringValue);

                            break;
                        }
                    }
                    drawContours(objectimage,drawcunts,0,Scalar(0,255,0),3);
                    objectimage.copyTo(outputImage(object));
                }
            }

            if(true)
            {
                tObject DetectedObject = {objectClass, orientation, xLeftBottom, yLeftBottom, xRightTop, yRightTop};
                DetectedObjects[i] = DetectedObject;

                if(m_showDebugVideo)
                {
                    ostringstream ss;
                    ss.str("");
                    ss << confidence;
                    String conf(ss.str());

                    Rect object(xLeftBottom, yLeftBottom, std::abs(xRightTop-xLeftBottom), std::abs(yRightTop-yLeftBottom));

                    rectangle(outputImage, object, Scalar(0, 255, 0));
                    String label = String(m_classNames[objectClass]) + ": " + conf;
                    int baseLine = 0;
                    Size labelSize = getTextSize(label, CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                    rectangle(outputImage, Rect(Point(xLeftBottom, max(0, yLeftBottom - labelSize.height)),
                                                Size(labelSize.width, labelSize.height + baseLine)),
                              Scalar(255, 255, 255), CV_FILLED);
                    putText(outputImage, label, Point(xLeftBottom, yLeftBottom),
                            CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,0));
                }
            }
            n++;
        }
    }

    // Number plate detection ----------------------------------


    //Debug Video
    if(m_showDebugVideo)
    {
        //draw property lines
        //tInt32 low = m_useRoi ? static_cast<tInt32>(m_lowerThreshold-(m_roiParams.roiY)) : static_cast<tInt32>(m_lowerThreshold);
        //tInt32 up = m_useRoi ? static_cast<tInt32>(m_upperThreshold-(m_roiParams.roiY)) : static_cast<tInt32>(m_upperThreshold);

        //line(outputImage, Point(0, low), Point(1910, low), Scalar(255, 255, 0), 3, LINE_8, 0);
        //line(outputImage, Point(0, up), Point(1910, up), Scalar(0, 255, 0), 3, LINE_8, 0);
        //        line(outputImage, Point(50, 750), Point(50, std::min(0, static_cast<int>(750-1080*m_minHeightOfCar))), Scalar(255, 255, 0), 3, LINE_8, 0);
        //        line(outputImage, Point(125, 750), Point(125, static_cast<int>(750-1080*m_minHeightOfBarbie)), Scalar(0, 255, 255), 3, LINE_8, 0);
        //        line(outputImage, Point(175, 750), Point(175, std::min(0, static_cast<int>(750-1080*m_maxHeightOfBarbie))), Scalar(0, 255, 255), 3, LINE_8, 0);
        //        line(outputImage, Point(250, 750), Point(250, std::min(0, static_cast<int>(750-1080*m_minHeightOfBaby))), Scalar(255, 0, 255), 3, LINE_8, 0);
        //        line(outputImage, Point(300, 750), Point(300, std::min(0, static_cast<int>(400+1080*m_maxHeightOfBaby))), Scalar(255, 0, 255), 3, LINE_8, 0);

        line(outputImage, Point(1,m_Properties.params.heightThresholdSpeed), Point(1279,m_Properties.params.heightThresholdSpeed), Scalar(0,255,0), 3, cv::LINE_8);

        //update output format if matrix size does not fit to
        tInt midpoint = (outputImage.cols - 1)/2 + SteeringValue*outputImage.cols/200;
        tInt height = 30;
        vector<Point> cunts;
        vector<vector<Point> > yoMama;
        cunts.push_back(Point{midpoint,outputImage.rows-21});
        cunts.push_back(Point{midpoint-10,outputImage.rows-51});
        cunts.push_back(Point{midpoint+10,outputImage.rows-51});
        yoMama.push_back(cunts);

        //line(outputImage,Point{(outputImage.cols - 1)/2 + SteeringValue*outputImage.cols/200.0f,outputImage.rows-1},Point{(outputImage.cols - 1)/2 + SteeringValue*outputImage.cols/200.0f,outputImage.rows-31},Scalar(0,255,0),15);

        //line(outputImage,Point{0,outputImage.rows-1},Point{0,outputImage.rows-21},Scalar(255,0,0),10);
        int numberOfLines = 12;

        for (int i = 0; i <numberOfLines +1; i++)
        {
            Scalar color = Scalar(0,0,255);
            double factor = i % 2 == 1 ? 0.6 : 1;
            if (i == numberOfLines/2){
                factor = 1.5;
                color = Scalar(255,0,0);
            }

            line(outputImage,Point{i*(outputImage.cols -1)/numberOfLines,outputImage.rows-1},Point{i*(outputImage.cols-1)/numberOfLines,outputImage.rows-factor*21},color,factor*10);
        }
        line(outputImage,Point{0,outputImage.rows-1},Point{outputImage.cols -1,outputImage.rows-1},Scalar(0,0,255),10,cv::LINE_4);


        //draw speedometer
        tInt speedMidpoint = CLAMP(0.66*(outputImage.rows-1) - speed*outputImage.rows, 5 , 920);

        //making a speed indicator and draw both speed and steering indicator
        vector<Point> speedIndicatorCnt;
        speedIndicatorCnt.push_back(Point{outputImage.cols-21, speedMidpoint});
        speedIndicatorCnt.push_back(Point{outputImage.cols-51, speedMidpoint-10});
        speedIndicatorCnt.push_back(Point{outputImage.cols-51, speedMidpoint+10});
        yoMama.push_back(speedIndicatorCnt);
        drawContours(outputImage,yoMama,0,Scalar(0,255,0),CV_FILLED);

        //draw big speed líne
        line(outputImage,Point{outputImage.cols-1,0},Point{outputImage.cols -1,outputImage.rows-1},Scalar(0,0,255),10,cv::LINE_4);

        //draw ticks for speedometer
        for (int i = 0; i <numberOfLines +1; i++)
        {
            Scalar color = Scalar(0,0,255);
            double factor = i % 2 == 1 ? 0.6 : 1;
            if (i == numberOfLines/2){
                factor = 1.5;
                color = Scalar(255,0,0);
            }

            line(outputImage,Point{outputImage.cols-1, i*(outputImage.rows -1)/numberOfLines,},Point{outputImage.cols-factor*21,i*(outputImage.rows-1)/numberOfLines},color,factor*10);
        }

        if (outputImage.total() * outputImage.elemSize() != m_sCurrentFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }
    if(m_showFPS)
    {
        auto t2 =chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        tFloat32 fTime = static_cast<tFloat32>(time);
        fTime = 1000/fTime;
        LOG_INFO(cString::Format("FPS: %f", fTime));
    }
    //m_oReader.Clear();
    RETURN_NOERROR;
}


double cKIObjectDetection::calculateAspect(vector<Point> contours)
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

double cKIObjectDetection::calculateMeanX(vector<Point> contours)
{
    tInt32 xmin,xmax;
    xmin = 2000;
    xmax = 0;
    for(vector<Point>::iterator pit = contours.begin(); pit != contours.end(); pit++)
    {
        xmin = pit->x <  xmin ? pit->x : xmin;
        xmax = pit->x >  xmax ? pit->x : xmax;
    }
    return static_cast<double>(xmax + xmin)/2.0f;
}

tResult cKIObjectDetection::TransmitSteering(tFloat32 value)
{
    if(m_active)
    {
        object_ptr<ISample> pSample;
        if (IS_OK(alloc_sample(pSample)))
        {
            auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pSample);
            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.f32Value, value));
        }
        m_oSteeringWriter << pSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cKIObjectDetection::TransmitSpeed(tFloat32 value)
{
    if(m_active)
    {
        object_ptr<ISample> pSample;
        if (IS_OK(alloc_sample(pSample)))
        {
            auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pSample);
            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.f32Value, value));
        }
        m_oSpeedWriter << pSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cKIObjectDetection::ProcessCurrentSpeed(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSample;
    if (IS_OK(m_oSpeedReader.GetNextSample(pSample)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.f32Value, &m_currentSpeed));
    }
    RETURN_NOERROR;
}

tResult cKIObjectDetection::ProcessActivate(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pSample;
    if (IS_OK(m_oActivateReader.GetNextSample(pSample)))
    {
        auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pSample);
        RETURN_IF_FAILED(oDecoder.IsValid());
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &m_active));
    }
    RETURN_NOERROR;
}
