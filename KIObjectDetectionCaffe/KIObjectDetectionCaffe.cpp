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

//using namespace std;
using cv::Mat;
using cv::Rect;
using cv::Scalar;
using cv::Size;
using cv::Point;
using cv::String;
using cv::Vec3f;
using cv::Vec3i;

#include "KIObjectDetectionCaffe.h"
#include <ADTF3_OpenCV_helper.h>

#include <iostream>
#include <chrono>

#define CLAMP(X, MIN, MAX) ((X < MIN) ? MIN : ((X > MAX) ? MAX : X))

ADTF_PLUGIN(LABEL_KI_OBJECT_DETECTION_FILTER, cKIObjectDetectionCaffe)

std::vector<cString> cKIObjectDetectionCaffe::readClassNames(const char *filename)
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

cKIObjectDetectionCaffe::cKIObjectDetectionCaffe()
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

    //m_oThreadRunner = runnable<>(ADTF_RUN_FUNCTION(Process));
    //RegisterRunner("thread_runner", m_oThreadRunner);
    //ConfigureThreadTrigger("thread_runner", tTrue);
    for (tUInt32 i = 0; i < m_oRunners.size(); i++)
    {
        m_oRunners[i] = runnable<>(ADTF_RUN_FUNCTION(Process));
        RegisterRunner(cString::Format("data_in_runner_%d", i).GetPtr(), m_oRunners[i]);
        ConfigureThreadTrigger(cString::Format("data_in_runner_%d", i), "basler_rgb");
    }

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
    //create output pin for blue light
    filter_create_pin(*this, m_oBlueLightWriter, "emergency_car", pTypeBoolSignalValue);

    //register properties
    RegisterPropertyVariable("opencl", m_bOpenCL);
    RegisterPropertyVariable("proto", m_strProtoTxt);
    RegisterPropertyVariable("model", m_strModel);
    RegisterPropertyVariable("classNames", m_strClassNames);
    RegisterPropertyVariable("show DebugVideo", m_showDebugVideo);
    RegisterPropertyVariable("show FPS", m_showFPS);


    RegisterPropertyVariable("blobFromImage Scale", m_blobParams.scale);
    RegisterPropertyVariable("blobFromImage meanR", m_blobParams.meanR);
    RegisterPropertyVariable("blobFromImage meanG", m_blobParams.meanG);
    RegisterPropertyVariable("blobFromImage meanB", m_blobParams.meanB);
    RegisterPropertyVariable("blobFromImage channelSwap", m_blobParams.channelSwap);
    RegisterPropertyVariable("blobFromImage Size (300 or 500)", m_blobParams.size);

    RegisterPropertyVariable("ROI - x", m_roiParams.roiX);
    RegisterPropertyVariable("ROI - y", m_roiParams.roiY);
    RegisterPropertyVariable("ROI - width", m_roiParams.width);
    RegisterPropertyVariable("ROI - height", m_roiParams.height);
    RegisterPropertyVariable("ROI - use ROI", m_useRoi);

    //RegisterPropertyVariable("Decision - Minimum Confidence Threshhold", m_confThr.min);
    RegisterPropertyVariable("Decision - Confidence Threshold Car", m_confThr.car);
    RegisterPropertyVariable("Decision - Confidence Threshold Barbie", m_confThr.barbie);
    RegisterPropertyVariable("Decision - Confidence Threshold Child", m_confThr.baby);

    RegisterPropertyVariable("Decision - min Height of Cars", m_minHeightOfCar);
    RegisterPropertyVariable("Decision - min Height of Barbies", m_minHeightOfBarbie);
    RegisterPropertyVariable("Decision - min Height of Babys", m_minHeightOfBaby);
    RegisterPropertyVariable("Decision - max Height of Barbies", m_maxHeightOfBarbie);
    RegisterPropertyVariable("Decision - max Height of Babys", m_maxHeightOfBaby);
    RegisterPropertyVariable("Decision - ignore all Objects over this threshold", m_lowerThreshold);
    RegisterPropertyVariable("Decision - ignore all Objects under this threshold", m_upperThreshold);

    //properties blaues licht
    RegisterPropertyVariable("Blue Light - channel subtraction weight", m_Properties.w);

    RegisterPropertyVariable("Blue Light - GaussianBlur KernelSize", m_Properties.blurr.kSize);
    RegisterPropertyVariable("Blue Light - GaussianBlur sigma", m_Properties.blurr.sigma);

    RegisterPropertyVariable("Blue Light - Hough: dp", m_Properties.hough.dp);
    RegisterPropertyVariable("Blue Light - Hough: minDistance", m_Properties.hough.minDist);
    RegisterPropertyVariable("Blue Light - Hough: Param1", m_Properties.hough.param1);
    RegisterPropertyVariable("Blue Light - Hough: Param2", m_Properties.hough.param2);
    RegisterPropertyVariable("Blue Light - Hough: minRadius", m_Properties.hough.minRadius);
    RegisterPropertyVariable("Blue Light - Hough: maxRadius", m_Properties.hough.maxRadius);

    RegisterPropertyVariable("Blue Light - Roi Height Scaling Factor", m_Properties.heightFactor);
    RegisterPropertyVariable("Blue Light - ROI Height Offset Scaling Factor", m_Properties.factor);
    RegisterPropertyVariable("Blue Light - Draw Blue Light Box", m_drawBlueBox);
    RegisterPropertyVariable("Blue Light - Save Blue Light Images", m_saveBluelightImgs);
    RegisterPropertyVariable("Blue Light - Directory for Saving Pictures", m_directory);

    //Caffe::set_mode(Caffe::GPU);
}

tResult cKIObjectDetectionCaffe::Init(tInitStage eStage)
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
    m_pNet.reset(new Net<float>(filenameProtoTxt.GetPtr(), TEST));
    m_pNet->CopyTrainedLayersFrom(filenameModel.GetPtr());

    m_pInputBlob = m_pNet->input_blobs()[0];
    m_pInputBlob->Reshape(1, 3, m_pInputBlob->width(), m_pInputBlob->height());  //height und width vllt vertauschen
    m_pNet->Reshape();

    m_oInputLayer.create(m_pInputBlob->width(), m_pInputBlob->height(), CV_32F);
    m_oInputBuffer.create(m_pInputBlob->width(), m_pInputBlob->height(), CV_32F);

    //auto output_layer = m_pNet->blob_by_name("detection_out");
    m_pOutputBlob = m_pNet->output_blobs()[0];
    m_oOutputLayer.create(m_pOutputBlob->width(), m_pOutputBlob->height(), CV_32F);
    m_oOutputBuffer.create(m_pOutputBlob->width(), m_pOutputBlob->height(), CV_32F);

    //    if (m_pNet.empty())
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
    //        m_pNet.setPreferableTarget(DNN_TARGET_OPENCL);
    //    }

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult cKIObjectDetectionCaffe::Shutdown(tInitStage eStage)
{
    RETURN_NOERROR;
}

tResult cKIObjectDetectionCaffe::Process(tTimeStamp tmTimeOfTrigger)
{
    Caffe::set_mode(Caffe::GPU);

    object_ptr<const ISample> pReadSample;
    //m_oSampleMutex.lock();
    if (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        //m_oSampleMutex.unlock();
        //for (tUInt32 i = 0; i < m_pNets.size(); i++)
        {
            //if (m_oMutex[i].try_lock())
            {
                object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
                //lock read buffer

                if (IS_OK(pReadSample->Lock(pReadBuffer)))
                {
                    //create a opencv matrix from the media sample buffer
                    Mat inputImage = Mat(Size(m_sCurrentFormat.m_ui32Width, m_sCurrentFormat.m_ui32Height),
                                         CV_8UC3, (uchar*)pReadBuffer->GetPtr());

                    auto t1 = chrono::high_resolution_clock::now();
                    RETURN_IF_FAILED(Process_(inputImage));
                    if(m_showFPS)
                    {
                        auto t2 =chrono::high_resolution_clock::now();
                        auto time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
                        tFloat32 fTime = static_cast<tFloat32>(time);
                        fTime = 1000/fTime;
                        LOG_INFO(cString::Format("FPS: %f", fTime));
                    }
                }
                //m_oMutex[i].unlock();
                //break;
            }
        }
        //LOG_INFO("KI Caffe: No Net available!");
    }
    else
    {
        //m_oSampleMutex.unlock();
    }
    RETURN_NOERROR;
}

tResult cKIObjectDetectionCaffe::Process_(Mat& inputImage)
{
    //the result for classification
    Mat outputImage;
    //    Mat detections;
    Mat detectionMat;
    Mat roiImg = m_useRoi ? inputImage(Rect(m_roiParams.roiX, m_roiParams.roiY, m_roiParams.width, m_roiParams.height)) : inputImage;

    if (m_showDebugVideo) roiImg.copyTo(outputImage);

    try
    {
        //MobileNet-SSD: scale:2.0/255 size:Size(300, 300)  mean:Scalar(127.5, 127.5, 127.5)
        Mat oBlob = cv::dnn::blobFromImage(roiImg, m_blobParams.scale, Size(m_blobParams.size, m_blobParams.size),
                                           Scalar(m_blobParams.meanR, m_blobParams.meanG, m_blobParams.meanB),m_blobParams.channelSwap, false);

        // dont block if gpu buffer is not available
        //        if (!m_oInputBufferMutex.try_lock())
        //        {
        //            return ERR_NOT_READY;
        //        }

        // upload blob to gpu buffer
        //        m_oInputBuffer.upload(oBlob);

        //        // copy blob from gpu buffer to net
        //        m_oInputLayerMutex.lock();
        //        m_oInputBuffer.swap(m_oInputLayer);
        //        m_pInputBlob->set_gpu_data(reinterpret_cast<float *>(m_oInputLayer.data));
        //        m_oInputBufferMutex.unlock();

        //        // run detection
        //        m_oOutputLayerMutex.lock();
        //        m_pNet->Forward();
        //        m_oInputLayerMutex.unlock();

        //        // copy results from net to gpu buffer
        //        m_oOutputBufferMutex.lock();
        //        m_oOutputLayer.swap(m_oOutputBuffer);
        //        m_pOutputBlob->set_gpu_data(reinterpret_cast<float *>(m_oOutputLayer.data));
        //        m_oOutputLayerMutex.unlock();

        //        // download results from gpu buffer
        //        m_oOutputBuffer.download(detections);
        //        m_oOutputBufferMutex.unlock();

        memcpy(m_pInputBlob->mutable_cpu_data(), oBlob.data, oBlob.total()*sizeof(float));
        m_pNet->Forward();
        //detectionMat = Mat(detection_blob->shape(2), detection_blob->shape(3), CV_32F, const_cast<float *>(detection_blob->cpu_data()));
        detectionMat.create(m_pOutputBlob->shape(2), m_pOutputBlob->shape(3), CV_32F);
        // todo: dont copy (const_cast? maybe just work on the blob instead of mats?)
        memcpy(detectionMat.data, m_pOutputBlob->cpu_data(), m_pOutputBlob->shape(2) * m_pOutputBlob->shape(3) * sizeof(float));
        //        if (!m_pNet.empty() && oBlob.size > 0)
        //        {
        //            m_pNet.setInput(oBlob, "data");        //set the network input
        //            detections = m_pNet.forward("detection_out");
        //            // Mobile-SSD detections [BatchID] [Class] [Confidence] [xLeftBottom] [yLeftBottom] [xRightTop] [yRightTop]

        //        }
    }
    catch (cv::Exception ex)
    {
        const char* err_msg = ex.what();
        LOG_ERROR(cString("OpenCV exception caught: ") + err_msg);
    }


    //detectionMat(detections.size[2], detections.size[3], CV_32F, detections.ptr<float>());

    tObject DetectedObjects[10];
    memset(DetectedObjects, 0, 10*sizeof(tObject));
    tSize n = 0;

    for(int i = 0; i < min(detectionMat.rows, 10); i++)
    {
        float confidence = detectionMat.at<float>(i, 2);
        tInt32 objectClass = (tInt32)(detectionMat.at<float>(i, 1));
        tFloat32 minConf = 2.9f;

        switch (Objects(objectClass)) {
        case Objects::Person:
            minConf = m_confThr.barbie;
            break;

        case Objects::Child:
            minConf = m_confThr.baby;
            break;
        case Objects::Car:
            minConf = m_confThr.car;
        default:
            break;
        }


        if(confidence >= minConf)
        {
            //tInt32 objectClass = (tInt32)(detectionMat.at<float>(i, 1));

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

            if(shallBeAdded)
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

    object_ptr<ISample> pSample;
    {
        if IS_OK(alloc_sample(pSample))
        {
            {
                auto oCodec = m_oCodecFactory.MakeCodecFor(pSample);
                tObject* pObjects = reinterpret_cast<tObject*>(oCodec.GetElementAddress(m_ObjectDetectionDataIDs.tObjectarrayId));
                //memcpy(pObjects, &DetectedObjects[0], 10*sizeof(tObject));
                memset(pObjects, 0, 10 * sizeof(tObject));
                tUInt32 nIdx = 0;
                for (const tObject& object : DetectedObjects)
                {
                    pObjects[nIdx].id = object.id;
                    pObjects[nIdx].orientation = object.orientation;
                    pObjects[nIdx].xBottomLeft = object.xBottomLeft;
                    pObjects[nIdx].yBottomLeft = object.yBottomLeft;
                    pObjects[nIdx].xTopRight = object.xTopRight;
                    pObjects[nIdx].yTopRight = object.yTopRight;
                    nIdx++;
                }

                RETURN_IF_FAILED(oCodec.SetElementValue(m_ObjectDetectionDataIDs.ui32SizeId,static_cast<tVoid*>(&n)));
                // RETURN_IF_FAILED(oCodec.SetElementValue(m_ObjectDetectionDataIDs.tObjectarrayId, &DetectedObjects));
            }
            m_oObjectWriter << pSample << flush << trigger;
        }
    }

    tInt red = 0;
    tInt blue = 2;
    Mat rgb[3];
    vector<Vec3f> circles;

    // check for blue light on top of each car
    for (const tObject& object : DetectedObjects)
    {
        if(Objects(object.id) == Objects::Car)
        {
            tInt heightCar = std::abs(object.yBottomLeft-object.yTopRight);
            tInt height = m_Properties.heightFactor * heightCar - max(0, -static_cast<tInt>(object.yBottomLeft - m_Properties.factor * heightCar));
            tInt offsetX = object.xBottomLeft;
            tInt offsetY = max(0, static_cast<tInt>(object.yBottomLeft - m_Properties.factor * heightCar));
            tInt width = min((roiImg.cols - 1) - offsetX, std::abs(object.xBottomLeft-object.xTopRight));

            Mat roi = (roiImg)(Rect(offsetX, offsetY, width, height));

            split(roi, rgb);

            addWeighted(rgb[blue], 1.0, rgb[red], -m_Properties.w, 0, rgb[blue]);
            addWeighted(rgb[blue], 1.0, rgb[1], -m_Properties.w, 0, rgb[blue]);

            Size kSize(m_Properties.blurr.kSize, m_Properties.blurr.kSize);

            GaussianBlur(rgb[blue], rgb[blue], kSize, m_Properties.blurr.sigma);
            HoughCircles(rgb[blue], circles, CV_HOUGH_GRADIENT,
                         m_Properties.hough.dp, m_Properties.hough.minDist,
                         m_Properties.hough.param1, m_Properties.hough.param2,
                         m_Properties.hough.minRadius, m_Properties.hough.maxRadius);

            tBool blueLightFound = !circles.empty();

            if (blueLightFound)
            {
                object_ptr< ISample> pWriteSample;
                if (IS_OK(alloc_sample(pWriteSample)))
                {
                    auto oCodec = m_BoolSignalValueSampleFactory.MakeCodecFor(pWriteSample);
                    RETURN_IF_FAILED(oCodec.IsValid());
                    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolSignalValueId.bValue, tTrue));
                    m_oBlueLightWriter << pWriteSample << flush << trigger;
                }

                if (m_showDebugVideo)
                {
                    LOG_INFO(cString("Blue Light found!"));
                }
            }

            if(m_showDebugVideo)
            {
                rectangle(outputImage, { offsetX, offsetY }, { offsetX+ width, offsetY + height }, { 255, 255, 0 }, 2);
                cvtColor(rgb[blue], rgb[blue], CV_GRAY2RGB);

                for(Vec3i c :circles)
                {
                    Point center = Point(c[0], c[1]);
                    // circle center
                    circle( rgb[blue], center, 1, Scalar(0,0,0), 3, cv::LINE_AA);
                    // circle outline
                    int radius = c[2];
                    circle( rgb[blue], center, radius + 1, Scalar(0,0,0), 3, cv::LINE_AA);
                    LOG_INFO(cString::Format("Radius: %d:",radius));
                }
                //draw blueLight
                if(m_drawBlueBox)
                {
                (rgb[blue]).copyTo((outputImage)(Rect(offsetX, offsetY, width, height)));
                }

                if(m_saveBluelightImgs){

                    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
                    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                    //std::cout << "TimeStamp: " << now_c<< '\n';
                    std::string timeStr = std::to_string(now_c);

                    cFilename dir = m_directory;
                    cString pathstr = static_cast<cString>(dir.GetPath());

                    string Filename;

                    vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                    compression_params.push_back(98);

                    Filename = pathstr + "_" + cString(timeStr.c_str()) + ".jpg";
                    imwrite(Filename,rgb[blue]); //grabbbin an Img

                }
            }

        }
    }

    //Debug Video
    if(m_showDebugVideo)
    {
        //draw property lines
        tInt32 low = m_useRoi ? static_cast<tInt32>(m_lowerThreshold-(m_roiParams.roiY)) : static_cast<tInt32>(m_lowerThreshold);
        tInt32 up = m_useRoi ? static_cast<tInt32>(m_upperThreshold-(m_roiParams.roiY)) : static_cast<tInt32>(m_upperThreshold);

        line(outputImage, Point(0, low), Point(1910, low), Scalar(255, 255, 0), 3, cv::LINE_8, 0);
        line(outputImage, Point(0, up), Point(1910, up), Scalar(0, 255, 0), 3, cv::LINE_8, 0);
        //        line(outputImage, Point(50, 750), Point(50, std::min(0, static_cast<int>(750-1080*m_minHeightOfCar))), Scalar(255, 255, 0), 3, LINE_8, 0);
        //        line(outputImage, Point(125, 750), Point(125, static_cast<int>(750-1080*m_minHeightOfBarbie)), Scalar(0, 255, 255), 3, LINE_8, 0);
        //        line(outputImage, Point(175, 750), Point(175, std::min(0, static_cast<int>(750-1080*m_maxHeightOfBarbie))), Scalar(0, 255, 255), 3, LINE_8, 0);
        //        line(outputImage, Point(250, 750), Point(250, std::min(0, static_cast<int>(750-1080*m_minHeightOfBaby))), Scalar(255, 0, 255), 3, LINE_8, 0);
        //        line(outputImage, Point(300, 750), Point(300, std::min(0, static_cast<int>(400+1080*m_maxHeightOfBaby))), Scalar(255, 0, 255), 3, LINE_8, 0);

        //update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sCurrentFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }


    RETURN_NOERROR;
}
