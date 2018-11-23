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

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;
using namespace adtf::filter;
using namespace adtf::mediadescription;
using namespace adtf_ddl;


using namespace cv;
using namespace cv::dnn;
using namespace std;

#include "ImgCollector.h"
#include <ADTF3_OpenCV_helper.h>
#include <iostream>
//#include "ADTF3_helper.h"

#include <string>

#include <chrono>
#include <ctime>

ADTF_PLUGIN(LABEL_ImgCollector_FILTER, cImgCollectorDetection)



cImgCollectorDetection::cImgCollectorDetection()
{

    m_active = false;

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

    filter_create_pin(*this, m_oActivateIn, "Activate",pTypeBoolSignalValue);

    //register properties
    RegisterPropertyVariable("show DebugVideo", m_showDebugVideo);
    RegisterPropertyVariable("use PNG (else jpeg)", m_usePNG);
    RegisterPropertyVariable("Directory for saving Pictures", m_directory);

}

tResult cImgCollectorDetection::Init(tInitStage eStage)
{
    // check if all necessary files exist

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    cFilename filenameDirectory = m_directory;
    adtf::services::ant::adtf_resolve_macros(filenameDirectory);
    if (!cFileSystem::Exists(filenameDirectory))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, cString::Format("Directory could not be loaded from %s", filenameDirectory.GetPtr()));
    }

    RETURN_NOERROR;
}

tResult cImgCollectorDetection::Shutdown(tInitStage eStage)
{
    RETURN_NOERROR;
}

tResult cImgCollectorDetection::Process(tTimeStamp tmTimeOfTrigger)
{
    if(tmTimeOfTrigger-m_lastTimeStamp > 1000000 * m_collectionRate)
    {
        m_lastTimeStamp = tmTimeOfTrigger;

        object_ptr<const ISample> pReadSample;

        Mat inputImage;
        Mat rgbImage;

        if (IS_OK(m_oReader.GetLastSample(pReadSample)))
        {
            object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
            //lock read buffer

            if (IS_OK(pReadSample->Lock(pReadBuffer)))
            {
                //create a opencv matrix from the media sample buffer
                inputImage = Mat(cv::Size(m_sCurrentFormat.m_ui32Width, m_sCurrentFormat.m_ui32Height),
                                 CV_8UC3, (uchar*)pReadBuffer->GetPtr());
            }
        }
        else
        {

            RETURN_NOERROR;
        }

        object_ptr<const ISample> pActivateSample;
        if (IS_OK(m_oActivateIn.GetNextSample(pActivateSample)))
        {
            auto oDecoder = m_BoolSignalValueSampleFactory.MakeDecoderFor(*pActivateSample);

            RETURN_IF_FAILED(oDecoder.IsValid());
            tBool active = tFalse;
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlBoolSignalValueId.bValue, &active));
            if(m_active != active)
            {
                LOG_INFO(cString::Format("Status now: %d:", active));
                m_active = active;
            }
        }

        if(m_active){
            std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::cout << "TimeStamp: " << now_c<< '\n';
            std::string timeStr = std::to_string(now_c);

            cFilename dir = m_directory;
            cString pathstr = static_cast<cString>(dir.GetPath());

            string Filename;
            if(!m_usePNG)
            {

                vector<int> compression_params;
                compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                compression_params.push_back(98);

                Filename = pathstr + "_" + timeStr + ".jpg";
                cvtColor(inputImage,rgbImage, COLOR_BGR2RGB);
                imwrite(Filename,rgbImage); //grabin an Img
            }
            else
            {
                Filename = pathstr + "_" + timeStr + ".png";
                cvtColor(inputImage,rgbImage, COLOR_BGR2RGB);
                imwrite(Filename,rgbImage); //grabin an Img
            }
        }

        if(m_showDebugVideo)
        {
            //update output format if matrix size does not fit to
            if (inputImage.total() * inputImage.elemSize() != m_sCurrentFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oWriter, inputImage);
            }
            // write to pin
            writeMatToPin(m_oWriter, inputImage, m_pClock->GetStreamTime());
        }
    }
    RETURN_NOERROR;
}
