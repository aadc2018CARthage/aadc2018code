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
#include "Capture.h"
#include "aadc_structs.h"
#include "ADTF3_helper.h"
#include "ADTF3_OpenCV_helper.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_LANEKEEPING_DATA_TRIGGERED_FILTER,"Capture",cCapture,adtf::filter::pin_trigger({ "VideoIn" }));

cCapture::cCapture()
{
	//create and set inital input format type
	m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
	adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
	set_stream_type_image_format(*pType, m_sImageFormat);

	//Register input pin
	Register(m_oVideoIn, "VideoIn", pType);
	//Register output pin
    Register(m_oVideoOut, "VideoOut", pType);

	//register callback for type changes
	m_oVideoIn.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
	{
		return ChangeType(m_oVideoIn, m_sImageFormat, *pType.Get(), m_oVideoOut);
    });

    RegisterPropertyVariable("ROI Offset X", roi.offsetX);
    RegisterPropertyVariable("ROI Offset Y", roi.offsetY);
    RegisterPropertyVariable("ROI Width", roi.width);
    RegisterPropertyVariable("ROI Height", roi.height);
    RegisterPropertyVariable("File", m_file);

    writer.open(String((*m_file).GetPtr()), VideoWriter::fourcc('M', 'J', 'P', 'G'), 10.0, { roi.width, roi.height });
}

tResult cCapture::Configure()
{
	//get clock object
	RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

	RETURN_NOERROR;
}

tResult cCapture::Process(tTimeStamp tmTimeOfTrigger)
{
    if (!writer.isOpened())
    {
        LOG_INFO("failed to open video file");
        RETURN_NOERROR;
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
            Rect r(roi.offsetX, roi.offsetY, roi.width, roi.height);
            Mat _roi = inputImage(r);
            writer << _roi;

            rectangle(inputImage, r, { 255, 0, 0 }, 5);
            writeMatToPin(m_oVideoOut, inputImage, m_pClock->GetStreamTime());
		}
	}
	RETURN_NOERROR;
}
