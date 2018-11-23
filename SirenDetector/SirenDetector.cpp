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
#include "SirenDetector.h"
#include "aadc_structs.h"
#include "ADTF3_helper.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_SIREN_DETECTOR_DATA_TRIGGERED_FILTER,"Siren",cSiren,adtf::filter::pin_trigger({ "audio" }));

cSiren::cSiren()
{
	RegisterPropertyVariable("Sample Rate", m_sampleRate);
	RegisterPropertyVariable("Peak Min Frequency", m_minFreq);
	RegisterPropertyVariable("Peak Max Frequency", m_maxFreq);
    RegisterPropertyVariable("Magnitude Threshold Peak Detection", m_magnitudeThreshold);

	object_ptr<IStreamType> pTypeAudio = make_object_ptr<cStreamType>(stream_meta_type_audio());
	Register(m_oAudioIn, "audio", pTypeAudio);
	
	object_ptr<IStreamType> pTypeBoolSignalValue;
	if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_boolFactory))
	{
		adtf_ddl::access_element::find_index(m_boolFactory, cString("bValue"), m_ddlBoolId.bValue);
	}
	else
	{
		LOG_WARNING("No mediadescription for tBoolSignalValue found!");
	}
    Register(m_oDetectionOut, "detection", pTypeBoolSignalValue);
}

tResult cSiren::Configure()
{
	RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

	essentia::init();
	essentia::standard::AlgorithmFactory& factory = essentia::standard::AlgorithmFactory::instance();

	m_algWindow = factory.create("Windowing", "type", "blackmanharris62");
	m_algSpectrum = factory.create("Spectrum");
    m_algPeaks = factory.create("SpectralPeaks", /*"minFrequency", *m_minFreq, "maxFrequency", *m_maxFreq, */"sampleRate", *m_sampleRate, "magnitudeThreshold", *m_magnitudeThreshold);

    m_algWindow->input("frame").set(m_bufAudio);
    m_algWindow->output("frame").set(m_bufWindow);
    m_algSpectrum->input("frame").set(m_bufWindow);
	m_algSpectrum->output("spectrum").set(m_bufSpectrum);
	m_algPeaks->input("spectrum").set(m_bufSpectrum);
    m_algPeaks->output("frequencies").set(m_bufFrequencies);
    m_algPeaks->output("magnitudes").set(m_bufMagnitudes);
	RETURN_NOERROR;
}

tResult cSiren::Process(tTimeStamp tmTimeOfTrigger)
{
	object_ptr<const ISample> pReadSample;
	while (IS_OK(m_oAudioIn.GetNextSample(pReadSample)))
	{
		object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
		if (IS_OK(pReadSample->Lock(pReadBuffer)))
		{
            m_bufAudio.clear();
            const tUInt8* ptr = static_cast<const tUInt8*>(pReadBuffer->GetPtr());
            for (tSize i = 0; i < pReadBuffer->GetSize() / sizeof(tUInt8); i++)
			{
                m_bufAudio.push_back(static_cast<Real>(ptr[i]));
			}


//            cString sample = cString::Format("data (%d):", m_bufAudio.size());
//            for (Real r : m_bufAudio)
//            {
//                sample += cString::Format(" %f", r);
//            }
//            LOG_INFO(sample);

			m_algWindow->compute();
			m_algSpectrum->compute();
			m_algPeaks->compute();

            if (!m_bufFrequencies.empty())
			{
                cString msg = cString::Format("Detected! Peaks (%d):", m_bufFrequencies.size());
                for (tSize i = 0; i < m_bufFrequencies.size(); i++)
				{
                    msg += cString::Format(" %f Hz, %f;", m_bufFrequencies[i], m_bufMagnitudes[i]);
				}
				LOG_INFO(msg);

				object_ptr<ISample> pBoolSample;
				if (IS_OK(alloc_sample(pBoolSample)))
				{
                    auto oCodec = m_boolFactory.MakeCodecFor(pBoolSample);

					RETURN_IF_FAILED(oCodec.IsValid());
					RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlBoolId.bValue, tTrue));

					m_oDetectionOut << pBoolSample << flush << trigger;
				}
			}
		}
	}
    RETURN_NOERROR;
}
