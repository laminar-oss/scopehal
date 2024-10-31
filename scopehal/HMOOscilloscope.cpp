/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/*
 * Current State
 * =============
 * - Digital channels not implemented
 * - Only basic edge trigger supported. Coupling, hysteresis, B trigger not implemented
 *
 * RS Oscilloscope driver parts (c) 2021 Francisco Sedano, tested on RTM3004
 */


#include "scopehal.h"
#include "HMOOscilloscope.h"
#include "EdgeTrigger.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

HMOOscilloscope::HMOOscilloscope(SCPITransport* transport)
	: SCPIDevice(transport)
	, SCPIInstrument(transport)
	, m_triggerArmed(false)
	, m_triggerOneShot(false)
	, m_sampleRateValid(false)
	, m_sampleDepthValid(false)
{
	//Last digit of the model number is the number of channels
	int model_number = atoi(m_model.c_str() + 3);	//FIXME: are all series IDs 3 chars e.g. "RTM"?
	int nchans = model_number % 10;

	for(int i=0; i<nchans; i++)
	{
		//Hardware name of the channel
		string chname = string("CHAN1");
		chname[4] += i;

		//Color the channels based on R&S's standard color sequence (yellow-green-orange-bluegray)
		string color = "#ffffff";
		switch(i)
		{
			case 0:
				color = "#ffff00";
				break;

			case 1:
				color = "#00ff00";
				break;

			case 2:
				color = "#ff8000";
				break;

			case 3:
				color = "#8080ff";
				break;
		}

		//Create the channel
		auto chan = new OscilloscopeChannel(
			this,
			chname,
			color,
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_VOLTS),
			Stream::STREAM_TYPE_ANALOG,
			i);
		m_channels.push_back(chan);
		chan->SetDefaultDisplayName();

		//Request all points when we download
		m_transport->SendCommand(chname + ":DATA:POIN MAX");
	}
	m_analogChannelCount = nchans;

	//Add the external trigger input
	m_extTrigChannel = new OscilloscopeChannel(
		this,
		"EX",
		"",
		Unit(Unit::UNIT_FS),
		Unit(Unit::UNIT_VOLTS),
		Stream::STREAM_TYPE_TRIGGER,
		m_channels.size());
	m_channels.push_back(m_extTrigChannel);
	m_extTrigChannel->SetDefaultDisplayName();

	//Configure transport format to raw IEEE754 float, little endian
	//TODO: if instrument internal is big endian, skipping the bswap might improve download performance?
	//Might be faster to do it on a beefy x86 than the embedded side of things.
	m_transport->SendCommand("FORM:DATA REAL");
	m_transport->SendCommand("FORM:BORD LSBFirst");

	//See what options we have
	m_transport->SendCommand("*OPT?");
	string reply = m_transport->ReadReply();
	vector<string> options;
	string opt;
	for(unsigned int i=0; i<reply.length(); i++)
	{
		if(reply[i] == 0)
		{
			options.push_back(opt);
			break;
		}

		else if(reply[i] == ',')
		{
			options.push_back(opt);
			opt = "";
		}

		else
			opt += reply[i];
	}
	if(opt != "")
		options.push_back(opt);

	//Print out the option list and do processing for each
	LogDebug("Installed options:\n");
	if(options.empty())
		LogDebug("* None\n");
	for(auto sopt : options)
	{
		LogDebug(" * %s ",sopt.c_str());
		if(opt == "B243")
			LogDebug("(350 MHz bandwidth upgrade for RTM3004)\n");
		else if(sopt == "K1")
			LogDebug("(SPI Bus)\n");
		else if(sopt == "K2")
			LogDebug("(UART / RS232)\n");
		else if(sopt == "K3")
			LogDebug("(CAN)\n");
		else if(sopt == "K5")
			LogDebug("(Audio signals)\n");
		else if(sopt == "B1") {
			// TODO add digital channels
			LogDebug("(Mixed signal, 16 channels)\n");
		}
		else if(sopt == "K31")
			LogDebug("(Power analysis)\n");
		else if(sopt == "K6")
			LogDebug("(MIL-1553)\n");
		else if(sopt == "K7")
			LogDebug("(ARINC 429)\n");
		else if(sopt == "K15")
			LogDebug("(History)\n");
		else if(sopt == "K18")
			LogDebug("(Spectrum analysis and spectrogram)\n");
		else if(sopt == "B6")
			LogDebug("(Signal generation)\n");
		else if(sopt == "B2410")
			LogDebug("(Bandwidth 1 GHz)\n");
		else if(sopt == "K36")
			LogDebug("(Frequency response analysis)\n");
		else
			LogDebug("(unknown)\n");
	}
}

HMOOscilloscope::~HMOOscilloscope()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

string HMOOscilloscope::GetDriverNameInternal()
{
	return "hmo";
}

unsigned int HMOOscilloscope::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t HMOOscilloscope::GetInstrumentTypesForChannel(size_t /*i*/) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device interface functions

void HMOOscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	m_channelOffsets.clear();
	m_channelVoltageRanges.clear();
	m_channelsEnabled.clear();
	m_channelCouplings.clear();
	m_channelAttenuations.clear();

	delete m_trigger;
	m_trigger = NULL;
}

bool HMOOscilloscope::IsChannelEnabled(size_t i)
{
	//ext trigger should never be displayed
	if(i == m_extTrigChannel->GetIndex())
		return false;

	//TODO: handle digital channels, for now just claim they're off
	if(i >= m_analogChannelCount)
		return false;

	lock_guard<recursive_mutex> lock(m_cacheMutex);

	if(m_channelsEnabled.find(i) != m_channelsEnabled.end())
		return m_channelsEnabled[i];

	lock_guard<recursive_mutex> lock2(m_mutex);

	m_transport->SendCommand(m_channels[i]->GetHwname() + ":STAT?");
	string reply = m_transport->ReadReply();
	if(reply == "OFF" || reply == "0")
	{
		m_channelsEnabled[i] = false;
		return false;
	}
	else
	{
		m_channelsEnabled[i] = true;
		return true;
	}
}

void HMOOscilloscope::EnableChannel(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(m_channels[i]->GetHwname() + ":STAT ON");

	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	m_channelsEnabled[i] = true;
}

void HMOOscilloscope::DisableChannel(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(m_channels[i]->GetHwname() + ":STAT OFF");

	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	m_channelsEnabled[i] = false;
}

vector<OscilloscopeChannel::CouplingType> HMOOscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_DC_50);
	ret.push_back(OscilloscopeChannel::COUPLE_GND);
	return ret;
}

OscilloscopeChannel::CouplingType HMOOscilloscope::GetChannelCoupling(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelCouplings.find(i) != m_channelCouplings.end())
			return m_channelCouplings[i];
	}

	string reply;
	{
		lock_guard<recursive_mutex> lock(m_mutex);

		m_transport->SendCommand(m_channels[i]->GetHwname() + ":COUP?");
		reply = m_transport->ReadReply();
	}
	OscilloscopeChannel::CouplingType coupling;

	if(reply == "ACLimit" || reply == "ACL")
		coupling = OscilloscopeChannel::COUPLE_AC_1M;
	else if(reply == "DCLimit" || reply == "DCL")
		coupling = OscilloscopeChannel::COUPLE_DC_1M;
	else if(reply == "GND")
		coupling = OscilloscopeChannel::COUPLE_GND;
	else if(reply == "DC")
		coupling = OscilloscopeChannel::COUPLE_DC_50;
	else
	{
		LogWarning("invalid coupling value\n");
		coupling = OscilloscopeChannel::COUPLE_DC_50;
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = coupling;
	return coupling;
}

void HMOOscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		switch(type)
		{
			case OscilloscopeChannel::COUPLE_DC_50:
				m_transport->SendCommand(m_channels[i]->GetHwname() + ":COUP DC");
				break;

			case OscilloscopeChannel::COUPLE_AC_1M:
				m_transport->SendCommand(m_channels[i]->GetHwname() + ":COUP ACLimit");
				break;

			case OscilloscopeChannel::COUPLE_DC_1M:
				m_transport->SendCommand(m_channels[i]->GetHwname() + ":COUP DCLimit");
				break;

			case OscilloscopeChannel::COUPLE_GND:
				m_transport->SendCommand(m_channels[i]->GetHwname() + ":COUP GND");
				break;

			default:
				LogError("Invalid coupling for channel\n");
		}
	}
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = type;
}

double HMOOscilloscope::GetChannelAttenuation(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelAttenuations.find(i) != m_channelAttenuations.end())
			return m_channelAttenuations[i];
	}
	// FIXME Don't know SCPI to get this, relying on cache
	return 1;
}

void HMOOscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelAttenuations[i] = atten;
	}

	lock_guard<recursive_mutex> lock(m_mutex);

	char cmd[128];
	snprintf(cmd, sizeof(cmd), "PROB%zd:SET:ATT:MAN ", m_channels[i]->GetIndex()+1);
	PushFloat(cmd, atten);
}

unsigned int HMOOscilloscope::GetChannelBandwidthLimit(size_t /*i*/)
{
	/*
	lock_guard<recursive_mutex> lock(m_mutex);

	m_transport->SendCommand(m_channels[i]->GetHwname() + ":BWL?");
	string reply = m_transport->ReadReply();
	if(reply == "20M")
		return 20;
	else
		return 0;
	*/

	LogWarning("HMOOscilloscope::GetChannelBandwidthLimit unimplemented\n");
	return 0;
}

void HMOOscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
	LogWarning("HMOOscilloscope::SetChannelBandwidthLimit unimplemented\n");
}

float HMOOscilloscope::GetChannelVoltageRange(size_t i, size_t /*stream*/)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelVoltageRanges.find(i) != m_channelVoltageRanges.end())
			return m_channelVoltageRanges[i];
	}

	lock_guard<recursive_mutex> lock2(m_mutex);

	m_transport->SendCommand(m_channels[i]->GetHwname() + ":RANGE?");

	string reply = m_transport->ReadReply();
	float range;
	sscanf(reply.c_str(), "%f", &range);
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelVoltageRanges[i] = range;
	return range;
}

void HMOOscilloscope::SetChannelVoltageRange(size_t i, size_t /*stream*/, float range)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelVoltageRanges[i] = range;
	}

	lock_guard<recursive_mutex> lock(m_mutex);
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:RANGE %.4f", m_channels[i]->GetHwname().c_str(), range);
	m_transport->SendCommand(cmd);
}

OscilloscopeChannel* HMOOscilloscope::GetExternalTrigger()
{
	//FIXME
	return NULL;
}

float HMOOscilloscope::GetChannelOffset(size_t i, size_t /*stream*/)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelOffsets.find(i) != m_channelOffsets.end())
			return m_channelOffsets[i];
	}

	lock_guard<recursive_mutex> lock2(m_mutex);

	m_transport->SendCommand(m_channels[i]->GetHwname() + ":OFFS?");

	string reply = m_transport->ReadReply();
	float offset;
	sscanf(reply.c_str(), "%f", &offset);
	offset = -offset;
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
	return offset;
}

void HMOOscilloscope::SetChannelOffset(size_t i, size_t /*stream*/, float offset)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelOffsets[i] = offset;
	}

	lock_guard<recursive_mutex> lock(m_mutex);
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:OFFS %.4f", m_channels[i]->GetHwname().c_str(), -offset);
	m_transport->SendCommand(cmd);
}

Oscilloscope::TriggerMode HMOOscilloscope::PollTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);

	m_transport->SendCommand("ACQ:STAT?");
	string stat = m_transport->ReadReply();

	if(stat == "RUN")
		return TRIGGER_MODE_RUN;
	else if( (stat == "STOP") || (stat == "BRE") )
		return TRIGGER_MODE_STOP;
	else /*if(stat == "COMP")*/
	{
		m_triggerArmed = false;
		return TRIGGER_MODE_TRIGGERED;
	}
}

bool HMOOscilloscope::AcquireData()
{
	//LogDebug("Acquiring data\n");

	lock_guard<recursive_mutex> lock(m_mutex);
	LogIndenter li;

	double xstart;
	double xstop;
	size_t length = 0;
	int ignored;
	map<int, vector<UniformAnalogWaveform*> > pending_waveforms;
	bool any_data = false;
	for(size_t i=0; i<m_analogChannelCount; i++)
	{
		if(!IsChannelEnabled(i))
			continue;

		//This is basically the same function as a LeCroy WAVEDESC, but much less detailed
		int rc = 0;
		while (length == 0) {
			m_transport->SendCommand(m_channels[i]->GetHwname() + ":DATA:HEAD?");
			string reply = m_transport->ReadReply();
			rc = sscanf(reply.c_str(), "%lf,%lf,%zu,%d", &xstart, &xstop, &length, &ignored);
			if (rc != 4) {
				break;
			}
		}
		if (rc != 4 || length == 0) {
			/* No data - Skip query the scope and move on */
			pending_waveforms[i].push_back(NULL);
			continue;
		}
		any_data = true;
		//Figure out the sample rate
		double capture_len_sec = xstop - xstart;
		double sec_per_sample = capture_len_sec / length;
		int64_t fs_per_sample = round(sec_per_sample * FS_PER_SECOND);
		//LogDebug("%ld fs/sample\n", fs_per_sample);

		float* temp_buf = new float[length];

		//Set up the capture we're going to store our data into (no high res timer on R&S scopes)
		auto cap = new UniformAnalogWaveform;
		cap->m_timescale = fs_per_sample;
		cap->m_triggerPhase = 0;
		cap->m_startTimestamp = time(NULL);
		double t = GetTime();
		cap->m_startFemtoseconds = (t - floor(t)) * FS_PER_SECOND;

		//Ask for the data
		m_transport->SendCommand(m_channels[i]->GetHwname() + ":DATA?");

		//Read and discard the length header
		char tmp[16] = {0};
		m_transport->ReadRawData(2, (unsigned char*)tmp);
		int num_digits = atoi(tmp+1);
		m_transport->ReadRawData(num_digits, (unsigned char*)tmp);
		//int actual_len = atoi(tmp);

		//Read the actual data.
		//Super easy, it comes across the wire in IEEE754 already!
		cap->Resize(length);
		cap->PrepareForCpuAccess();
		m_transport->ReadRawData(length*sizeof(float), (unsigned char*)cap->m_samples.GetCpuPointer());
		cap->MarkSamplesModifiedFromCpu();

		//Discard trailing newline
		m_transport->ReadRawData(1, (unsigned char*)tmp);

		//Done, update the data
		pending_waveforms[i].push_back(cap);

		//Clean up
		delete[] temp_buf;
	}
	if (!any_data)
	{
		LogDebug("Skip update, no data from scope\n");
		//Re-arm the trigger if not in one-shot mode
		if(!m_triggerOneShot)
		{
			m_transport->SendCommand("SING");
			m_triggerArmed = true;
		}

		return false;
	}
	//Now that we have all of the pending waveforms, save them in sets across all channels
	m_pendingWaveformsMutex.lock();
	size_t num_pending = 1;	//TODO: segmented capture support
	for(size_t i=0; i<num_pending; i++)
	{
		SequenceSet s;
		for(size_t j=0; j<m_analogChannelCount; j++)
		{
			if(IsChannelEnabled(j))
				s[GetOscilloscopeChannel(j)] = pending_waveforms[j][i];
		}
		m_pendingWaveforms.push_back(s);
	}
	m_pendingWaveformsMutex.unlock();

	//TODO: support digital channels

	//Re-arm the trigger if not in one-shot mode
	if(!m_triggerOneShot)
	{
		m_transport->SendCommand("SING");
		m_triggerArmed = true;
	}

	//LogDebug("Acquisition done\n");
	return true;
}

void HMOOscilloscope::Start()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand("SING");
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void HMOOscilloscope::StartSingleTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand("SING");
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void HMOOscilloscope::Stop()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand("STOP");
	m_triggerArmed = false;
	m_triggerOneShot = true;
}

void HMOOscilloscope::ForceTrigger()
{
	LogError("HMOOscilloscope::ForceTrigger not implemented\n");
}

bool HMOOscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

vector<uint64_t> HMOOscilloscope::GetSampleRatesNonInterleaved()
{
	LogWarning("HMOOscilloscope::GetSampleRatesNonInterleaved unimplemented\n");

	//FIXME
	vector<uint64_t> ret;
	return ret;
}

vector<uint64_t> HMOOscilloscope::GetSampleRatesInterleaved()
{
	return GetSampleRatesNonInterleaved();
}

set<Oscilloscope::InterleaveConflict> HMOOscilloscope::GetInterleaveConflicts()
{
	set<Oscilloscope::InterleaveConflict> ret;

	for(size_t i=0; i<m_analogChannelCount; i++)
		ret.emplace(InterleaveConflict(GetOscilloscopeChannel(i), GetOscilloscopeChannel(i)));

	return ret;
}

vector<uint64_t> HMOOscilloscope::GetSampleDepthsNonInterleaved()
{
	//FIXME
	vector<uint64_t> ret;
	return ret;
}

vector<uint64_t> HMOOscilloscope::GetSampleDepthsInterleaved()
{
	LogWarning("HMOOscilloscope::GetSampleDepthsInterleaved unimplemented\n");

	//FIXME
	vector<uint64_t> ret;
	return ret;
}

uint64_t HMOOscilloscope::GetSampleRate()
{
	if(m_sampleRateValid)
	{
		LogDebug("GetSampleRate() queried and returned cached value %" PRIu64 "\n", m_sampleRate);
		return m_sampleRate;
	}

	m_sampleRate = stod(m_transport->SendCommandQueuedWithReply("ACQUIRE:SRATE?"));
	m_sampleRateValid = true;

	LogDebug("GetSampleRate() queried and got new value %" PRIu64 "\n", m_sampleRate);

	return 1;
}

uint64_t HMOOscilloscope::GetSampleDepth()
{
	if(m_sampleDepthValid)
	{
		LogDebug("GetSampleDepth() queried and returned cached value %" PRIu64 "\n", m_sampleDepth);
		return m_sampleDepth;
	}

	GetSampleRate();

	m_sampleDepth = stod(m_transport->SendCommandQueuedWithReply("TIMEBASE:RANGE?")) * (double)m_sampleRate;
	m_sampleDepthValid = true;

	LogDebug("GetSampleDepth() queried and got new value %" PRIu64 "\n", m_sampleDepth);

	return 1;
}

void HMOOscilloscope::SetSampleDepth(uint64_t depth)
{
	GetSampleRate();

	//Update the cache
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_sampleDepth = depth;
		m_sampleDepthValid = true;
	}

	LogDebug("SetSampleDepth() setting to %" PRIu64 "\n", depth);

	m_transport->SendCommandQueued(string("TIMEBASE:RANGE ") + to_string((double)depth / (double)m_sampleRate));
}

void HMOOscilloscope::SetSampleRate(uint64_t rate)
{
	//Update the cache
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_sampleRate = rate;
		m_sampleRateValid = true;
	}

	LogDebug("SetSampleRate() setting to %" PRIu64 "\n", rate);

	m_transport->SendCommandQueued(string("ACQUIRE:SRATE ") + to_string(rate));

	SetSampleDepth(m_sampleDepth);
}

void HMOOscilloscope::SetTriggerOffset(int64_t /*offset*/)
{
	//FIXME
}

int64_t HMOOscilloscope::GetTriggerOffset()
{
	//FIXME
	return 0;
}

bool HMOOscilloscope::IsInterleaving()
{
	return false;
}

bool HMOOscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

void HMOOscilloscope::PullTrigger()
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//TODO: Figure out trigger type
	if(true)
		PullEdgeTrigger();

	//Unrecognized trigger type
	else
	{
		LogWarning("Unknown trigger type\n");
		delete m_trigger;
		m_trigger = NULL;
		return;
	}
}

/**
	@brief Reads settings for an edge trigger from the instrument
 */
void HMOOscilloscope::PullEdgeTrigger()
{
	//Clear out any triggers of the wrong type
	if( (m_trigger != NULL) && (dynamic_cast<EdgeTrigger*>(m_trigger) != NULL) )
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new EdgeTrigger(this);
	EdgeTrigger* et = dynamic_cast<EdgeTrigger*>(m_trigger);

	lock_guard<recursive_mutex> lock(m_mutex);

	//Source
	//This is a bit annoying because the hwname's used here are DIFFERENT than everywhere else!
	m_transport->SendCommand("TRIG:A:SOUR?");
	string reply = m_transport->ReadReply();
	if(reply.find("CH") == 0)
		et->SetInput(0, StreamDescriptor(GetOscilloscopeChannel(atoi(reply.c_str()+2) - 1), 0), true);
	else if(reply == "EXT")
		et->SetInput(0, StreamDescriptor(m_extTrigChannel, 0), true);
	else
		LogWarning("Unknown trigger source %s\n", reply.c_str());

	//Level
	m_transport->SendCommand("TRIG:A:LEV?");
	reply = m_transport->ReadReply();
	et->SetLevel(stof(reply));

	//TODO: Edge slope
}

void HMOOscilloscope::PushTrigger()
{
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(et)
		PushEdgeTrigger(et);

	else
		LogWarning("Unknown trigger type (not an edge)\n");
}

/**
	@brief Pushes settings for an edge trigger to the instrument
 */
void HMOOscilloscope::PushEdgeTrigger(EdgeTrigger* trig)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	char tmp[81];

	// They use CH1, CH2 and so on here :-(

	snprintf(tmp, sizeof(tmp), "TRIG:A:SOUR CH%zd", trig->GetInput(0).m_channel->GetIndex()+1);
	m_transport->SendCommand(tmp);

	snprintf(tmp, sizeof(tmp), "TRIG:A:LEV%zd %f", trig->GetInput(0).m_channel->GetIndex()+1, trig->GetLevel());
	m_transport->SendCommand(tmp);

	string slope_str;
	switch(trig->GetType())
	{
		case EdgeTrigger::EDGE_RISING:
			slope_str = "POS";
			break;
		case EdgeTrigger::EDGE_FALLING:
			slope_str = "NEG";
			break;
		case EdgeTrigger::EDGE_ANY:
			slope_str = "EITH";
			break;
		default:
			LogDebug("Unsupported edge type: %d\n", trig->GetType());
			return;
	}
	m_transport->SendCommand("TRIG:A:EDGE:SLOP " + slope_str);
}

/**
	@brief Sends float, assumes transport is already mutexed
 */

void HMOOscilloscope::PushFloat(string path, float f)
{
	m_transport->SendCommand(path + " " + to_string_sci(f));
}
