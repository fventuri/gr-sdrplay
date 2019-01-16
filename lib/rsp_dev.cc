/* -*- c++ -*- */
/*
 * gr-sdrplay Copyright 2018 HB9FXQ, Frank Werner-Krippendorf.
 *            Copyright 2019 Franco Venturi - changes for SDRplay API version 3
 *
 * Credits for the rsp_dev class goes go to:
 * gr-osmosdr Copyright 2018 Jeff Long <willcode4@gmail.com> of the gr-osmosdr-fork!
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "rsp_dev.h"
#include <gnuradio/io_signature.h>

#include <boost/assign.hpp>
#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono.hpp>

#include <iostream>
#include <string>
#include <mutex>
#include <sdrplay_api.h>

namespace gr
{
namespace sdrplay
{

#define MAX_SUPPORTED_DEVICES 4

using namespace boost::assign;

// Index by sdrplay_api_Bw_MHzT
static std::vector<double> bandwidths = {
    0, // Dummy
    200e3,
    300e3,
    600e3,
    1536e3,
    5000e3,
    6000e3,
    7000e3,
    8000e3};

static std::string hwName(int hwVer)
{
    if (hwVer == SDRPLAY_RSP1_ID)
        return "RSP1";
    if (hwVer == SDRPLAY_RSP2_ID)
        return "RSP2";
    if (hwVer == SDRPLAY_RSP1A_ID)
        return "RSP1A";
    if (hwVer == SDRPLAY_RSPduo_ID)
        return "RSPduo";
    return "UNK";
}

int rsp_dev::_refcount = 0;

void rsp_dev::
    list_available_rsp_devices()
{

    unsigned int _devIndex = 0;
    unsigned int numDevices = 0;

    sdrplay_api_DeviceT mirDevices[MAX_SUPPORTED_DEVICES];
    sdrplay_api_GetDevices(mirDevices, &numDevices, MAX_SUPPORTED_DEVICES);

    if (_devIndex + 1 > numDevices)
    {
        std::cerr << "Failed to open SDRplay device " << std::endl;
        throw std::runtime_error("Failed to open SDRplay device ");
    }

    for (unsigned int i = 0; i < numDevices; i++)
    {

        std::cerr << "RSP devIndex: [" << i << "] " << hwName(mirDevices[i].hwVer) << " " << mirDevices[i].SerNo << std::endl;
    }
}

rsp_dev::rsp_dev()
{
    _samplesPerPacket = -1;
    _hwVer = -1;
    _biasT = 0;
    _bufferOffset = 0;
    _bufferSpaceRemaining = 0;
    _auto_gain = true;
    _gRdB = 40;
    _lna = 0;
    _bcastNotch = 0;
    _dabNotch = 0;
    _fsHz = 2e6;
    _decim = 1;
    _rfHz = 100e6;
    _bwType = sdrplay_api_BW_0_300;
    _ifType = sdrplay_api_IF_Zero;
    _loMode = sdrplay_api_LO_Auto;
    _dcMode = true;
    _iqMode = true;
    _tuner = 0;
    _rspduo_mode = sdrplay_api_RspDuoMode_Unknown;
    _buffer = NULL;
    _streaming = false;
    _reinit = false;
    _device = { 0 };
    _deviceParams = 0;
    _chParams = 0;
    _debug = 0;
    if (_refcount == 0)
        sdrplay_api_Open();
    _refcount++;
}

rsp_dev::~rsp_dev()
{
    if (_streaming)
    {
        stopStreaming();
    }
    _refcount--;
    if (_refcount == 0)
        sdrplay_api_Close();
}

// Called by sdrplay streamer thread when data is available
void rsp_dev::streamCallback(short *xi, short *xq,
                             sdrplay_api_StreamCbParamsT *params,
                             unsigned int numSamples, unsigned int reset)
{
    unsigned int i = 0;
    _reinit = false;

    boost::mutex::scoped_lock lock(_bufferMutex);

    while (i < numSamples)
    {
        if (!_streaming || _reinit)
        {
            return;
        }

        while (!_buffer)
        {
            if (boost::cv_status::timeout ==
                _bufferReady.wait_for(lock, boost::chrono::milliseconds(250)))
                return;
        }

        while ((i < numSamples) && (_bufferSpaceRemaining > 0))
        {
            _buffer[_bufferOffset] =
                gr_complex(float(xi[i]) / 32768.0, float(xq[i]) / 32768.0);
            i++;
            _bufferOffset++;
            _bufferSpaceRemaining--;
        }

        if (_bufferSpaceRemaining == 0)
        {
            _buffer = NULL;
            _bufferReady.notify_one();
        }
    }
}

// Callback wrapper
void rsp_dev::streamCallbackWrap(short *xi, short *xq,
                                 sdrplay_api_StreamCbParamsT *params,
                                 unsigned int numSamples, unsigned int reset,
                                 void *cbContext)
{
    rsp_dev *obj = (rsp_dev *)cbContext;
    obj->streamCallback(xi, xq,
                        params,
                        numSamples, reset);
}

// Callback wrapper to discard a stream
void rsp_dev::discardStreamCallbackWrap(short *xi, short *xq,
                                        sdrplay_api_StreamCbParamsT *params,
                                        unsigned int numSamples, unsigned int reset,
                                        void *cbContext)
{
    return;
}

// Called by sdrplay streamer thread when an event is received.
void rsp_dev::eventCallback(sdrplay_api_EventT eventId,
                            sdrplay_api_TunerSelectT tuner,
                            sdrplay_api_EventParamsT *params)
{

    if (eventId == sdrplay_api_GainChange)
    {
        int gRdB = params->gainParams.gRdB;
        int lnaGRdB = params->gainParams.lnaGRdB;
        if (_debug)
        {
            std::cerr << "GR change, BB+MIX -" << gRdB << "dB, LNA -" << lnaGRdB << std::endl;
        }
        // params->gainParams.curr is a calibrated gain value
    }
    else if (eventId == sdrplay_api_PowerOverloadChange)
    {
        if (params->powerOverloadParams.powerOverloadChangeType ==
            sdrplay_api_Overload_Detected)
        {
            // OVERLOAD DETECTED
        }
        else if (params->powerOverloadParams.powerOverloadChangeType ==
                 sdrplay_api_Overload_Corrected)
        {
            // OVERLOAD CORRECTED
        }
    }
}

// Callback wrapper
void rsp_dev::eventCallbackWrap(sdrplay_api_EventT eventId,
                                sdrplay_api_TunerSelectT tuner,
                                sdrplay_api_EventParamsT *params,
                                void *cbContext)
{
    rsp_dev *obj = (rsp_dev *)cbContext;
    obj->eventCallback(eventId, tuner, params);
}

void rsp_dev::startStreaming(void)
{
    if (_streaming)
    {
        return;
    }

    unsigned int numDevices;
    sdrplay_api_DeviceT mirDevices[MAX_SUPPORTED_DEVICES];
    sdrplay_api_LockDeviceApi();
    sdrplay_api_GetDevices(mirDevices, &numDevices, MAX_SUPPORTED_DEVICES);

    int _devIndex = 0;

    if (_deviceIndexOrSerial.length() > 2 /*It's a SerialNo*/)
    {

        bool match = false;

        for (unsigned int i = 0; i < numDevices; i++)
        {
            if (_deviceIndexOrSerial.compare(std::string(mirDevices[i].SerNo)) == 0)
            {
                std::cerr << "Found requested RSP with SerialNO: " << mirDevices[i].SerNo << std::endl;
                _devIndex = i;
                match = true;
                break;
            }
        }

        if (!match)
        {
            std::cerr << "FALLBACK TO DEV INDEX = !!!! Could NOT find RSP SerialNO: " << _deviceIndexOrSerial << std::endl;
        }

    }

    _device = mirDevices[_devIndex];
    _hwVer = _device.hwVer;

    sdrplay_api_SelectDevice(&_device);
    sdrplay_api_UnlockDeviceApi();

    std::cerr << "Using SDRplay " << hwName(_hwVer) << " "
              << mirDevices[_devIndex].SerNo << std::endl;

    sdrplay_api_GetDeviceParams(_device.dev, &_deviceParams);

    if (_hwVer == SDRPLAY_RSPduo_ID)
    {
        if (_device.rspDuoMode & sdrplay_api_RspDuoMode_Master)
        {
            _device.tuner = (sdrplay_api_TunerSelectT)_tuner;
            if (_rspduo_mode == sdrplay_api_RspDuoMode_Single_Tuner)
            {
                _device.rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
            }
            else if (_rspduo_mode == sdrplay_api_RspDuoMode_Dual_Tuner)
            {
	        _device.rspDuoMode = sdrplay_api_RspDuoMode_Master;
                _device.rspDuoSampleFreq = _fsHz;
            }
        }
        else
        {
            // Slave device case: the master has control of the parameters
        }
    }
    _chParams = _device.tuner == sdrplay_api_Tuner_B ? _deviceParams->rxChannelB : _deviceParams->rxChannelA;

    _streaming = true;

    set_biasT(_biasT);

    // Set first LO frequency
    set_lo_mode((int)_loMode);

    set_gain_mode(get_gain_mode());

    int gRdB = _gRdB;

    sdrplay_api_CallbackFnsT callbackFns;
    if (_device.tuner == sdrplay_api_Tuner_B)
    {
        callbackFns.StreamACbFn = discardStreamCallbackWrap;
        callbackFns.StreamBCbFn = streamCallbackWrap;
    }
    else
    {
        callbackFns.StreamACbFn = streamCallbackWrap;
        callbackFns.StreamBCbFn = discardStreamCallbackWrap;
    }
    callbackFns.EventCbFn = eventCallbackWrap;

    _chParams->tunerParams.gain.gRdB = gRdB;
    _deviceParams->devParams->fsFreq.fsHz = _fsHz;
    _chParams->tunerParams.rfFreq.rfHz = _rfHz;
    _chParams->tunerParams.bwType = _bwType;
    _chParams->tunerParams.ifType = _ifType;
    _chParams->tunerParams.gain.LNAstate = checkLNA(_lna);

    // Set decimation with halfband filter
    _chParams->ctrlParams.decimation.enable = _decim != 1;
    _chParams->ctrlParams.decimation.decimationFactor = _decim;
    _chParams->ctrlParams.decimation.wideBandSignal = 1;

    _chParams->ctrlParams.dcOffset.DCenable = _dcMode;
    _chParams->ctrlParams.dcOffset.IQenable = _iqMode;

    // Model-specific initialization
    if (_hwVer == SDRPLAY_RSP2_ID)
    {
        set_antenna(get_antenna());
        _chParams->rsp2TunerParams.rfNotchEnable = _bcastNotch;
    }
    else if (_hwVer == SDRPLAY_RSPduo_ID)
    {
        set_antenna(get_antenna());
        if (_antenna == "HIGHZ")
            _chParams->rspDuoTunerParams.tuner1AmNotchEnable = _bcastNotch;
        else
            _chParams->rspDuoTunerParams.rfNotchEnable = _bcastNotch;
        _chParams->rspDuoTunerParams.rfDabNotchEnable = _dabNotch;
    }
    else if (_hwVer == SDRPLAY_RSP1A_ID)
    {
        _deviceParams->devParams->rsp1aParams.rfNotchEnable = _bcastNotch;
        _deviceParams->devParams->rsp1aParams.rfDabNotchEnable = _dabNotch;
    }

    sdrplay_api_Init(_device.dev, &callbackFns, this);
}

void rsp_dev::stopStreaming(void)
{
    if (!_streaming)
        return;

    _streaming = false;

    sdrplay_api_Uninit(_device.dev);
    sdrplay_api_ReleaseDevice(&_device);
}

void rsp_dev::reinitDevice(int reason)
{
    // If no reason given, reinit everything
    if (reason == sdrplay_api_Update_None)
        reason = (sdrplay_api_Update_Tuner_Gr |
                  sdrplay_api_Update_Dev_Fs |
                  sdrplay_api_Update_Tuner_Frf |
                  sdrplay_api_Update_Tuner_BwType |
                  sdrplay_api_Update_Tuner_IfType |
                  sdrplay_api_Update_Tuner_LoMode |
                  sdrplay_api_Update_Rsp2_AmPortSelect |
                  sdrplay_api_Update_RspDuo_AmPortSelect);

    int gRdB;

    gRdB = _gRdB;

    // Tell stream CB to return
    _reinit = true;

    _chParams->tunerParams.gain.gRdB = gRdB;
    _deviceParams->devParams->fsFreq.fsHz = _fsHz;
    _chParams->tunerParams.rfFreq.rfHz = _rfHz;
    _chParams->tunerParams.bwType = _bwType;
    _chParams->tunerParams.ifType = _ifType;
    _chParams->tunerParams.gain.LNAstate = checkLNA(_lna);
    sdrplay_api_Update(_device.dev, _device.tuner, (sdrplay_api_ReasonForUpdateT)reason);

    // Set decimation with halfband filter
    _chParams->ctrlParams.decimation.enable = _decim != 1;
    _chParams->ctrlParams.decimation.decimationFactor = _decim;
    _chParams->ctrlParams.decimation.wideBandSignal = 1;

    _bufferReady.notify_one();
}

double rsp_dev::set_sample_rate(double rate)
{
    rate = std::min(std::max(rate, 62.5e3), 10e6);
    _fsHz = rate;

    // Decimation is required for rates below 2MS/s
    _decim = 1;
    while (_fsHz < 2e6)
    {
        _decim *= 2;
        _fsHz *= 2;
    }

    if (_streaming)
        reinitDevice(sdrplay_api_Update_Dev_Fs);

    return get_sample_rate();
}

double rsp_dev::get_sample_rate() const
{
    return _fsHz / _decim;
}

double rsp_dev::set_center_freq(double freq)
{
    _rfHz = freq;

    if (_streaming)
    {
        reinitDevice(sdrplay_api_Update_Tuner_Frf);
    }

    return get_center_freq();
}

double rsp_dev::get_center_freq() const
{
    return _rfHz;
}

bool rsp_dev::set_gain_mode(bool automatic)
{
    _auto_gain = automatic;
    if (_streaming)
    {
        if (automatic)
        {
            _chParams->ctrlParams.agc.enable = sdrplay_api_AGC_5HZ; /*TODO: expose argument */
            _chParams->ctrlParams.agc.setPoint_dBfs = -30; /*TODO: magic number */
        }
        else
        {
            _chParams->ctrlParams.agc.enable = sdrplay_api_AGC_DISABLE;
            _chParams->ctrlParams.agc.setPoint_dBfs = -30; /*TODO: magic number */
        }
        _chParams->ctrlParams.agc.knee_dBfs = 0;
        _chParams->ctrlParams.agc.decay_ms = 0;
        _chParams->ctrlParams.agc.decay_ms = 0;
        _chParams->ctrlParams.agc.hang_ms = 0;
        _chParams->ctrlParams.agc.syncUpdate = 0;
        _chParams->ctrlParams.agc.LNAstate = checkLNA(_lna);

        reinitDevice(sdrplay_api_Update_Ctrl_Agc);
    }

    return _auto_gain;
}

bool rsp_dev::get_gain_mode() const
{
    return _auto_gain;
}

int rsp_dev::checkLNA(int lna)
{
    // Maaaagic - see gain tables in API doc
    if (_hwVer == SDRPLAY_RSP1_ID)
    {
        lna = std::min(3, lna);
    }
    else if (_hwVer == SDRPLAY_RSP1A_ID)
    {
        if (_rfHz < 60000000)
            lna = std::min(6, lna);
        else if (_rfHz >= 1000000000)
            lna = std::min(8, lna);
        else
            lna = std::min(9, lna);
    }
    else if (_hwVer == SDRPLAY_RSP2_ID)
    {
        if (_rfHz >= 420000000)
            lna = std::min(5, lna);
        else if (_rfHz < 60000000 && _antenna == "HIGHZ")
            lna = std::min(4, lna);
        else
            lna = std::min(8, lna);
    }
    else if (_hwVer == SDRPLAY_RSPduo_ID)
    {
        if (_rfHz >= 1000000000)
            lna = std::min(8, lna);
        else if (_rfHz < 60000000 && _antenna == "HIGHZ")
            lna = std::min(4, lna);
        else if (_rfHz < 60000000)
            lna = std::min(6, lna);
        else
            lna = std::min(9, lna);
    }

    return lna;
}

double rsp_dev::set_gain(double gain)
{
    set_gain(gain, "IF_ATTEN_DB");
    return get_gain("IF_ATTEN_DB");
}

double rsp_dev::set_gain(double gain, const std::string &name)
{
    bool bcastNotchChanged = false;
    bool dabNotchChanged = false;
    bool gainChanged = false;

    if (name == "LNA_ATTEN_STEP")
    {
        if (gain != _lna) //RSP1 will only send bool / 0||1
            gainChanged = true;
        _lna = int(gain);
    }
    else if (name == "IF_ATTEN_DB")
    {
        // Ignore out-of-bounds values, since caller knows limits. (GQRX spurious calls).
        if (gain >= 20.0 && gain <= 59.0 && gain != _gRdB)
        {
            gainChanged = true;
            _gRdB = int(gain);
        }
    }
    // RSP1A, RSP2
    else if (name == "BCAST_NOTCH" && (_hwVer == SDRPLAY_RSP2_ID ||
              _hwVer == SDRPLAY_RSPduo_ID || _hwVer == SDRPLAY_RSP1A_ID))
    {
        if (int(gain) != _bcastNotch)
            bcastNotchChanged = true;
        _bcastNotch = int(gain);
    }
    // RSP1A
    else if (name == "DAB_NOTCH" && (_hwVer == SDRPLAY_RSPduo_ID ||
              _hwVer == SDRPLAY_RSP1A_ID))
    {
        if (int(gain) != _dabNotch)
            dabNotchChanged = true;
        _dabNotch = int(gain);
    }

    if (_streaming)
    {
        if (gainChanged)
        {
            _chParams->tunerParams.gain.gRdB = _gRdB;
            _chParams->tunerParams.gain.LNAstate = checkLNA(_lna);
            _chParams->tunerParams.gain.syncUpdate = 0; /* immediate */
        }

        if (bcastNotchChanged)
        {
            if (_hwVer == SDRPLAY_RSP1A_ID)
            {
                _deviceParams->devParams->rsp1aParams.rfNotchEnable = _bcastNotch;
            }
            else if (_hwVer == SDRPLAY_RSP2_ID)
            {
                _chParams->rsp2TunerParams.rfNotchEnable = _bcastNotch;
            }
            else if (_hwVer == SDRPLAY_RSPduo_ID)
            {
                if (_antenna == "HIGHZ")
                    _chParams->rspDuoTunerParams.tuner1AmNotchEnable = _bcastNotch;
                else
                    _chParams->rspDuoTunerParams.rfNotchEnable = _bcastNotch;
            }
        }

        if (dabNotchChanged)
        {
            if (_hwVer == SDRPLAY_RSP1A_ID)
            {
                _deviceParams->devParams->rsp1aParams.rfDabNotchEnable = _dabNotch;
            }
            else if (_hwVer == SDRPLAY_RSPduo_ID)
            {
                _chParams->rspDuoTunerParams.rfDabNotchEnable = _dabNotch;
            }
        }

        reinitDevice(sdrplay_api_Update_Tuner_Gr);
    }

    return get_gain();
}

double rsp_dev::get_gain() const
{
    return get_gain("IF_ATTEN_DB");
}

double rsp_dev::get_gain(const std::string &name) const
{
    if (name == "LNA_ATTEN_STEP")
        return _lna;
    else if (name == "BCAST_NOTCH")
        return _bcastNotch;
    else if (name == "DAB_NOTCH")
        return _dabNotch;
    else if (name == "IF_ATTEN_DB")
        return _gRdB;
    else
        return 0;
}

std::string rsp_dev::set_antenna(const std::string &antenna)
{
    _antenna = antenna;

    if (_streaming)
    {
        if (_hwVer == SDRPLAY_RSP2_ID)
        {
            // HIGHZ is ANTENNA_B with AmPortSelect
            if (antenna == "HIGHZ")
            {
               _chParams->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_B;
               _chParams->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_1;
            }
            else
            {
                if (antenna == "A")
                    _chParams->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_A;
                else
                    _chParams->rsp2TunerParams.antennaSel = sdrplay_api_Rsp2_ANTENNA_B;
               _chParams->rsp2TunerParams.amPortSel = sdrplay_api_Rsp2_AMPORT_2;
            }

            reinitDevice(sdrplay_api_Update_Rsp2_AmPortSelect);
        }
        else if (_hwVer == SDRPLAY_RSPduo_ID)
        {
            if (antenna == "HIGHZ")
            {
                _device.tuner = sdrplay_api_Tuner_A;
                _chParams->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_1;
            }
            else if (antenna == "T1_50ohm")
            {
                _device.tuner = sdrplay_api_Tuner_A;
                _chParams->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
            }
            else
            {
                _device.tuner = sdrplay_api_Tuner_B;
                _chParams->rspDuoTunerParams.tuner1AmPortSel = sdrplay_api_RspDuo_AMPORT_2;
            }

            reinitDevice(sdrplay_api_Update_RspDuo_AmPortSelect);
        }
    }
    return antenna;
}

std::string rsp_dev::get_antenna() const
{
    return _antenna;
}

void rsp_dev::set_dc_offset_mode(int mode)
{
    _dcMode = mode == 1;
}

void rsp_dev::set_iq_balance_mode(int mode)
{
    _iqMode = mode == 1;
}

void rsp_dev::set_debug_mode(int mode)
{
    _debug = mode == 1;
    sdrplay_api_DebugEnable(NULL, _debug ? sdrplay_api_DbgLvl_Verbose : sdrplay_api_DbgLvl_Disable);
}

double rsp_dev::set_bandwidth(double bandwidth)
{
    _bwType = sdrplay_api_BW_8_000;

    for (double bw : bandwidths)
    {
        if (bw == 0)
            continue;
        if (bandwidth <= bw)
        {
            _bwType = (sdrplay_api_Bw_MHzT)(bw / 1e3);
            break;
        }
    }

    int actual = get_bandwidth();
    std::cerr << "SDRplay bandwidth requested=" << bandwidth
              << " actual=" << actual << std::endl;

    if (_streaming)
    {
        reinitDevice(sdrplay_api_Update_Tuner_BwType);
    }

    return actual;
}

double rsp_dev::get_bandwidth() const
{
    return (double)_bwType * 1e3;
}

int rsp_dev::fetch_work_buffer(gr_complex *grWorkBuffer, int noutput_items)
{
    if (!_streaming)
        startStreaming();

    {
        boost::mutex::scoped_lock lock(_bufferMutex);
        _buffer = grWorkBuffer;
        _bufferSpaceRemaining = noutput_items;
        _bufferOffset = 0;
        _bufferReady.notify_one();

        while (_buffer && _streaming)
        {
            _bufferReady.wait(lock);
        }
    }

    if (_streaming)
    {
        return 0;
    }

    return 0; // ???
}

void rsp_dev::set_if_type(int ifType)
{
    _ifType = (sdrplay_api_If_kHzT)ifType;

    if (_streaming)
    {
        reinitDevice(sdrplay_api_Update_Tuner_IfType);
    }
}

void rsp_dev::set_lo_mode(int lo_mode)
{
    _loMode = (sdrplay_api_LoModeT)lo_mode;

    if (_streaming)
    {
        _chParams->tunerParams.loMode = _loMode;
        reinitDevice(sdrplay_api_Update_Tuner_LoMode);
    }
}

void rsp_dev::set_biasT(bool biasT)
{
    _biasT = biasT;

    if (_streaming)
    {
        if (_hwVer == SDRPLAY_RSP2_ID)
            _chParams->rsp2TunerParams.biasTEnable = biasT ? 1 : 0;
        else if (_hwVer == SDRPLAY_RSPduo_ID)
            _chParams->rspDuoTunerParams.biasTEnable = biasT ? 1 : 0;
        else if (_hwVer == SDRPLAY_RSP1A_ID)
            _chParams->rsp1aTunerParams.biasTEnable = biasT ? 1 : 0;
    }
}

void rsp_dev::set_deviceIndexOrSerial(const std::string &deviceIndexOrSerial)
{
    _deviceIndexOrSerial = deviceIndexOrSerial;
}

void rsp_dev::set_tuner(int tuner)
{
    _tuner = tuner;
}

void rsp_dev::set_rspduo_mode(int rspduo_mode)
{
    _rspduo_mode = rspduo_mode;
}

} // namespace sdrplay
} // namespace gr
