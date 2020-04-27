/* -*- c++ -*- */
/*
 * gr-sdrplay Copyright 2018 HB9FXQ, Frank Werner-Krippendorf.
 *            Copyright 2020 Franco Venturi - changes for SDRplay API version 3
 *                                            and Dual Tuner for RSPduo
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
#include <gnuradio/block_registry.h>
#include <gnuradio/top_block.h>
#include <pmt/pmt.h>

#include <boost/assign.hpp>
#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono.hpp>

#include <csignal>
#include <iostream>
#include <string>
#include <mutex>
#include <sdrplay_api.h>
// fv
#include <chrono>
#include <thread>

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
    if (hwVer == SDRPLAY_RSPdx_ID)
        return "RSPdx";
    return "UNK";
}

void rsp_dev::list_available_rsp_devices()
{
    unsigned int numDevices = 0;

    sdrplay_api::get_instance();
    sdrplay_api_DeviceT sdrplayDevices[MAX_SUPPORTED_DEVICES];
    sdrplay_api_GetDevices(sdrplayDevices, &numDevices, MAX_SUPPORTED_DEVICES);

    if (numDevices == 0)
    {
        std::cerr << "Failed to open SDRplay device " << std::endl;
        throw std::runtime_error("Failed to open SDRplay device ");
    }

    for (unsigned int i = 0; i < numDevices; i++)
    {

        std::cerr << "RSP devIndex: [" << i << "] " << hwName(sdrplayDevices[i].hwVer) << " " << sdrplayDevices[i].SerNo << std::endl;
    }
}

rsp_dev::rsp_dev()
{
    _device_status = None;
    _hwVer = -1;
    _biasT = 0;
    _bufferOffset[0] = 0;
    _bufferOffset[1] = 0;
    _bufferSpaceRemaining[0] = 0;
    _bufferSpaceRemaining[1] = 0;
    _overflowBufferFirst[0] = 0;
    _overflowBufferFirst[1] = 0;
    _overflowBufferLast[0] = 0;
    _overflowBufferLast[1] = 0;
    _auto_gain = true;
    _gRdB = 40;
    _lna = 0;
    _bcastNotch = 0;
    _dabNotch = 0;
    _fsHz = 6e6;
    _decim = 1;
    _rfHz = 100e6;
    _bwType = sdrplay_api_BW_1_536;
    _ifType = sdrplay_api_IF_1_620;
    _loMode = sdrplay_api_LO_Auto;
    _dcMode = true;
    _iqMode = true;
    _rspduo_mode = sdrplay_api_RspDuoMode_Unknown;
    _rspduo_sample_freq = _fsHz;
    _buffer[0] = NULL;
    _buffer[1] = NULL;
    _reinit = false;
    _device = { 0 };
    _deviceParams = 0;
    _chParams = 0;
    _debug = 0;
}

rsp_dev::~rsp_dev()
{
    stop();
}

void *rsp_dev::xcopy(gr_complex *dest, const short *srci, const short *srcq, int n)
{
    for (int i = 0; i < n; ++i)
    {
        dest[i] = gr_complex(float(srci[i]) / 32768.0, float(srcq[i]) / 32768.0);
    }
    return dest;
}

void *rsp_dev::xcopy(short (*dest)[2], const short *srci, const short *srcq, int n)
{
    for (int i = 0; i < n; ++i)
    {
        dest[i][0] = srci[i];
        dest[i][1] = srcq[i];
    }
    return dest;
}

// Called by sdrplay streamer thread when data is available
template <class T>
void rsp_dev::streamCallback(short *xi, short *xq,
                             sdrplay_api_StreamCbParamsT *params,
                             unsigned int numSamples, unsigned int reset,
                             int stream_index)
{
    _reinit = false;

    boost::mutex::scoped_lock lock(_bufferMutex);

    if (!(_device_status & Streaming) || _reinit)
    {
        return;
    }

    while (!_buffer[stream_index])
    {
        if (boost::cv_status::timeout ==
            _bufferReady[stream_index].wait_for(lock, boost::chrono::milliseconds(250)))
        {
            std::cerr << "timeout on stream " << stream_index << "\n";
            return;
        }
    }

    T *buf = static_cast<T*>(_buffer[stream_index]) + _bufferOffset[stream_index];
    int ncopy = std::min((int)numSamples, _bufferSpaceRemaining[stream_index]);
    xcopy(buf, xi, xq, ncopy);
    _bufferOffset[stream_index] += ncopy;
    _bufferSpaceRemaining[stream_index] -= ncopy;

    // if there's any samples left to copy, store them temporarily in the
    // overflow buffer
    if (numSamples > ncopy)
    {
        _overflowBufferFirst[stream_index] = 0;
        _overflowBufferLast[stream_index] = numSamples - ncopy;
        // discard any overflow samples > OVERFLOW_BUFFER_SIZE
        size_t szover = std::min(numSamples - ncopy,
            static_cast<unsigned int>(OVERFLOW_BUFFER_SIZE)) * sizeof(short);
        memcpy(_overflowBuffer[stream_index][0], &xi[ncopy], szover);
        memcpy(_overflowBuffer[stream_index][1], &xq[ncopy], szover);
    }

    if (_bufferSpaceRemaining[stream_index] == 0)
    {
        _buffer[stream_index] = NULL;
        _bufferReady[stream_index].notify_one();
    }
}

// Callback wrappers
template <class T>
void rsp_dev::streamACallbackWrap(short *xi, short *xq,
                                  sdrplay_api_StreamCbParamsT *params,
                                  unsigned int numSamples, unsigned int reset,
                                  void *cbContext)
{
    rsp_dev *obj = (rsp_dev *)cbContext;
    obj->streamCallback<T>(xi, xq, params, numSamples, reset, 0);
}

template <class T>
void rsp_dev::streamBCallbackWrap(short *xi, short *xq,
                                  sdrplay_api_StreamCbParamsT *params,
                                  unsigned int numSamples, unsigned int reset,
                                  void *cbContext)
{
    rsp_dev *obj = (rsp_dev *)cbContext;
    obj->streamCallback<T>(xi, xq, params, numSamples, reset, 1);
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

bool rsp_dev::start_streaming()
{
    set_biasT(_biasT);

    set_gain_mode(get_gain_mode());

    int gRdB = _gRdB;

    sdrplay_api_CallbackFnsT callbackFns;
    if (_output_type == fc32) {
        callbackFns.StreamACbFn = streamACallbackWrap<gr_complex>;
        callbackFns.StreamBCbFn = streamBCallbackWrap<gr_complex>;
    } else if (_output_type == sc16) {
        callbackFns.StreamACbFn = streamACallbackWrap<short[2]>;
        callbackFns.StreamBCbFn = streamBCallbackWrap<short[2]>;
    } else {
        std::cerr << "Invalid output type: " << _output_type << std::endl;
        callbackFns.StreamACbFn = discardStreamCallbackWrap;
        callbackFns.StreamBCbFn = discardStreamCallbackWrap;
    }
    callbackFns.EventCbFn = eventCallbackWrap;

    if (_deviceParams->devParams != NULL)
        _deviceParams->devParams->fsFreq.fsHz = _fsHz;

    _chParams->tunerParams.gain.gRdB = gRdB;
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

    if (_dualchParams != NULL) {
        _dualchParams->tunerParams.gain.gRdB = gRdB;
        _dualchParams->tunerParams.rfFreq.rfHz = _rfHz;
        _dualchParams->tunerParams.bwType = _bwType;
        _dualchParams->tunerParams.ifType = _ifType;
        _dualchParams->tunerParams.gain.LNAstate = checkLNA(_lna);

        // Set decimation with halfband filter
        _dualchParams->ctrlParams.decimation.enable = _decim != 1;
        _dualchParams->ctrlParams.decimation.decimationFactor = _decim;
        _dualchParams->ctrlParams.decimation.wideBandSignal = 1;

        _dualchParams->ctrlParams.dcOffset.DCenable = _dcMode;
        _dualchParams->ctrlParams.dcOffset.IQenable = _iqMode;
    }

    // Model-specific initialization
    if (_hwVer == SDRPLAY_RSP2_ID)
    {
        set_antenna(get_antenna());
        _chParams->rsp2TunerParams.rfNotchEnable = _bcastNotch;
    }
    else if (_hwVer == SDRPLAY_RSPduo_ID)
    {
        set_antenna(get_antenna());
        if (_antenna == "HIGHZ") {
            _chParams->rspDuoTunerParams.tuner1AmNotchEnable = _bcastNotch;
        } else {
            _chParams->rspDuoTunerParams.rfNotchEnable = _bcastNotch;
            if (_dualchParams != NULL) {
                _dualchParams->rspDuoTunerParams.rfNotchEnable = _bcastNotch;
            }
        }
        _chParams->rspDuoTunerParams.rfDabNotchEnable = _dabNotch;
        if (_dualchParams != NULL) {
            _dualchParams->rspDuoTunerParams.rfDabNotchEnable = _dabNotch;
        }
    }
    else if (_hwVer == SDRPLAY_RSP1A_ID)
    {
        _deviceParams->devParams->rsp1aParams.rfNotchEnable = _bcastNotch;
        _deviceParams->devParams->rsp1aParams.rfDabNotchEnable = _dabNotch;
    }

    sdrplay_api_ErrT err;
    err = sdrplay_api_Init(_device.dev, &callbackFns, this);
    if (err != sdrplay_api_Success)
    {
        return false;
    }
    _device_status |= Streaming;
    return true;
}

bool rsp_dev::stop_streaming(void)
{
    sdrplay_api_ErrT err;
    err = sdrplay_api_Uninit(_device.dev);
    if (err != sdrplay_api_Success)
    {
        return false;
    }
    _device_status &= ~Streaming;
    return true;
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
    if (_deviceParams->devParams != NULL)
        _deviceParams->devParams->fsFreq.fsHz = _fsHz;
    _chParams->tunerParams.rfFreq.rfHz = _rfHz;
    _chParams->tunerParams.bwType = _bwType;
    _chParams->tunerParams.ifType = _ifType;
    _chParams->tunerParams.gain.LNAstate = checkLNA(_lna);

    // Set decimation with halfband filter
    _chParams->ctrlParams.decimation.enable = _decim != 1;
    _chParams->ctrlParams.decimation.decimationFactor = _decim;
    _chParams->ctrlParams.decimation.wideBandSignal = 1;

    sdrplay_api_Update(_device.dev, _device.tuner, (sdrplay_api_ReasonForUpdateT)reason, sdrplay_api_Update_Ext1_None);

    _bufferReady[0].notify_one();
    _bufferReady[1].notify_one();
}

// also sets IF type and bandwidth
double rsp_dev::set_sample_rate(double rate)
{
    double prev_fsHz = _fsHz;
    int prev_decim = _decim;
    sdrplay_api_If_kHzT prev_ifType = _ifType;
    sdrplay_api_Bw_MHzT prev_bwType = _bwType;

    if (rate <= 2e6) {
        if      (rate == 62.5e3) { _decim = 32; }
        else if (rate == 125e3)  { _decim = 16; }
        else if (rate == 250e3)  { _decim =  8; }
        else if (rate == 500e3)  { _decim =  4; }
        else if (rate ==   1e6)  { _decim =  2; }
        else if (rate ==   2e6)  { _decim =  1; }
        else {
            std::cerr << "invalid sample rate: " << rate << ". sample_rate unchanged\n";
            return get_sample_rate();
        }
        if (_rspduo_mode != sdrplay_api_RspDuoMode_Unknown &&
            _rspduo_mode != sdrplay_api_RspDuoMode_Single_Tuner &&
            _rspduo_sample_freq == 8e6) {
            _fsHz = 8e6;
            _ifType = sdrplay_api_IF_2_048;
        } else {
            _fsHz = 6e6;
            _ifType = sdrplay_api_IF_1_620;
        }
    } else if (rate == 2.048e6 || rate == 3e6 || rate == 4e6 || rate == 5e6 ||
               rate == 6e6 || rate == 7e6 || rate == 8e6 || rate == 9e6 ||
               rate == 10e6) {
        _decim = 1;
        _fsHz = rate;
        _ifType = sdrplay_api_IF_Zero;
    } else {
        std::cerr << "invalid sample rate: " << rate << ". sample_rate unchanged\n";
        return get_sample_rate();
    }
    _sample_rate = rate;

    set_bandwidth();

    if (_device_status & Streaming) {
        sdrplay_api_ReasonForUpdateT reason_for_update = sdrplay_api_Update_None;
        if (_fsHz != prev_fsHz) {
            if (_deviceParams->devParams != NULL)
                _deviceParams->devParams->fsFreq.fsHz = _fsHz;
            reason_for_update = (sdrplay_api_ReasonForUpdateT)(reason_for_update | sdrplay_api_Update_Dev_Fs);
        }
        if (_decim != prev_decim) {
            _chParams->ctrlParams.decimation.enable = _decim != 1;
            _chParams->ctrlParams.decimation.decimationFactor = _decim;
            _chParams->ctrlParams.decimation.wideBandSignal = 1;
            reason_for_update = (sdrplay_api_ReasonForUpdateT)(reason_for_update | sdrplay_api_Update_Ctrl_Decimation);
        }
        if (_ifType != prev_ifType) {
            _chParams->tunerParams.ifType = _ifType;
            reason_for_update = (sdrplay_api_ReasonForUpdateT)(reason_for_update | sdrplay_api_Update_Tuner_IfType);
        }
        if (_bwType != prev_bwType) {
            _chParams->tunerParams.bwType = _bwType;
            reason_for_update = (sdrplay_api_ReasonForUpdateT)(reason_for_update | sdrplay_api_Update_Tuner_BwType);
        }
        if (reason_for_update != sdrplay_api_Update_None)
            reinitDevice(reason_for_update);
    }

    return get_sample_rate();
}

double rsp_dev::get_sample_rate() const
{
    return _sample_rate;
}

double rsp_dev::set_center_freq(double freq)
{
    _rfHz = freq;

    if (_device_status & Streaming)
    {
        _chParams->tunerParams.rfFreq.rfHz = _rfHz;
        if (_dualchParams != NULL) {
            _dualchParams->tunerParams.rfFreq.rfHz = _rfHz;
        }
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
    if (_device_status & Streaming)
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
        _chParams->ctrlParams.agc.setPoint_dBfs = 0;
        _chParams->ctrlParams.agc.attack_ms = 0;
        _chParams->ctrlParams.agc.decay_ms = 0;
        _chParams->ctrlParams.agc.decay_delay_ms = 0;
        _chParams->ctrlParams.agc.decay_threshold_dB = 0;
        _chParams->ctrlParams.agc.syncUpdate = 0;

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

    if (_device_status & Streaming)
    {
        if (gainChanged)
        {
            _chParams->tunerParams.gain.gRdB = _gRdB;
            _chParams->tunerParams.gain.LNAstate = checkLNA(_lna);
            _chParams->tunerParams.gain.syncUpdate = 0; /* immediate */
            if (_dualchParams != NULL) {
                _dualchParams->tunerParams.gain.gRdB = _gRdB;
                _dualchParams->tunerParams.gain.LNAstate = checkLNA(_lna);
                _dualchParams->tunerParams.gain.syncUpdate = 0; /* immediate */
            }
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

    if (_device_status & Streaming)
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

// bandwidth depends strictly on sample rate
double rsp_dev::set_bandwidth()
{
    if      (_sample_rate <  300e3) { _bwType = sdrplay_api_BW_0_200; }
    else if (_sample_rate <  600e3) { _bwType = sdrplay_api_BW_0_300; }
    else if (_sample_rate < 1536e3) { _bwType = sdrplay_api_BW_0_600; }
    else if (_sample_rate <    5e6) { _bwType = sdrplay_api_BW_1_536; }
    else if (_sample_rate <    6e6) { _bwType = sdrplay_api_BW_5_000; }
    else if (_sample_rate <    7e6) { _bwType = sdrplay_api_BW_6_000; }
    else if (_sample_rate <    8e6) { _bwType = sdrplay_api_BW_7_000; }
    else                            { _bwType = sdrplay_api_BW_8_000; }

    int actual = get_bandwidth();
    std::cerr << "SDRplay actual bandwidth=" << actual << std::endl;

    if (_device_status & Streaming)
    {
        reinitDevice(sdrplay_api_Update_Tuner_BwType);
    }

    return actual;
}

double rsp_dev::get_bandwidth() const
{
    return (double)_bwType * 1e3;
}

// pull the samples from the overflow buffer, returns true if there's still
// space left in in the gnuradio buffer
template <class T>
bool rsp_dev::process_overflow_buffer(int si)
{
    int num_overflow = _overflowBufferLast[si] - _overflowBufferFirst[si];
    if (num_overflow == 0)
        return true;
    T *buf = static_cast<T*>(_buffer[si]) + _bufferOffset[si];
    int ncopy = std::min(num_overflow, _bufferSpaceRemaining[si]);
    short *xi = _overflowBuffer[si][0] + _overflowBufferFirst[si];
    short *xq = _overflowBuffer[si][1] + _overflowBufferFirst[si];
    xcopy(buf, xi, xq, ncopy);
    _bufferOffset[si] += ncopy;
    _bufferSpaceRemaining[si] -= ncopy;
    _overflowBufferFirst[si] += ncopy;
    if (_overflowBufferFirst[si] == _overflowBufferLast[si])
    {
        _overflowBufferFirst[si] = 0;
        _overflowBufferLast[si]  = 0;
    }
    return _bufferSpaceRemaining[si] > 0;
}

// makes sure that bufferSpaceRemaining == 0 and that all the overflow buffers
// have the same number of samples
bool rsp_dev::buffers_sanity_check(int nstreams)
{
    bool is_ok = true;
    for (int stream_index = 0; stream_index < nstreams; ++stream_index)
    {
        if (_bufferSpaceRemaining[stream_index] != 0)
        {
             std::cerr << "stream " << stream_index << " has still space remanining\n";
             is_ok = false;
        }
    }
    if (nstreams == 1)
        return is_ok;
    int num_overflow[2] = { _overflowBufferLast[0] - _overflowBufferFirst[0],
                            _overflowBufferLast[1] - _overflowBufferFirst[1] };
    int min_overflow = std::min(num_overflow[0], num_overflow[1]);
    for (int stream_index = 0; stream_index < nstreams; ++stream_index)
    {
        if (num_overflow[stream_index] > min_overflow)
        {
             std::cerr << "discarding " << (num_overflow[stream_index] - min_overflow) << " items (" << _overflowBufferFirst[0] << ", " << _overflowBufferLast[0] << ") and (" << _overflowBufferFirst[1] << ", " << _overflowBufferLast[1] << ") from stream " << stream_index << "\n";
             _overflowBufferLast[stream_index] = _overflowBufferFirst[stream_index] + min_overflow;
             is_ok = false;
        }
    }
    return is_ok;
}

int rsp_dev::fetch_work_buffers(gr_vector_void_star &output_items,
                                int noutput_items)
{
    if (!(_device_status & Streaming))
    {
        std::cerr << "device is not streaming\n";
        return noutput_items;
    }

    int nstreams = output_items.size();

    {
        boost::mutex::scoped_lock lock(_bufferMutex);
        bool hasSpaceRemaining[2];
        for (int stream_index = 0; stream_index < nstreams; ++stream_index)
        {
            _buffer[stream_index] = output_items[stream_index];
            _bufferSpaceRemaining[stream_index] = noutput_items;
            _bufferOffset[stream_index] = 0;
            if (_output_type == fc32) {
                hasSpaceRemaining[stream_index] = process_overflow_buffer<gr_complex>(stream_index);
            } else if (_output_type == sc16) {
                hasSpaceRemaining[stream_index] = process_overflow_buffer<short[2]>(stream_index);
            } else {
                std::cerr << "Invalid output type: " << _output_type << std::endl;
                return noutput_items;
            }
            if (hasSpaceRemaining[stream_index])
            {
                _bufferReady[stream_index].notify_one();
            }
        }

        if (!hasSpaceRemaining[0] && (nstreams < 2 || !hasSpaceRemaining[1]))
        {
            return 0;
        }

        for (int stream_index = 0; stream_index < nstreams; ++stream_index)
        {
            if (hasSpaceRemaining[stream_index])
            {
                while (_buffer[stream_index] && (_device_status & Streaming))
                {
                    _bufferReady[stream_index].wait(lock);
                }
            }
        }
    }

    buffers_sanity_check(nstreams);

    if (_device_status & Streaming)
    {
        return 0;
    }

    return noutput_items;
}

void rsp_dev::set_biasT(bool biasT)
{
    _biasT = biasT;

    if (!(_device_status & Selected))
    {
        return;
    }

    sdrplay_api_ReasonForUpdateT reasonForUpdate = sdrplay_api_Update_None;
    if (_hwVer == SDRPLAY_RSP2_ID) {
        _chParams->rsp2TunerParams.biasTEnable = biasT ? 1 : 0;
        reasonForUpdate = sdrplay_api_Update_Rsp2_BiasTControl;
    } else if (_hwVer == SDRPLAY_RSPduo_ID) {
        _chParams->rspDuoTunerParams.biasTEnable = biasT ? 1 : 0;
        reasonForUpdate = sdrplay_api_Update_RspDuo_BiasTControl;
    } else if (_hwVer == SDRPLAY_RSP1A_ID) {
        _chParams->rsp1aTunerParams.biasTEnable = biasT ? 1 : 0;
        reasonForUpdate = sdrplay_api_Update_Rsp1a_BiasTControl;
    }
    if ((_device_status & Streaming) && (reasonForUpdate != sdrplay_api_Update_None))
        sdrplay_api_Update(_device.dev, _device.tuner, reasonForUpdate, sdrplay_api_Update_Ext1_None);
}

void rsp_dev::set_deviceIndexOrSerial(const std::string &deviceIndexOrSerial)
{
    _deviceIndexOrSerial = deviceIndexOrSerial;
}

void rsp_dev::set_rspduo_mode(const std::string& rspduo_mode)
{
    if (rspduo_mode == "ST") {
        _rspduo_mode = sdrplay_api_RspDuoMode_Single_Tuner;
        return;
    }
    if (rspduo_mode == "DT") {
        _rspduo_mode = sdrplay_api_RspDuoMode_Dual_Tuner;
        _rspduo_sample_freq = 6000000;
        return;
    }
    if (rspduo_mode == "MA6") {
        _rspduo_mode = sdrplay_api_RspDuoMode_Master;
        _rspduo_sample_freq = 6000000;
        return;
    }
    if (rspduo_mode == "MA8") {
        _rspduo_mode = sdrplay_api_RspDuoMode_Master;
        _rspduo_sample_freq = 8000000;
        return;
    }
    if (rspduo_mode == "SL") {
        _rspduo_mode = sdrplay_api_RspDuoMode_Slave;
        return;
    }
    std::cerr << "invalid RSPduo mode: " << rspduo_mode << "\n";
    throw std::runtime_error("invalid RSPduo mode");
}

void rsp_dev::set_output_type(const std::string& output_type)
{
    if (output_type == "fc32") {
        _output_type = fc32;
        return;
    }
    if (output_type == "sc16") {
        _output_type = sc16;
        return;
    }
    std::cerr << "invalid output type: " << output_type << "\n";
    throw std::runtime_error("invalid output type");
}

size_t rsp_dev::output_size(const std::string& output_type)
{
    if (output_type == "fc32")
        return sizeof(gr_complex);
    if (output_type == "sc16")
        return sizeof(short[2]);
    // invalid output type - return 0
    return 0;
}

void rsp_dev::flush() {
    return;
}

bool rsp_dev::configure_rspduo()
{
    // set RSPduo mode
    if (!(_rspduo_mode & _device.rspDuoMode))
    {
        std::cerr << "Invalid RSPduo mode - valid modes are: " << _device.rspDuoMode << std::endl;
        return false;
    }
    _device.rspDuoMode = (sdrplay_api_RspDuoModeT)_rspduo_mode;

    // set RSPduo mode
    _device.tuner = sdrplay_api_Tuner_Neither;
    if (_rspduo_mode == sdrplay_api_RspDuoMode_Single_Tuner ||
        _rspduo_mode == sdrplay_api_RspDuoMode_Master)
    {
        if (_antenna == "T1_50ohm" || _antenna == "HIGHZ")
        {
            _device.tuner = sdrplay_api_Tuner_A;
        }
        else if (_antenna == "T2_50ohm")
        {
            _device.tuner = sdrplay_api_Tuner_B;
        }
    }

    // check antenna configuraion in slave mode
    if (_rspduo_mode == sdrplay_api_RspDuoMode_Slave)
    {
        if (_device.tuner == sdrplay_api_Tuner_A &&
            !(_antenna == "T1_50ohm" || _antenna == "HIGHZ"))
        {
            _antenna = "T1_50ohm";
            std::cerr << "Invalid RSPduo antenna in slave mode - forcing " << _antenna << " antenna" << std::endl;
        }
        else if (_device.tuner == sdrplay_api_Tuner_B &&
                 !(_antenna == "T2_50ohm"))
        {
            _antenna = "T2_50ohm";
            std::cerr << "Invalid RSPduo antenna in slave mode - forcing " << _antenna << " antenna" << std::endl;
        }
    }

    // set sample freq
    if (_rspduo_mode == sdrplay_api_RspDuoMode_Dual_Tuner ||
        _rspduo_mode == sdrplay_api_RspDuoMode_Master)
    {
        _device.rspDuoSampleFreq = _rspduo_sample_freq;
    }

    return true;
}

bool rsp_dev::select_rsp_device()
{
    unsigned int numDevices;
    sdrplay_api_DeviceT sdrplayDevices[MAX_SUPPORTED_DEVICES];
    sdrplay_api_LockDeviceApi();
    sdrplay_api_GetDevices(sdrplayDevices, &numDevices, MAX_SUPPORTED_DEVICES);

    int _devIndex = 0;

    if (_deviceIndexOrSerial.length() > 2 /*It's a SerialNo*/)
    {
        bool match = false;

        for (unsigned int i = 0; i < numDevices; i++)
        {
            if (_deviceIndexOrSerial.compare(std::string(sdrplayDevices[i].SerNo)) == 0)
            {
                std::cerr << "Found requested RSP with SerialNO: " << sdrplayDevices[i].SerNo << std::endl;
                _devIndex = i;
                match = true;
                break;
            }
        }

        if (!match)
        {
            std::cerr << "FALLBACK TO DEV INDEX = !!!! Could NOT find RSP SerialNO: " << _deviceIndexOrSerial << std::endl;
        }
    } else {
        _devIndex = strtol(_deviceIndexOrSerial.c_str(), NULL, 0);
        if (_devIndex < 0 || _devIndex >= numDevices) {
            std::cerr << "FALLBACK TO DEV INDEX = 0 !!!! Could NOT find RSP with index: " << _deviceIndexOrSerial << std::endl;
        }
    }

    _device = sdrplayDevices[_devIndex];
    _hwVer = _device.hwVer;

    if (_hwVer == SDRPLAY_RSPduo_ID)
    {
        if (!configure_rspduo())
            return false;
    }

    sdrplay_api_SelectDevice(&_device);
    sdrplay_api_UnlockDeviceApi();

    _device_status |= Selected;

    std::cerr << "Using SDRplay " << hwName(_hwVer) << " "
              << sdrplayDevices[_devIndex].SerNo << std::endl;

    sdrplay_api_DebugEnable(_device.dev, _debug ? sdrplay_api_DbgLvl_Verbose : sdrplay_api_DbgLvl_Disable);

    sdrplay_api_GetDeviceParams(_device.dev, &_deviceParams);

    _chParams = _deviceParams->rxChannelA;
    _dualchParams = NULL;
    if (_device.tuner == sdrplay_api_Tuner_B)
    {
        _chParams = _deviceParams->rxChannelB;
    }
    else if (_device.tuner == sdrplay_api_Tuner_Both)
    {
        _dualchParams = _deviceParams->rxChannelB;
    }

    return true;
}

bool rsp_dev::start() {
    if (_device_status & Streaming)
    {
        return true;
    }

    if (!(_device_status & Selected))
    {
        if (!select_rsp_device()) {
            return false;
        }
        sdrplay_api::get_instance().add_active_rsp(this);
    }

    if (!(_device_status & Streaming))
    {
        if (!start_streaming()) {
            return false;
        }
    }

    return true;
}

bool rsp_dev::release_rsp_device()
{
    sdrplay_api_ReleaseDevice(&_device);
    _device_status &= ~Selected;
    return true;
}

bool rsp_dev::stop() {
    if (_device_status == None)
    {
        return true;
    }

    if (_device_status & Streaming)
    {
        if (!stop_streaming()) {
            return false;
        }
    }

    if (_device_status & Selected)
    {
        if (!release_rsp_device()) {
            return false;
        }
        sdrplay_api::get_instance().remove_active_rsp(this);
    }
    return true;
}

void signal_handler(int signum)
{
    std::exit(signum);
}

// Singleton class for SDRplay API (only one per process)
sdrplay_api::sdrplay_api()
{
    sdrplay_api_ErrT err;
    // Open API
    err = sdrplay_api_Open();
    if (err != sdrplay_api_Success) {
        std::cerr << "sdrplay_api_Open() Error: " << sdrplay_api_GetErrorString(err) << std::endl;
        throw std::runtime_error("sdrplay_api_Open() failed");
    }
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Check API versions match
    float ver;
    err = sdrplay_api_ApiVersion(&ver);
    if (err != sdrplay_api_Success) {
        std::cerr << "sdrplay_api_ApiVersion() Error: " << sdrplay_api_GetErrorString(err) << std::endl;
        sdrplay_api_Close();
        throw std::runtime_error("ApiVersion() failed");
    }
    if (ver != SDRPLAY_API_VERSION) {
        std::cerr << "sdrplay_api version: " << ver << " does not equal build version: " << SDRPLAY_API_VERSION << std::endl;
        sdrplay_api_Close();
        throw std::runtime_error("sdrplay_api version does not match build version");
    }
}

sdrplay_api::~sdrplay_api()
{

    // stop active RSPs cleanly
    std::vector<rsp_dev*> rsps(active_rsps.begin(), active_rsps.end());
    for (auto rsp = rsps.begin(); rsp != rsps.end(); ++rsp) {
        (*rsp)->stop();
    }

    sdrplay_api_ErrT err;
    // Close API
    err = sdrplay_api_Close();
    if (err != sdrplay_api_Success) {
        std::cerr << "sdrplay_api_Close() Error: " << sdrplay_api_GetErrorString(err) << std::endl;
    }
}

void sdrplay_api::add_active_rsp(rsp_dev* rsp)
{
    active_rsps.insert(rsp);
    return;
}

void sdrplay_api::remove_active_rsp(rsp_dev* rsp)
{
    active_rsps.erase(rsp);
    return;
}

} // namespace sdrplay
} // namespace gr
