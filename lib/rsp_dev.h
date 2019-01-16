/* -*- c++ -*- */
/*
 * Copyright 2018 HB9FXQ, Frank Werner-Krippendorf.
 * Copyright 2019 Franco Venturi - changes for SDRplay API version 3
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


#ifndef GR_SDRPLAY_RSP_DEV_H
#define GR_SDRPLAY_RSP_DEV_H

#include <sdrplay_api.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <gnuradio/gr_complex.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace gr {
    namespace sdrplay {

        class rsp_dev {
        public:
            rsp_dev();

            ~rsp_dev();

            void list_available_rsp_devices();

            double set_sample_rate(double rate);

            double get_sample_rate(void) const;

            double set_center_freq(double freq);

            double get_center_freq() const;

            bool set_gain_mode(bool automatic);

            bool get_gain_mode() const;

            double set_gain(double gain);

            double set_gain(double gain, const std::string &name);

            double get_gain() const;

            double get_gain(const std::string &name) const;

            std::string set_antenna(const std::string &antenna);

            std::string get_antenna() const;

            void set_dc_offset_mode(int mode);

            void set_iq_balance_mode(int mode);

            void set_debug_mode(int mode);

            double set_bandwidth(double bandwidth);

            double get_bandwidth() const;

            void startStreaming(void);

            void stopStreaming(void);

            int fetch_work_buffer(gr_complex *grWorkBuffer, int noutput_items);

            void set_if_type(int ifType);

            void set_lo_mode(int lo_mode);

            void set_biasT(bool biasT);

            void set_deviceIndexOrSerial(const std::string &deviceIndexOrSerial);

            void set_tuner(int tuner);

            void set_rspduo_mode(int rspduo_mode);

        private:
            void reinitDevice(int reason);

            int checkLNA(int lna);

            void streamCallback(short *xi, short *xq,
                                sdrplay_api_StreamCbParamsT *params,
                                unsigned int numSamples, unsigned int reset);

            void eventCallback(sdrplay_api_EventT eventId,
                               sdrplay_api_TunerSelectT tuner,
                               sdrplay_api_EventParamsT *params);

            static void streamCallbackWrap(short *xi, short *xq,
                                           sdrplay_api_StreamCbParamsT *params,
                                           unsigned int numSamples, unsigned int reset,
                                           void *cbContext);

            static void discardStreamCallbackWrap(short *xi, short *xq,
                                                  sdrplay_api_StreamCbParamsT *params,
                                                  unsigned int numSamples, unsigned int reset,
                                                  void *cbContext);

            static void eventCallbackWrap(sdrplay_api_EventT eventId,
                                          sdrplay_api_TunerSelectT tuner,
                                          sdrplay_api_EventParamsT *params,
                                          void *cbContext);

            static int _refcount;

            bool _auto_gain;
            int _gRdB;
            int _lna;
            int _bcastNotch;
            int _dabNotch;
            double _fsHz;
            int _decim;
            double _rfHz;
            sdrplay_api_Bw_MHzT _bwType;
            sdrplay_api_If_kHzT _ifType;
            sdrplay_api_LoModeT _loMode;
            int _samplesPerPacket;
            bool _dcMode;
            bool _iqMode;
            bool _debug;
            unsigned char _hwVer;
            sdrplay_api_DeviceT _device;
            sdrplay_api_DeviceParamsT *_deviceParams;
            sdrplay_api_RxChannelParamsT *_chParams;
            std::string _antenna;
            int _biasT;
            std::string _deviceIndexOrSerial;
            int _tuner;
            int _rspduo_mode;

            bool _streaming;
            gr_complex *_buffer;
            int _bufferOffset;
            int _bufferSpaceRemaining;
            boost::mutex _bufferMutex;
            boost::condition_variable _bufferReady;  // buffer is ready to move to other thread

            bool _reinit;  // signal streamer to return after a reinit
        };
    }
}


#endif //GR_SDRPLAY_RSP_DEV_H
