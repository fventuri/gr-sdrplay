/* -*- c++ -*- */
/*
 * Copyright 2018 HB9FXQ, Frank Werner-Krippendorf.
 * Copyright 2020 Franco Venturi - changes for SDRplay API version 3
 *                                 and Dual Tuner for RSPduo
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
#include <set>
#include <stdlib.h>
#include <string>
#include <vector>
#include <gnuradio/gr_complex.h>
#include <gnuradio/types.h>
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

            double get_sample_rate() const;

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

            double set_bandwidth();

            double get_bandwidth() const;

            bool start_streaming();

            bool stop_streaming();

            int fetch_work_buffers(gr_vector_void_star &output_items,
                                   int noutput_items);

            void set_biasT(bool biasT);

            void set_deviceIndexOrSerial(const std::string &deviceIndexOrSerial);

            void set_rspduo_mode(const std::string& rspduo_mode);

            void set_output_type(const std::string& output_type);

            static size_t output_size(const std::string& output_type);

            void flush();

            bool configure_rspduo();

            bool select_rsp_device();

            bool start();

            bool release_rsp_device();

            bool stop();

        private:
            void reinitDevice(int reason);

            int checkLNA(int lna);

            template <class T>
            bool process_overflow_buffer(int stream_index);

            bool buffers_sanity_check(int nstreams);

            static void *xcopy(gr_complex *dest, const short *srci,
                               const short *srcq, int n);

            static void *xcopy(short (*dest)[2], const short *srci,
                               const short *srcq, int n);

            template <class T>
            void streamCallback(short *xi, short *xq,
                                sdrplay_api_StreamCbParamsT *params,
                                unsigned int numSamples, unsigned int reset,
                                int stream_index);

            void eventCallback(sdrplay_api_EventT eventId,
                               sdrplay_api_TunerSelectT tuner,
                               sdrplay_api_EventParamsT *params);

            template <class T>
            static void streamACallbackWrap(short *xi, short *xq,
                                            sdrplay_api_StreamCbParamsT *params,
                                            unsigned int numSamples, unsigned int reset,
                                            void *cbContext);

            template <class T>
            static void streamBCallbackWrap(short *xi, short *xq,
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
            bool _dcMode;
            bool _iqMode;
            bool _debug;
            unsigned char _hwVer;
            sdrplay_api_DeviceT _device;
            sdrplay_api_DeviceParamsT *_deviceParams;
            sdrplay_api_RxChannelParamsT *_chParams;
            sdrplay_api_RxChannelParamsT *_dualchParams;
            std::string _antenna;
            int _biasT;
            std::string _deviceIndexOrSerial;
            int _rspduo_mode;
            double _rspduo_sample_freq;
            double _sample_rate;
            enum OutputTypes {fc32=1, sc16=2};
            enum OutputTypes _output_type;

            enum DeviceStatus {None=0, Selected=1, Streaming=2};
            int _device_status;
            void *_buffer[2];
            int _bufferOffset[2];
            int _bufferSpaceRemaining[2];
            boost::mutex _bufferMutex;
            boost::condition_variable _bufferReady[2];

            static const unsigned int OVERFLOW_BUFFER_SIZE = 2016;
            short _overflowBuffer[2][2][OVERFLOW_BUFFER_SIZE];
            int _overflowBufferFirst[2];
            int _overflowBufferLast[2];

            bool _reinit;  // signal streamer to return after a reinit
        };

        // Singleton class for SDRplay API (only one per process)
        class sdrplay_api {
        public:
            static sdrplay_api& get_instance()
            {
                static sdrplay_api instance;
                return instance;
            }
            void add_active_rsp(rsp_dev*);
            void remove_active_rsp(rsp_dev*);

        private:
            std::set<rsp_dev*> active_rsps;
            sdrplay_api();

        public:
            ~sdrplay_api();
            sdrplay_api(sdrplay_api const&)    = delete;
            void operator=(sdrplay_api const&) = delete;
        };
    }
}

#endif //GR_SDRPLAY_RSP_DEV_H
