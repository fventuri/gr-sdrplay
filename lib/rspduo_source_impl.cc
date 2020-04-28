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

#include <gnuradio/io_signature.h>
#include "rspduo_source_impl.h"

#include <sdrplay_api.h>

#include <boost/assign.hpp>
#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono.hpp>
#include <iostream>
#include <string>


namespace gr {
    namespace sdrplay {

        using namespace boost::assign;

        rspduo_source::sptr
        rspduo_source::make(const std::string& rspduo_mode, double sample_rate,
                            double rf_freq, const std::string& antenna,
                            const std::string& device_serial, bool agc_enabled, 
                            double if_atten_db, int lna_atten_step, 
                            bool bcast_notch, bool dab_notch,
                            const std::string& output_type, bool dc_offset_mode,
                            bool iq_balance_mode, bool bias_t,
                            bool debug_enabled) {
            return gnuradio::get_initial_sptr(new rspduo_source_impl(
                            rspduo_mode, sample_rate, rf_freq, antenna,
                            device_serial, agc_enabled, if_atten_db,
                            lna_atten_step, bcast_notch, dab_notch, output_type,
                            dc_offset_mode, iq_balance_mode, bias_t,
                            debug_enabled));
        }

        rspduo_source_impl::rspduo_source_impl(const std::string& rspduo_mode,
                            double sample_rate, double rf_freq,
                            const std::string& antenna,
                            const std::string& device_serial, bool agc_enabled,
                            double if_atten_db, int lna_atten_step,
                            bool bcast_notch, bool dab_notch,
                            const std::string& output_type, bool dc_offset_mode,
                            bool iq_balance_mode, bool bias_t,
                            bool debug_enabled)
                : gr::sync_block("rspduo_source",
                                 gr::io_signature::make(0, 0, 0),
                                 gr::io_signature::make(1, 2,
                                         rsp_dev::output_size(output_type))) {
            dev = new rsp_dev();
            dev->list_available_rsp_devices();
            dev->set_debug_mode(debug_enabled);
            dev->set_deviceIndexOrSerial(device_serial);
            dev->set_rspduo_mode(rspduo_mode);
            dev->set_antenna(antenna);
            dev->set_sample_rate(sample_rate);
            dev->set_output_type(output_type);
            dev->set_center_freq(rf_freq);
            dev->set_gain_mode(agc_enabled);
            dev->set_dc_offset_mode(dc_offset_mode);
            dev->set_iq_balance_mode(iq_balance_mode);
            dev->set_gain(if_atten_db, "IF_ATTEN_DB");
            dev->set_gain(bcast_notch, "BCAST_NOTCH");
            dev->set_gain(dab_notch, "DAB_NOTCH");
            dev->set_gain(lna_atten_step, "LNA_ATTEN_STEP");
            dev->set_biasT(bias_t);
        }

        rspduo_source_impl::~rspduo_source_impl() {
            dev->stop();
            delete dev;
        }

        int
        rspduo_source_impl::work(int noutput_items,
                                 gr_vector_const_void_star &input_items,
                                 gr_vector_void_star &output_items) {

            return noutput_items - dev->fetch_work_buffers(output_items,
                                                           noutput_items);
        }

        bool rspduo_source_impl::start() {
            return dev->start();
        }

        bool rspduo_source_impl::stop() {
            return dev->stop();
        }

        void rspduo_source_impl::set_rf_freq(float rf_freq) {
            dev->set_center_freq(rf_freq);
        }

        void rspduo_source_impl::set_agc_enabled(bool agc_enabled) {
            dev->set_gain_mode(agc_enabled);
        }

        void rspduo_source_impl::set_if_atten_db(int if_atten_db) {
            dev->set_gain(if_atten_db, "IF_ATTEN_DB");
        }

        void rspduo_source_impl::set_biasT(bool bias_t) {
            dev->set_biasT(bias_t);
        }

        void rspduo_source_impl::set_lna_atten_step(int lna_atten_step) {
            dev->set_gain(lna_atten_step, "LNA_ATTEN_STEP");
        }

    } /* namespace rsp */
} /* namespace gr */
