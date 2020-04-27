/* -*- c++ -*- */
/* 
 * Copyright 2018 HB9FXQ, Frank Werner-Krippendorf.
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
#include "rsp2_source_impl.h"

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

        rsp2_source::sptr
        rsp2_source::make(double rf_freq, double bw, bool agc_enabled, double if_atten_db,
                          bool dc_offset_mode, bool iq_balance_mode, bool debug_enabled, int if_type, int lo_mode,
                          double sample_rate, int lna_atten_step,
                          std::string device_serial, std::string antenna) {
            return gnuradio::get_initial_sptr(
                    new rsp2_source_impl(rf_freq, bw, agc_enabled, if_atten_db, dc_offset_mode, iq_balance_mode,
                                         debug_enabled, if_type, lo_mode, sample_rate,
                                         lna_atten_step, device_serial, antenna));
        }

        rsp2_source_impl::rsp2_source_impl(double rf_freq, double bw, bool agc_enabled, double if_atten_db,
                                           bool dc_offset_mode, bool iq_balance_mode, bool debug_enabled, int if_type,
                                           int lo_mode, double sample_rate,
                                           int lna_atten_step,
                                           std::string device_serial, std::string antenna)
                : gr::sync_block("rsp2_source",
                                 gr::io_signature::make(0, 0, 0),
                                 gr::io_signature::make(1, 1, sizeof(gr_complex))) {
            dev = new rsp_dev();
            dev->list_available_rsp_devices();
            dev->set_debug_mode(debug_enabled);
            dev->set_center_freq(rf_freq);
            dev->set_gain_mode(agc_enabled);
            dev->set_dc_offset_mode(dc_offset_mode);
            dev->set_iq_balance_mode(iq_balance_mode);
            dev->set_sample_rate(sample_rate);
            dev->set_gain(if_atten_db, "IF_ATTEN_DB");
            dev->set_gain(lna_atten_step, "LNA_ATTEN_STEP");
            dev->set_deviceIndexOrSerial(device_serial);
            dev->set_antenna(antenna);
        }

        rsp2_source_impl::~rsp2_source_impl() {
            dev->stop();
            delete dev;
        }

        int
        rsp2_source_impl::work(int noutput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items) {

            return noutput_items - dev->fetch_work_buffers(output_items,
                                                           noutput_items);
        }

        void rsp2_source_impl::set_rf_freq(float rf_freq) {
            dev->set_center_freq(rf_freq);
        }

        void rsp2_source_impl::set_agc_enabled(bool agc_enabled) {
            dev->set_gain_mode(agc_enabled);
        }

        void rsp2_source_impl::set_if_atten_db(int if_atten_db) {
            dev->set_gain(if_atten_db, "IF_ATTEN_DB");
        }


        void rsp2_source_impl::set_lna_atten_step(int lna_atten_step) {
            dev->set_gain(lna_atten_step, "LNA_ATTEN_STEP");
        }
    } /* namespace rsp */
} /* namespace gr */

