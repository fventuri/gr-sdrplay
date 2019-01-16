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

#ifndef INCLUDED_SDRPLAY_RSP1A_SOURCE_IMPL_H
#define INCLUDED_SDRPLAY_RSP1A_SOURCE_IMPL_H

#include <sdrplay/rsp1a_source.h>
#include <sdrplay_api.h>
#include "rsp_dev.h"


namespace gr {
    namespace sdrplay {

        class rsp1a_source_impl : public rsp1a_source {
        private:
            rsp_dev *dev;

        public:

            explicit rsp1a_source_impl(double rf_freq, double bw, bool agc_enabled, double if_atten_db,
                                       bool dc_offset_mode, bool iq_balance_mode, bool debug_enabled, int if_type,
                                       int lo_mode, double sample_rate, bool bcast_notch, bool dab_notch,
                                       int lna_atten_step, bool bias_t,
                                       std::string device_serial);

            ~rsp1a_source_impl();

            int work(int noutput_items,
                     gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items);

            void set_rf_freq(float rf_freq);

            void set_agc_enabled(bool agc_enabled);

            void set_if_atten_db(int if_atten_db);

            void set_biasT(bool bias_t);

            void set_lna_atten_step(int lna_atten_step);
        };

    } // namespace sdrplay
} // namespace gr

#endif /* INCLUDED_SDRPLAY_RSP1A_SOURCE_IMPL_H */

