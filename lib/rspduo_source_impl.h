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

#ifndef INCLUDED_SDRPLAY_RSPDUO_SOURCE_IMPL_H
#define INCLUDED_SDRPLAY_RSPDUO_SOURCE_IMPL_H

#include <sdrplay/rspduo_source.h>
#include <sdrplay_api.h>
#include "rsp_dev.h"


namespace gr {
    namespace sdrplay {

        class rspduo_source_impl : public rspduo_source {
        private:
            rsp_dev *dev;

        public:

            explicit rspduo_source_impl(const std::string& rspduo_mode,
                                        double sample_rate, double rf_freq,
                                        const std::string& antenna,
                                        const std::string& device_serial,
                                        bool agc_enabled, double if_atten_db,
                                        int lna_atten_step, bool bcast_notch,
                                        bool dab_notch,
                                        const std::string&  output_type,
                                        bool dc_offset_mode,
                                        bool iq_balance_mode, bool bias_t,
                                        bool debug_enabled);

            ~rspduo_source_impl();

            int work(int noutput_items,
                     gr_vector_const_void_star &input_items,
                     gr_vector_void_star &output_items);

            void set_rf_freq(float rf_freq);

            void set_agc_enabled(bool agc_enabled);

            void set_if_atten_db(int if_atten_db);

            void set_biasT(bool bias_t);

            void set_lna_atten_step(int lna_atten_step);

            // fv
            void flush();
            bool start();
            bool stop();

        };

    } // namespace sdrplay
} // namespace gr

#endif /* INCLUDED_SDRPLAY_RSPdUO_SOURCE_IMPL_H */

