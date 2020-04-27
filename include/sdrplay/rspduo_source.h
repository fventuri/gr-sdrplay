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


#ifndef INCLUDED_SDRPLAY_RSPDUO_SOURCE_H
#define INCLUDED_SDRPLAY_RSPDUO_SOURCE_H

#include <sdrplay/api.h>
#include <gnuradio/sync_block.h>

#include <boost/assign.hpp>
#include <boost/format.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono.hpp>

#include <iostream>
#include <mutex>



namespace gr {
  namespace sdrplay {


#define MAX_SUPPORTED_DEVICES   4

    /*!
     * \brief <+description of block+>
     * \ingroup sdrplay
     *
     */
    class SDRPLAY_API rspduo_source : virtual public gr::sync_block
    {
    public:
        typedef boost::shared_ptr<rspduo_source> sptr;

        static sptr make(const std::string& rspduo_mode, double sample_rate,
                         double rf_freq, const std::string& antenna,
                         const std::string& device_serial, bool agc_enabled,
                         double if_atten_db, int lna_atten_step,
                         bool bcast_notch, bool dab_notch,
                         const std::string& output_type, bool dc_offset_mode,
                         bool iq_balance_mode, bool bias_t, bool debug_enabled);

        virtual void set_rf_freq(float rf_freq) = 0;

        virtual void set_agc_enabled(bool agc_enabled) = 0;

        virtual void set_if_atten_db(int if_atten_db) = 0;

        virtual void set_lna_atten_step(int lna_atten_step) = 0;

        virtual void set_biasT(bool bias_t) = 0;
    };

  } // namespace sdrplay
} // namespace gr

#endif /* INCLUDED_SDRPLAY_RSPDUO_SOURCE_H */

