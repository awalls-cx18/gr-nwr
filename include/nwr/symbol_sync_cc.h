/* -*- c++ -*- */
/*
 * Copyright 2004,2011,2012 Free Software Foundation, Inc.
 * Copyright (C) 2016-2017  Andy Walls <awalls.cx18@gmail.com>
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_NWR_SYMBOL_SYNC_CC_H
#define INCLUDED_NWR_SYMBOL_SYNC_CC_H

#include <nwr/api.h>
#include <gnuradio/block.h>
#include <nwr/timing_error_detector.h>

namespace gr {
  namespace nwr {
    
    /*!
     * \brief Symbol Synchronizer block with complex input, complex output.
     * \ingroup synchronizers_blk
     *
     * \details
     * This implements a discrete-time error-tracking synchronizer.
     *
     * For this block to work properly, the input stream must meet the
     * following requirements:
     *
     * 1. the input pulses must have peaks (not flat), which usually can
     * be implemented by using a matched filter before this block.
     *
     * 2. the input pulses should nominally match the normalized
     * slicer constellation, which is normalized to an average symbol
     * magnitude of 1.0 over the entire constellation.
     *
     */
    class NWR_API symbol_sync_cc : virtual public block
    {
    public:
      // gr::nwr::symbol_sync_cc::sptr
      typedef boost::shared_ptr<symbol_sync_cc> sptr;

      /*!
       * Make a Symbol Synchronizer block.
       *
       * \details
       * This implements a discrete-time error-tracking synchronizer.
       *
       * For this block to work properly, the input stream must meet the
       * following requirements:
       *
       * 1. the input pulses must have peaks (not flat), which usually can
       * be implemented by using a matched filter before this block.
       *
       * 2. the input pulses should nominally match the normalized
       * slicer constellation, which is normalized to an average symbol
       * magnitude of 1.0 over the entire constellation.
       *
       * \param detector_type
       * The enumerated type of timing error detector to use.
       * See the timing_error_detector class for a list of possible types.
       *
       * \param sps
       * User specified nominal clock period in samples per symbol.
       *
       * \param loop_bw
       * Approximate normailzed loop bandwidth of the symbol clock tracking
       * loop. It should nominally be close to 0, but greater than 0.  If
       * unsure, start with a number around 0.040, and experiment to find the
       * value that works best for your situation.
       *
       * \param damping_factor
       * Damping factor of the symbol clock tracking loop.
       * Damping < 1.0f is an under-damped loop.
       * Damping = 1.0f is a critically-damped loop.
       * Damping > 1.0f is an over-damped loop.
       * One should generally use an over-damped loop for symbol clock tracking.
       *
       * \param max_deviation
       * Maximum absolute deviation of the average clock period estimate
       * from the user specified nominal clock period in samples per symbol.
       *
       * \param osps
       * The number of output samples per symbol (default=1).
       *
       * \param slicer
       * A constellation obj shared pointer that will be used by
       * decision directed timing error detectors to make decisions.
       * I.e. the timing error detector will use this constellation
       * as a slicer, if the particular algorithm needs sliced
       * symbols.
       *
       */
      static sptr make(timing_error_detector::ted_type detector_type,
                       float sps,
                       float loop_bw,
                       float damping_factor = 2.0f,
                       float max_deviation = 1.5f,
                       int osps = 1,
                       digital::constellation_sptr slicer =
                                                 digital::constellation_sptr());
      
      virtual float loop_bandwidth() const = 0;
      virtual float damping_factor() const = 0;
      virtual float alpha() const = 0;
      virtual float beta() const = 0;

      virtual void set_loop_bandwidth (float fn_norm) = 0;
      virtual void set_damping_factor (float zeta) = 0;
      virtual void set_alpha (float alpha) = 0;
      virtual void set_beta (float beta) = 0;
    };

  } /* namespace nwr */
} /* namespace gr */

#endif /* INCLUDED_NWR_SYMBOL_SYNC_CC_H */
