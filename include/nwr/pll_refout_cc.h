/* -*- c++ -*- */
/*
 * Copyright 2004,2011,2012 Free Software Foundation, Inc.
 * Copyright (C) 2017  Andy Walls <awalls.cx18@gmail.com>
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

#ifndef INCLUDED_NWR_PLL_REFOUT_CC_H
#define INCLUDED_NWR_PLL_REFOUT_CC_H

#include <nwr/api.h>
#include <gnuradio/blocks/control_loop.h>
#include <gnuradio/sync_block.h>
#include <cmath>

namespace gr {
  namespace nwr {

    /*!
     * \brief Implements a PLL which locks to the input frequency and outputs a carrier
     * \ingroup synchronizers_blk
     *
     * \details
     * Input stream 0: complex
     * Output stream 0: complex
     *
     * This PLL locks onto a [possibly noisy] reference carrier on the
     * input and outputs a clean version which is phase and frequency
     * aligned to it.
     *
     * All settings max_freq and min_freq are in terms of radians per
     * sample, NOT HERTZ. The loop bandwidth determins the lock range.
     */
    class NWR_API pll_refout_cc
      : virtual public sync_block,
        virtual public blocks::control_loop
    {
    public:
      // gr::nwr::pll_refout_cc::sptr
      typedef boost::shared_ptr<pll_refout_cc> sptr;

      /* \brief Make PLL block that outputs the tracked carrier signal.
       *
       * \param loop_bw: control loop's bandwidth parameter.
       * \param max_freq: maximum (normalized) frequency PLL will lock to.
       * \param min_freq: minimum (normalized) frequency PLL will lock to.
       * \param df: damping factor, default is 1.0/sqrt(2.0)
       * \param alpha: PI filter proportional gain, overrides loop_bw & df.
       * \param beta: PI filter integral gain, overrides loop_bw & df.
       */
      static sptr make(float loop_bw, float max_freq, float min_freq,
                       float df = M_SQRT1_2,
                       float alpha = -1.0f, float beta = -1.0f);
    };

  } /* namespace nwr */
} /* namespace gr */

#endif /* INCLUDED_NWR_PLL_REFOUT_CC_H */
