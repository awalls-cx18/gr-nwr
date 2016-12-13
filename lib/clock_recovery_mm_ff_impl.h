/* -*- c++ -*- */
/*
 * Copyright 2004,2011,2012,2014 Free Software Foundation, Inc.
 * Copyright (C) 2016  Andy Walls <awalls.cx18@gmail.com>
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

#ifndef INCLUDED_NWR_CLOCK_RECOVERY_MM_FF_IMPL_H
#define	INCLUDED_NWR_CLOCK_RECOVERY_MM_FF_IMPL_H

#include <nwr/clock_recovery_mm_ff.h>
#include <gnuradio/filter/mmse_fir_interpolator_ff.h>

namespace gr {
  namespace nwr {

    class clock_recovery_mm_ff_impl : public clock_recovery_mm_ff
    {
    public:
      clock_recovery_mm_ff_impl(float omega, float gain_omega,
                                float mu, float gain_mu,
                                float omega_relative_limit);
      ~clock_recovery_mm_ff_impl();

      void forecast(int noutput_items, gr_vector_int &ninput_items_required);
      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);

      float mu() const { return d_mu;}
      // d_omega is the tracked samples/symbol - 0.5, because the
      // sample phase wrapped d_mu ranges in
      // [0.0f, 1.0f] instead of [-0.5f, 0.5f]
      float omega() const { return d_omega + 0.5f;}
      float gain_mu() const { return d_gain_mu;}
      float gain_omega() const { return d_gain_omega;}

      void set_verbose (bool verbose) { d_verbose = verbose; }
      void set_gain_mu (float gain_mu) { d_gain_mu = gain_mu; }
      void set_gain_omega (float gain_omega) { d_gain_omega = gain_omega; }
      void set_mu (float mu) { d_mu = mu; }
      void set_omega (float omega);

    private:
      float d_mu;                   // fractional sample position [0.0, 1.0]
      float d_gain_mu;              // gain for adjusting mu
      float d_omega;                // nominal frequency - 0.5f
      float d_gain_omega;           // gain for adjusting omega
      float d_omega_relative_limit; // used to compute min and max omega
      float d_omega_mid;            // average omega
      float d_omega_lim;            // actual omega clipping limit

      float d_prev_y;
      float d_prev_decision;
      float d_interp_fraction;
      filter::mmse_fir_interpolator_ff *d_interp;

      bool d_verbose;

      // For reverting the process state back one interation
      float d_prev_mu;
      float d_prev_omega;
      float d_prev2_y;
      float d_prev2_decision;
      float d_prev_interp_fraction;

      float slice(float x);
      float timing_error_detector(float curr_y);
      void symbol_period_limit();
      void advance_loop(float error);
      int clock_sample_phase_wrap();
      int distance_from_current_input(int mu_int);

      void revert_distance_state();
      void revert_loop_state();
      void revert_timing_error_detector_state();
    };

  } /* namespace nwr */
} /* namespace gr */

#endif /* INCLUDED_NWR_CLOCK_RECOVERY_MM_FF_IMPL_H */
