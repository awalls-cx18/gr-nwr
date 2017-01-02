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
#include <nwr/clock_tracking_loop.h>
#include <gnuradio/filter/mmse_fir_interpolator_ff.h>

namespace gr {
  namespace nwr {

    class clock_recovery_mm_ff_impl : public clock_recovery_mm_ff
    {
    public:
      clock_recovery_mm_ff_impl(float sps,
                                float loop_bw,
                                float damping_factor,
                                float max_deviation);
      ~clock_recovery_mm_ff_impl();

      void forecast(int noutput_items, gr_vector_int &ninput_items_required);
      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);

      // Symbol Clock Tracking and Estimation
      float loop_bandwidth() const { return d_clock->get_loop_bandwidth(); }
      float damping_factor() const { return d_clock->get_damping_factor(); }
      float alpha() const { return d_clock->get_alpha(); }
      float beta() const { return d_clock->get_beta(); }

      void set_loop_bandwidth (float fn_norm) {
          d_clock->set_loop_bandwidth(fn_norm);
      }
      void set_damping_factor (float zeta) {
          d_clock->set_damping_factor(zeta);
      }
      void set_alpha (float alpha) { d_clock->set_alpha(alpha); }
      void set_beta (float beta) { d_clock->set_beta(beta); }

    private:
      // Timing Error Detector
      float d_prev_y;
      float d_prev_decision;
      float d_prev2_y;
      float d_prev2_decision;

      // Symbol Clock Tracking and Estimation
      clock_tracking_loop *d_clock;

      // Interpolator and Interpolator Positioning and Alignment
      filter::mmse_fir_interpolator_ff *d_interp;
      float d_interp_fraction; // a.k.a. interpolator sample phase
      float d_prev_interp_fraction;

      // Tag Propagation and Symbol Clock Tracking Reset/Resync
      uint64_t d_filter_delay; // interpolator filter delay
      std::vector<tag_t> d_tags;
      std::vector<tag_t> d_new_tags;
      pmt::pmt_t d_time_est_key;
      pmt::pmt_t d_clock_est_key;

      // Optional Diagnostic Outputs
      int d_noutputs;
      float *d_out_error;
      float *d_out_instantaneous_clock_period;
      float *d_out_average_clock_period;

      // Timing Error Detector
      float slice(float x);
      float timing_error_detector(float curr_y);
      void revert_timing_error_detector_state();
      void sync_reset_timing_error_detector();

      // Symbol Clock and Interpolator Positioning and Alignment
      int distance_from_current_input();
      void sample_distance_phase_wrap(float d, int &n, float &f);
      float sample_distance_phase_unwrap(int n, float f) {
          return static_cast<float>(n) + f;
      }
      void revert_distance_state();

      // Tag Propagation and Clock Tracking Reset/Resync
      void collect_tags(uint64_t nitems_rd, int count);
      bool find_sync_tag(uint64_t nitems_rd, int iidx,
                         int clock_distance,
                         uint64_t &tag_offset,
                         float &timing_offset,
                         float &clock_period);
      void propagate_tags(uint64_t nitems_rd, int iidx,
                          float inst_clock_distance,
                          float inst_clock_period,
                          uint64_t nitems_wr, int oidx);
      void save_expiring_tags(uint64_t nitems_rd, int consumed);

      // Optional Diagnostic Outputs
      void setup_optional_outputs(gr_vector_void_star &output_items);
      void emit_optional_output(int oidx,
                                float error,
                                float inst_clock_period,
                                float avg_clock_period);
    };

  } /* namespace nwr */
} /* namespace gr */

#endif /* INCLUDED_NWR_CLOCK_RECOVERY_MM_FF_IMPL_H */
