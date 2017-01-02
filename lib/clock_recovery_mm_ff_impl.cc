/* -*- c++ -*- */
/*
 * Copyright 2004,2010-2012,2014 Free Software Foundation, Inc.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "clock_recovery_mm_ff_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/math.h>
#include <stdexcept>

namespace gr {
  namespace nwr {

    clock_recovery_mm_ff::sptr
    clock_recovery_mm_ff::make(float sps,
                               float loop_bw,
                               float damping_factor,
                               float max_deviation)
    {
      return gnuradio::get_initial_sptr
        (new clock_recovery_mm_ff_impl(sps,
                                       loop_bw,
                                       damping_factor,
                                       max_deviation));
    }

    clock_recovery_mm_ff_impl::clock_recovery_mm_ff_impl(float sps,
                                                         float loop_bw,
                                                         float damping_factor,
                                                         float max_deviation)
      : block("clock_recovery_mm_ff",
              io_signature::make(1, 1, sizeof(float)),
              io_signature::makev(1, 4, std::vector<int>(4, sizeof(float)))),
        d_prev_y(0.0f),
        d_prev2_y(0.0f),
        d_interp(new filter::mmse_fir_interpolator_ff()),
        d_interp_fraction(0.0f),
        d_prev_interp_fraction(0.0f),
        d_tags(),
        d_new_tags(),
        d_time_est_key(pmt::intern("time_est")),
        d_clock_est_key(pmt::intern("clock_est")),
        d_noutputs(1),
        d_out_error(NULL),
        d_out_instantaneous_clock_period(NULL),
        d_out_average_clock_period(NULL)
    {
      if (sps <= 1.0f)
        throw std::out_of_range("nominal samples per symbol must be > 1");

      // Symbol Clock Tracking and Estimation
      d_clock = new clock_tracking_loop(loop_bw,
                                        sps + max_deviation,
                                        sps - max_deviation,
                                        sps,
                                        damping_factor);

      // Timing Error Detector
      d_prev_decision = slice(d_prev_y);
      d_prev2_decision = slice(d_prev2_y);

      // Tag Propagation and Clock Tracking Reset/Resync
      set_relative_rate (1.0 / sps);
      set_tag_propagation_policy(TPP_DONT);
      d_filter_delay = (d_interp->ntaps() + 1) / 2;
    }

    clock_recovery_mm_ff_impl::~clock_recovery_mm_ff_impl()
    {
      delete d_interp;
      delete d_clock;
    }

    void
    clock_recovery_mm_ff_impl::forecast(int noutput_items,
                                        gr_vector_int &ninput_items_required)
    {
      unsigned ninputs = ninput_items_required.size();
      // The '+ 2' in the expression below is an effort to always have at least
      // one output sample, even if the main loop decides it has to revert
      // one computed sample and wait for the next call to general_work().
      // The d_clock->get_max_avg_period() is also an effort to do the same,
      // in case we have the worst case allowable clock timing deviation on
      // input.
      for(unsigned i=0; i < ninputs; i++)
        ninput_items_required[i] =
                        static_cast<int>(ceilf((noutput_items + 2)
                                               * d_clock->get_max_avg_period()))
                        + static_cast<int>(d_interp->ntaps());
    }

    //
    // Timing Error Detector
    //
    float
    clock_recovery_mm_ff_impl::slice(float x)
    {
      return x < 0 ? -1.0F : 1.0F;
    }

    float
    clock_recovery_mm_ff_impl::timing_error_detector(float curr_y)
    {
        float error;
        float curr_decision = slice(curr_y);

        error = d_prev_decision * curr_y - curr_decision * d_prev_y;

        d_prev2_y = d_prev_y;
        d_prev2_decision = d_prev_decision;

        d_prev_y = curr_y;
        d_prev_decision = curr_decision;

        return error;
    }

    void
    clock_recovery_mm_ff_impl::revert_timing_error_detector_state()
    {
        d_prev_y = d_prev2_y;
        d_prev_decision = d_prev2_decision;
    }

    //
    // Symbol Clock and Interpolator Positioning and Alignment
    //
    void
    clock_recovery_mm_ff_impl::sample_distance_phase_wrap(float d,
                                                          int &n, float &f)
    {
        float whole_samples = roundf(d);
        f = d - whole_samples;
        n = static_cast<int>(whole_samples);
    }

    int
    clock_recovery_mm_ff_impl::distance_from_current_input()
    {
        float d;
        int whole_samples_until_clock;

        d_prev_interp_fraction = d_interp_fraction;

        d = d_interp_fraction + d_clock->get_inst_period();

        sample_distance_phase_wrap(d, whole_samples_until_clock,
                                   d_interp_fraction);
        if (d_interp_fraction < 0.0f) {
            d_interp_fraction += 1.0f;
            whole_samples_until_clock--;
        }
        return whole_samples_until_clock;
    }

    void
    clock_recovery_mm_ff_impl::revert_distance_state()
    {
        d_interp_fraction = d_prev_interp_fraction;
    }

    void
    clock_recovery_mm_ff_impl::sync_reset_timing_error_detector()
    {
        // force next the next timing error to be 0.0f
        d_prev_y = 0.0f;
        d_prev_decision = 0.0f;
        d_prev2_y = 0.0f;
        d_prev2_decision = 0.0f;
    }

    //
    // Tag Propagation and Clock Tracking Reset/Resync
    //
    void
    clock_recovery_mm_ff_impl::collect_tags(uint64_t nitems_rd, int count)
    {
        // Get all the tags in offset order
        // d_new_tags is used to look for time_est and clock_est tags.
        // d_tags is used for manual tag propagation.
        d_new_tags.clear();
        get_tags_in_range(d_new_tags, 0, nitems_rd, nitems_rd + count);
        std::sort(d_new_tags.begin(), d_new_tags.end(), tag_t::offset_compare);
        d_tags.insert(d_tags.end(), d_new_tags.begin(), d_new_tags.end());
        std::sort(d_tags.begin(), d_tags.end(), tag_t::offset_compare);
    }

    bool
    clock_recovery_mm_ff_impl::find_sync_tag(uint64_t nitems_rd, int iidx,
                                             int clock_distance,
                                             uint64_t &tag_offset,
                                             float &timing_offset,
                                             float &clock_period)
    {
        bool found;
        uint64_t soffset, eoffset;
        std::vector<tag_t>::iterator t;
        std::vector<tag_t>::iterator t2;

        // PLL Reset/Resynchronization to time_est & clock_est tags (1st part)
        //
        // Look for a time_est tag between the current interpolated input sample
        // and the next predicted interpolated input sample. (both rounded up)
        soffset = nitems_rd + d_filter_delay + static_cast<uint64_t>(iidx + 1);
        eoffset = soffset + clock_distance;
        found = false;
        for (t = d_new_tags.begin();
             t != d_new_tags.end();
             t = d_new_tags.erase(t)) {

            if (t->offset > eoffset) // search finished
                break;

            if (t->offset < soffset) // tag is in the past of what we care about
                continue;

            if (not pmt::eq(t->key, d_time_est_key) and  // not a time_est tag
                not pmt::eq(t->key, d_clock_est_key)   ) // not a clock_est tag
                continue;

            found = true;
            tag_offset = t->offset;
            if (pmt::eq(t->key, d_time_est_key)) {
                // got a time_est tag
                timing_offset = static_cast<float>(pmt::to_double(t->value));
                // next instantaneous clock period estimate will be nominal
                clock_period = d_clock->get_nom_avg_period();

                // Look for a clock_est tag at the same offset,
                // as we prefer clock_est tags
                for (t2 = ++t; t2 != d_new_tags.end(); ++t2) {
                    if (t2->offset > t->offset) // search finished
                        break;
                    if (not pmt::eq(t->key, d_clock_est_key)) // not a clock_est
                        continue;
                    // Found a clock_est tag at the same offset
                    tag_offset = t2->offset;
                    timing_offset = static_cast<float>(
                                  pmt::to_double(pmt::tuple_ref(t2->value, 0)));
                    clock_period = static_cast<float>(
                                  pmt::to_double(pmt::tuple_ref(t2->value, 1)));
                    break;
                }
            } else {
                // got a clock_est tag
                timing_offset = static_cast<float>(
                                   pmt::to_double(pmt::tuple_ref(t->value, 0)));
                clock_period = static_cast<float>(
                                   pmt::to_double(pmt::tuple_ref(t->value, 1)));
            }

            if (not(timing_offset >= -1.0f and timing_offset <= 1.0f)) {
                // the time_est/clock_est tag's payload is invalid
                GR_LOG_WARN(d_logger,
                            boost::format("ignoring time_est/clock_est tag with"
                                          " value %.2f, outside of allowed "
                                          "range [-1.0, 1.0]") % timing_offset);
                found = false;
                continue;
            }

            if (t->offset == soffset and timing_offset < 0.0f) {
                // already handled clock times earlier than this previously
                found = false;
                continue;
            }

            if (t->offset == eoffset and timing_offset >= 0.0f) {
                // handle clock times greater than this later
                found = false;
                break;
            }

            if (found == true)
                break;
        }
        return found;
    }

    void
    clock_recovery_mm_ff_impl::propagate_tags(uint64_t nitems_rd, int iidx,
                                              float inst_clock_distance,
                                              float inst_clock_period,
                                              uint64_t nitems_wr, int oidx)
    {
        // Tag Propagation
        //
        // Onto this output sample, place all the remaining tags that
        // came before the interpolated input sample, and all the tags
        // on and after the interpolated input sample, up to half way to
        // the next interpolated input sample.

        uint64_t mid_period_offset = nitems_rd + d_filter_delay
                    + static_cast<uint64_t>(iidx)
                    + static_cast<uint64_t>(llroundf(inst_clock_distance
                                                     - inst_clock_period/2.0f));

        uint64_t output_offset = nitems_wr + static_cast<uint64_t>(oidx);

        int i;
        std::vector<tag_t>::iterator t;
        for (t = d_tags.begin();
             t != d_tags.end() and t->offset <= mid_period_offset;
             t = d_tags.erase(t)) {
            t->offset = output_offset;
            for (i = 0; i < d_noutputs; i++)
                add_item_tag(i, *t);
        }
    }

    void
    clock_recovery_mm_ff_impl::save_expiring_tags(uint64_t nitems_rd,
                                                  int consumed)
    {
        // Deferred Tag Propagation
        //
        // Only save away input tags that will not be available
        // in the next call to general_work().  Otherwise we would
        // create duplicate tags next time around.
        // Tags that have already been propagated, have already been erased
        // from d_tags.

        uint64_t consumed_offset = nitems_rd + static_cast<uint64_t>(consumed);
        std::vector<tag_t>::iterator t;

        for (t = d_tags.begin(); t != d_tags.end(); ) {
            if (t->offset < consumed_offset)
                ++t;
            else
                t = d_tags.erase(t);
        }
    }

    //
    // Optional Diagnostic Outputs
    //
    void
    clock_recovery_mm_ff_impl::setup_optional_outputs(
                                              gr_vector_void_star &output_items)
    {
        d_noutputs = output_items.size();
        d_out_error = NULL;
        d_out_instantaneous_clock_period = NULL;
        d_out_average_clock_period = NULL;

        if (d_noutputs < 2)
            return;
        d_out_error = (float *) output_items[1];

        if (d_noutputs < 3)
            return;
        d_out_instantaneous_clock_period = (float *) output_items[2];

        if (d_noutputs < 4)
            return;
        d_out_average_clock_period = (float *) output_items[3];
    }

    void
    clock_recovery_mm_ff_impl::emit_optional_output(int oidx,
                                                    float error,
                                                    float inst_clock_period,
                                                    float avg_clock_period)
    {
        if (d_noutputs < 2)
            return;
        d_out_error[oidx] = error;

        if (d_noutputs < 3)
            return;
        d_out_instantaneous_clock_period[oidx] = inst_clock_period;

        if (d_noutputs < 4)
            return;
        d_out_average_clock_period[oidx] = avg_clock_period;
    }

    int
    clock_recovery_mm_ff_impl::general_work(
                                         int noutput_items,
                                         gr_vector_int &ninput_items,
                                         gr_vector_const_void_star &input_items,
                                         gr_vector_void_star &output_items)
    {
      // max input to consume
      int ni = ninput_items[0] - static_cast<int>(d_interp->ntaps());
      if (ni <= 0)
          return 0;

      const float *in = (const float *)input_items[0];
      float *out = (float *)output_items[0];

      setup_optional_outputs(output_items);

      int ii = 0; // input index
      int oo = 0; // output index
      float error;
      float inst_clock_period; // between interpolated samples
      float inst_clock_distance; // from an input sample's position
      float avg_clock_period;
      int n;

      uint64_t nitems_rd = nitems_read(0);
      uint64_t nitems_wr = nitems_written(0);
      uint64_t sync_tag_offset;
      float sync_timing_offset;
      float sync_clock_period;

      // Tag Propagation and Symbol Clock Tracking Reset/Resync
      collect_tags(nitems_rd, ni);

      while (oo < noutput_items) {
        // Application of Symbol Clock and Interpolator Positioning & Alignment
        //
        // produce output sample
        out[oo] = d_interp->interpolate(&in[ii], d_interp_fraction);

        // Timing Error Detector
        error = timing_error_detector(out[oo]);

        // Symbol Clock Tracking and Estimation
        d_clock->advance_loop(error);
        avg_clock_period = d_clock->get_avg_period();
        inst_clock_period = d_clock->get_inst_period();
        d_clock->phase_wrap();
        d_clock->period_limit();

        // Diagnostic Output of Symbol Clock Tracking cycle results
        emit_optional_output(oo, error, inst_clock_period, avg_clock_period);

        // Symbol Clock and Interpolator Positioning & Alignment
        n = distance_from_current_input();
        inst_clock_distance = sample_distance_phase_unwrap(n,
                                                           d_interp_fraction);
        if (ii + n >= ni) {
            // This check and revert is needed when the samples per
            // symbol is greater than d_interp->ntaps() (normally 8);
            // otherwise we would consume() more input than we were
            // given.

            // Symbol Clock and Interpolator Positioning & Alignment
            revert_distance_state();
            // Symbol Clock Tracking and Estimation
            d_clock->revert_loop();
            // Timing Error Detector
            revert_timing_error_detector_state();
            break;
        }

        // Symbol Clock Tracking Reset/Resync to time_est and clock_est tags
        if (find_sync_tag(nitems_rd, ii, n, sync_tag_offset,
                          sync_timing_offset, sync_clock_period) == true) {

            //
            // Symbol Clock and Interpolator Positioning & Alignment
            // Symbol Clock Tracking and Estimation
            //

            // Adjust this instantaneous clock period to land right where
            // time_est/clock_est indicates.  Fix up PLL state as necessary.
            revert_distance_state();

            // NOTE: the + 1 below was determined empirically, but doesn't
            // seem right on paper (maybe rounding in the computation of
            // d_filter_delay is the culprit).  Anyway, experiment trumps
            // theory *every* time; so + 1 it is.
            inst_clock_distance = static_cast<float>(
                  static_cast<int>(sync_tag_offset - nitems_rd - d_filter_delay)
                  - ii + 1)
                  + sync_timing_offset;
            inst_clock_period = inst_clock_distance - d_interp_fraction;

            d_clock->set_inst_period(inst_clock_period);
            n = distance_from_current_input();

            // next instantaneous clock period estimate will match the nominal
            // or comes from the clock_est tag
            d_clock->set_avg_period(sync_clock_period);
            avg_clock_period = d_clock->get_avg_period();

            // Timing Error Detector
            sync_reset_timing_error_detector();

            // Revised Diagnostic Output of Symbol Clock Tracking cycle results
            emit_optional_output(oo, 0.0f, inst_clock_period, avg_clock_period);
        }

        // Tag Propagation
        propagate_tags(nitems_rd, ii, inst_clock_distance, inst_clock_period,
                       nitems_wr, oo);

        // Symbol Clock and Interpolator Positioning & Alignment
        ii += n;
        oo++;
      }

      // Deferred Tag Propagation
      save_expiring_tags(nitems_rd, ii);

      // Symbol Clock and Interpolator Positioning & Alignment
      consume_each(ii);
      return oo;
    }

  } /* namespace nwr */
} /* namespace gr */
