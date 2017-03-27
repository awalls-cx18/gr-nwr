/* -*- c++ -*- */
/*
 * Copyright 2004,2010-2013 Free Software Foundation, Inc.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "pll_refout_cc_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/sincos.h>
#include <math.h>
#include <gnuradio/math.h>

namespace gr {
  namespace nwr {

#ifndef M_TWOPI
#define M_TWOPI (2.0f*M_PI)
#endif

    pll_refout_cc::sptr
    pll_refout_cc::make(float loop_bw, float max_freq, float min_freq,
                        float df, float alpha, float beta)
    {
      return gnuradio::get_initial_sptr
	(new pll_refout_cc_impl(loop_bw, max_freq, min_freq, df, alpha, beta));
    }

    pll_refout_cc_impl::pll_refout_cc_impl(float loop_bw,
                                           float max_freq, float min_freq,
                                           float df, float alpha, float beta)
      : sync_block("pll_refout_cc",
		   io_signature::make(1, 1, sizeof(gr_complex)),
                   io_signature::makev(1, 4, std::vector<int>(4, sizeof(float)))),
	blocks::control_loop(loop_bw, max_freq, min_freq),
        d_noutputs(1),
        d_out_error(NULL),
        d_out_instantaneous_phase_inc(NULL),
        d_out_average_phase_inc(NULL)
    {
      // Brute force fix of the output io_signature, because I can't get
      // an anonymous std::vector<int>() rvalue, with a const expression
      // initializing the vector, to work.  Lvalues seem to make everything
      // better.
      int output_io_sizes[4] = {
          sizeof(gr_complex),
          sizeof(float), sizeof(float), sizeof(float)
      };
      std::vector<int> output_io_sizes_vector(&output_io_sizes[0],
                                              &output_io_sizes[4]);
      set_output_signature(io_signature::makev(1, 4, output_io_sizes_vector));

      if (df >= 0.0f)
          set_damping_factor(df);

      if (alpha >= 0.0f and beta >= 0.0f) {
          set_alpha(alpha);
          set_beta(beta);
      }
    }

    pll_refout_cc_impl::~pll_refout_cc_impl()
    {
    }

    void
    pll_refout_cc_impl::setup_optional_outputs(
                                              gr_vector_void_star &output_items)
    {
        d_noutputs = output_items.size();
        d_out_error = NULL;
        d_out_instantaneous_phase_inc = NULL;
        d_out_average_phase_inc = NULL;

        if (d_noutputs < 2)
            return;
        d_out_error = (float *) output_items[1];

        if (d_noutputs < 3)
            return;
        d_out_instantaneous_phase_inc = (float *) output_items[2];

        if (d_noutputs < 4)
            return;
        d_out_average_phase_inc = (float *) output_items[3];
    }

    void
    pll_refout_cc_impl::emit_optional_output(int oidx,
                                             float error,
                                             float inst_phase_inc,
                                             float avg_phase_inc)
    {
        if (d_noutputs < 2)
            return;
        d_out_error[oidx] = error;

        if (d_noutputs < 3)
            return;
        d_out_instantaneous_phase_inc[oidx] = inst_phase_inc;

        if (d_noutputs < 4)
            return;
        d_out_average_phase_inc[oidx] = avg_phase_inc;
    }

    float
    pll_refout_cc_impl::mod_2pi(float in)
    {
      if(in > M_PI)
	return in - M_TWOPI;
      else if(in < -M_PI)
	return in+ M_TWOPI;
      else
	return in;
    }

    float
    pll_refout_cc_impl::phase_detector(gr_complex sample,float ref_phase)
    {
      float sample_phase;
      sample_phase = gr::fast_atan2f(sample.imag(),sample.real());
      return mod_2pi(sample_phase-ref_phase);
    }

    int
    pll_refout_cc_impl::work(int noutput_items,
			     gr_vector_const_void_star &input_items,
			     gr_vector_void_star &output_items)
    {
      const gr_complex *iptr = (gr_complex*)input_items[0];
      gr_complex *optr = (gr_complex*)output_items[0];

      setup_optional_outputs(output_items);

      float error, prev_phase;
      float t_imag, t_real;

      for(int idx = 0; idx < noutput_items; idx++) {
	gr::sincosf(d_phase,&t_imag,&t_real);
	*optr++ = gr_complex(t_real,t_imag);

	error = phase_detector(*iptr++,d_phase);

        prev_phase = d_phase;
	advance_loop(error);
        emit_optional_output(idx, error, d_phase - prev_phase, d_freq);
	phase_wrap();
	frequency_limit();
      }
      return noutput_items;
    }

    void
    pll_refout_cc_impl::set_loop_bandwidth(float bw)
    {
      blocks::control_loop::set_loop_bandwidth(bw);
    }

    void
    pll_refout_cc_impl::set_damping_factor(float df)
    {
      blocks::control_loop::set_damping_factor(df);
    }

    void
    pll_refout_cc_impl::set_alpha(float alpha)
    {
      blocks::control_loop::set_alpha(alpha);
    }

    void
    pll_refout_cc_impl::set_beta(float beta)
    {
      blocks::control_loop::set_beta(beta);
    }

    void
    pll_refout_cc_impl::set_frequency(float freq)
    {
      blocks::control_loop::set_frequency(freq);
    }

    void
    pll_refout_cc_impl::set_phase(float phase)
    {
      blocks::control_loop::set_phase(phase);
    }

    void
    pll_refout_cc_impl::set_min_freq(float freq)
    {
      blocks::control_loop::set_min_freq(freq);
    }

    void
    pll_refout_cc_impl::set_max_freq(float freq)
    {
      blocks::control_loop::set_max_freq(freq);
    }


    float
    pll_refout_cc_impl::get_loop_bandwidth() const
    {
      return blocks::control_loop::get_loop_bandwidth();
    }

    float
    pll_refout_cc_impl::get_damping_factor() const
    {
      return blocks::control_loop::get_damping_factor();
    }

    float
    pll_refout_cc_impl::get_alpha() const
    {
      return blocks::control_loop::get_alpha();
    }

    float
    pll_refout_cc_impl::get_beta() const
    {
      return blocks::control_loop::get_beta();
    }

    float
    pll_refout_cc_impl::get_frequency() const
    {
      return blocks::control_loop::get_frequency();
    }

    float
    pll_refout_cc_impl::get_phase() const
    {
      return blocks::control_loop::get_phase();
    }

    float
    pll_refout_cc_impl::get_min_freq() const
    {
      return blocks::control_loop::get_min_freq();
    }

    float
    pll_refout_cc_impl::get_max_freq() const
    {
      return blocks::control_loop::get_max_freq();
    }

  } /* namespace nwr */
} /* namespace gr */
