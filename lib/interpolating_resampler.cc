/* -*- c++ -*- */
/*
 * Copyright (C) 2017  Andy Walls <awalls.cx18@gmail.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this file; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <nwr/interpolating_resampler.h>
#include <gnuradio/math.h>
#include <stdexcept>

namespace gr {
  namespace nwr {

    interpolating_resampler::interpolating_resampler(enum ir_type type,
                                                     bool derivative)
      : d_type(type),
        d_derivative(derivative),
        d_phase(0.0f),
        d_phase_wrapped(0.0f),
        d_phase_n(0),
        d_prev_phase(0.0f),
        d_prev_phase_wrapped(0.0f),
        d_prev_phase_n(0)
    {
        switch (d_type) {
        case IR_MMSE_8TAP:
            break;
        case IR_PFB_NO_MF:
            break;
        case IR_PFB_MF:
        case IR_NONE:
        default: 
            throw std::invalid_argument(
              "interpolating_resampler: invalid interpolating resampler type.");
            break;
        }

        sync_reset(0.0f);
    }

    void 
    interpolating_resampler::next_phase(float increment,
                                        float &phase,
                                        int   &phase_n,
                                        float &phase_wrapped)
    {
        float n;

        phase = d_phase_wrapped + increment;
        n = floorf(phase);
        phase_wrapped = phase - n;
        phase_n = static_cast<int>(n);
    }

    void 
    interpolating_resampler::advance_phase(float increment)
    {
        d_prev_phase = d_phase;
        d_prev_phase_wrapped = d_phase_wrapped;
        d_prev_phase_n = d_phase_n;

        next_phase(increment, d_phase, d_phase_n, d_phase_wrapped);
    }

    void
    interpolating_resampler::revert_phase()
    {
        d_phase = d_prev_phase;
        d_phase_wrapped = d_prev_phase_wrapped;
        d_phase_n = d_prev_phase_n;
    }

    void
    interpolating_resampler::sync_reset(float phase)
    {
        float n;

        d_phase = phase;
        n = floorf(d_phase);
        d_phase_wrapped = d_phase - n;
        d_phase_n = static_cast<int>(n);

        d_prev_phase = d_phase;
        d_prev_phase_wrapped = d_phase_wrapped;
        d_prev_phase_n = d_phase_n;
    }

    /*************************************************************************/

    interpolating_resampler_ccf *
    interpolating_resampler_ccf::make(enum ir_type type,
                                      bool derivative,
                                      int nfilts,
                                      const std::vector<float> &taps)
    {
        interpolating_resampler_ccf *ret = NULL;
        switch (type) {
        case IR_MMSE_8TAP:
            ret = new interp_resampler_mmse_8tap_cc(derivative);
            break;
        case IR_PFB_NO_MF:
            ret = new interp_resampler_pfb_no_mf_cc(derivative, nfilts);
            break;
        case IR_PFB_MF:
            if (nfilts <= 1)
               throw std::invalid_argument("interpolating_resampler_ccf: "
                                           "number of polyphase filter arms "
                                           "must be greater than 1");
            if (taps.size() < nfilts)
               throw std::invalid_argument("interpolating_resampler_ccf: "
                                           "length of the prototype filter taps"
                                           " must be greater than or equal to "
                                           "the number of polyphase filter arms"
                                           ".");
        case IR_NONE:
        default: 
            throw std::invalid_argument("interpolating_resampler_ccf: invalid "
                                        "interpolating resampler type.");
            break;
        }
        return ret;
    }

    /*************************************************************************/

    interpolating_resampler_fff *
    interpolating_resampler_fff::make(enum ir_type type,
                                      bool derivative,
                                      int nfilts,
                                      const std::vector<float> &taps)
    {
        interpolating_resampler_fff *ret = NULL;
        switch (type) {
        case IR_MMSE_8TAP:
            ret = new interp_resampler_mmse_8tap_ff(derivative);
            break;
        case IR_PFB_NO_MF:
            ret = new interp_resampler_pfb_no_mf_ff(derivative, nfilts);
            break;
        case IR_PFB_MF:
            if (nfilts <= 1)
               throw std::invalid_argument("interpolating_resampler_fff: "
                                           "number of polyphase filter arms "
                                           "must be greater than 1");
            if (taps.size() < nfilts)
               throw std::invalid_argument("interpolating_resampler_fff: "
                                           "length of the prototype filter taps"
                                           " must be greater than or equal to "
                                           "the number of polyphase filter arms"
                                           ".");
        case IR_NONE:
        default: 
            throw std::invalid_argument("interpolating_resampler_fff: invalid "
                                        "interpolating resampler type.");
            break;
        }
        return ret;
    }

    /*************************************************************************/

    interp_resampler_mmse_8tap_cc::interp_resampler_mmse_8tap_cc(
                                                                bool derivative)
    : interpolating_resampler_ccf(interpolating_resampler::IR_MMSE_8TAP,
                                  derivative),
      d_interp(NULL),
      d_interp_diff(NULL)
    {
        d_interp = new gr::filter::mmse_fir_interpolator_cc(); 
        if (d_interp == NULL)
          throw std::runtime_error("unable to create mmse_fir_interpolator_cc");

        if (d_derivative) {
            d_interp_diff = new mmse_interp_differentiator_cc();
            if (d_interp_diff == NULL)
                throw std::runtime_error("unable to create "
                                         "mmse_interp_differentiator_cc");
        }
    }

    interp_resampler_mmse_8tap_cc::~interp_resampler_mmse_8tap_cc()
    {
        delete d_interp;
        if (d_derivative)
            delete d_interp_diff;
    }

    gr_complex
    interp_resampler_mmse_8tap_cc::interpolate(const gr_complex input[],
                                               float mu) const
    {
        return d_interp->interpolate(input, mu);
    }

    gr_complex
    interp_resampler_mmse_8tap_cc::differentiate(const gr_complex input[],
                                                 float mu) const
    {
        return d_interp_diff->differentiate(input, mu);
    }

    unsigned int
    interp_resampler_mmse_8tap_cc::ntaps() const
    {
        return d_interp->ntaps();
    }

    /*************************************************************************/

    interp_resampler_mmse_8tap_ff::interp_resampler_mmse_8tap_ff(
                                                                bool derivative)
    : interpolating_resampler_fff(interpolating_resampler::IR_MMSE_8TAP,
                                  derivative),
      d_interp(NULL),
      d_interp_diff(NULL)
    {
        d_interp = new gr::filter::mmse_fir_interpolator_ff(); 
        if (d_interp == NULL)
          throw std::runtime_error("unable to create mmse_fir_interpolator_ff");

        if (d_derivative) {
            d_interp_diff = new mmse_interp_differentiator_ff();
            if (d_interp_diff == NULL)
                throw std::runtime_error("unable to create "
                                         "mmse_interp_differentiator_ff");
        }
    }

    interp_resampler_mmse_8tap_ff::~interp_resampler_mmse_8tap_ff()
    {
        delete d_interp;
        if (d_derivative)
            delete d_interp_diff;
    }

    float
    interp_resampler_mmse_8tap_ff::interpolate(const float input[],
                                               float mu) const
    {
        return d_interp->interpolate(input, mu);
    }

    float
    interp_resampler_mmse_8tap_ff::differentiate(const float input[],
                                                 float mu) const
    {
        return d_interp_diff->differentiate(input, mu);
    }

    unsigned int
    interp_resampler_mmse_8tap_ff::ntaps() const
    {
        return d_interp->ntaps();
    }

    /*************************************************************************/

#include <gnuradio/filter/interpolator_taps.h>
#include <nwr/interp_differentiator_taps.h>

    interp_resampler_pfb_no_mf_cc::interp_resampler_pfb_no_mf_cc(
                                                                bool derivative,
                                                                int nfilts)
    : interpolating_resampler_ccf(interpolating_resampler::IR_PFB_NO_MF,
                                  derivative),
      d_nfilters(0),
      d_filters(),
      d_diff_filters()
    {
        if (nfilts <= 1)
            throw std::invalid_argument("interpolating_resampler_pfb_no_mf_cc: "
                                        "number of polyphase filter arms "
                                        "must be greater than 1");

        // Round up the number of filter arms to the current or next power of 2
        d_nfilters = 
               1 << (static_cast<int>(log2f(static_cast<float>(nfilts-1))) + 1);

        // N.B. We assume in this class: NSTEPS == DNSTEPS and NTAPS == DNTAPS

        // Limit to the maximum number of precomputed MMSE tap sets
        if (d_nfilters <= 0 or d_nfilters > NSTEPS)
            d_nfilters = NSTEPS;

        // Create our polyphase filter arms for the steps from 0.0 to 1.0 from
        // the MMSE interpolating filter and MMSE interpolating differentiator
        // taps rows.
        // N.B. We create an extra final row for an offset of 1.0, because it's
        // easier than dealing with wrap around from 0.99... to 0.0 shifted
        // by 1 tap.
        d_filters = 
         std::vector<gr::filter::kernel::fir_filter_ccf*>(d_nfilters + 1, NULL);
        d_diff_filters = 
         std::vector<gr::filter::kernel::fir_filter_ccf*>(d_nfilters + 1, NULL);

        std::vector<float> t(NTAPS, 0);
        int incr = NSTEPS/d_nfilters;
        int src, dst;
        for (src = 0, dst = 0; src <= NSTEPS; src += incr, dst++) {

            t.assign(&taps[src][0], &taps[src][NTAPS]);
            d_filters[dst] = new gr::filter::kernel::fir_filter_ccf(1, t);
            if (d_filters[dst] == NULL)
                throw std::runtime_error("unable to create fir_filter_ccf");

            if (d_derivative) {
                t.assign(&Dtaps[src][0], &Dtaps[src][DNTAPS]);
                d_diff_filters[dst] =
                                   new gr::filter::kernel::fir_filter_ccf(1, t);
                if (d_diff_filters[dst] == NULL)
                    throw std::runtime_error("unable to create fir_filter_ccf");
            }
        }
    }

    interp_resampler_pfb_no_mf_cc::~interp_resampler_pfb_no_mf_cc()
    {
        for (int i = 0; i <= d_nfilters; i++) {
            delete d_filters[i];
            if (d_derivative)
                delete d_diff_filters[i];
        }
    }

    gr_complex
    interp_resampler_pfb_no_mf_cc::interpolate(const gr_complex input[],
                                               float mu) const
    {
        int arm = static_cast<int>(rint(mu * d_nfilters));

        if (arm < 0 or arm > d_nfilters)
            throw std::runtime_error("interp_resampler_pfb_no_mf_cc: mu is not "
                                     "in the range [0.0, 1.0]");

        return d_filters[arm]->filter(input);
    }

    gr_complex
    interp_resampler_pfb_no_mf_cc::differentiate(const gr_complex input[],
                                                 float mu) const
    {
        int arm = static_cast<int>(rint(mu * d_nfilters));

        if (arm < 0 or arm > d_nfilters)
            throw std::runtime_error("interp_resampler_pfb_no_mf_cc: mu is not "
                                     "in the range [0.0, 1.0]");

        return d_diff_filters[arm]->filter(input);
    }

    unsigned int
    interp_resampler_pfb_no_mf_cc::ntaps() const
    {
        return NTAPS;
    }

    /*************************************************************************/

    interp_resampler_pfb_no_mf_ff::interp_resampler_pfb_no_mf_ff(
                                                                bool derivative,
                                                                int nfilts)
    : interpolating_resampler_fff(interpolating_resampler::IR_PFB_NO_MF,
                                  derivative),
      d_nfilters(0),
      d_filters(),
      d_diff_filters()
    {
        if (nfilts <= 1)
            throw std::invalid_argument("interpolating_resampler_pfb_no_mf_ff: "
                                        "number of polyphase filter arms "
                                        "must be greater than 1");

        // Round up the number of filter arms to the current or next power of 2
        d_nfilters = 
               1 << (static_cast<int>(log2f(static_cast<float>(nfilts-1))) + 1);

        // N.B. We assume in this class: NSTEPS == DNSTEPS and NTAPS == DNTAPS

        // Limit to the maximum number of precomputed MMSE tap sets
        if (d_nfilters <= 0 or d_nfilters > NSTEPS)
            d_nfilters = NSTEPS;

        // Create our polyphase filter arms for the steps from 0.0 to 1.0 from
        // the MMSE interpolating filter and MMSE interpolating differentiator
        // taps rows.
        // N.B. We create an extra final row for an offset of 1.0, because it's
        // easier than dealing with wrap around from 0.99... to 0.0 shifted
        // by 1 tap.
        d_filters = 
         std::vector<gr::filter::kernel::fir_filter_fff*>(d_nfilters + 1, NULL);
        d_diff_filters = 
         std::vector<gr::filter::kernel::fir_filter_fff*>(d_nfilters + 1, NULL);

        std::vector<float> t(NTAPS, 0);
        int incr = NSTEPS/d_nfilters;
        int src, dst;
        for (src = 0, dst = 0; src <= NSTEPS; src += incr, dst++) {

            t.assign(&taps[src][0], &taps[src][NTAPS]);
            d_filters[dst] = new gr::filter::kernel::fir_filter_fff(1, t);
            if (d_filters[dst] == NULL)
                throw std::runtime_error("unable to create fir_filter_fff");

            if (d_derivative) {
                t.assign(&Dtaps[src][0], &Dtaps[src][DNTAPS]);
                d_diff_filters[dst] =
                                   new gr::filter::kernel::fir_filter_fff(1, t);
                if (d_diff_filters[dst] == NULL)
                    throw std::runtime_error("unable to create fir_filter_fff");
            }
        }
    }

    interp_resampler_pfb_no_mf_ff::~interp_resampler_pfb_no_mf_ff()
    {
        for (int i = 0; i <= d_nfilters; i++) {
            delete d_filters[i];
            if (d_derivative)
                delete d_diff_filters[i];
        }
    }

    float
    interp_resampler_pfb_no_mf_ff::interpolate(const float input[],
                                               float mu) const
    {
        int arm = static_cast<int>(rint(mu * d_nfilters));

        if (arm < 0 or arm > d_nfilters)
            throw std::runtime_error("interp_resampler_pfb_no_mf_ff: mu is not "
                                     "in the range [0.0, 1.0]");

        return d_filters[arm]->filter(input);
    }

    float
    interp_resampler_pfb_no_mf_ff::differentiate(const float input[],
                                                 float mu) const
    {
        int arm = static_cast<int>(rint(mu * d_nfilters));

        if (arm < 0 or arm > d_nfilters)
            throw std::runtime_error("interp_resampler_pfb_no_mf_ff: mu is not "
                                     "in the range [0.0, 1.0]");

        return d_diff_filters[arm]->filter(input);
    }

    unsigned int
    interp_resampler_pfb_no_mf_ff::ntaps() const
    {
        return NTAPS;
    }

    /*************************************************************************/

  } /* namespace nwr */
} /* namespace gr */
