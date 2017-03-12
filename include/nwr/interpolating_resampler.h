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

#ifndef INCLUDED_NWR_INTERPOLATING_RESAMPLER_H
#define INCLUDED_NWR_INTERPOLATING_RESAMPLER_H

#include <nwr/api.h>

#include <gnuradio/gr_complex.h>
#include <vector>
#include <gnuradio/filter/mmse_fir_interpolator_cc.h>
#include <gnuradio/filter/mmse_fir_interpolator_ff.h>
#include <nwr/mmse_interp_differentiator_cc.h>
#include <nwr/mmse_interp_differentiator_ff.h>

namespace gr {
  namespace nwr {

    class NWR_API interpolating_resampler
    {
      public:

        // Interpolating Resampler type
        enum ir_type {
            IR_NONE      = -1,
            IR_MMSE_8TAP = 0,  // Valid for [-Fs/4, Fs/4] bandlimited input
            IR_PFB_NO_MF = 1,  // No matched filtering, just interpolation
            IR_PFB_MF    = 2,
        };

        virtual ~interpolating_resampler() {}

        virtual unsigned int ntaps() const = 0;

        virtual float phase()         { return d_phase; }
        virtual int   phase_n()       { return d_phase_n; }
        virtual float phase_wrapped() { return d_phase_wrapped; }

        virtual void next_phase(float increment,
                                float &phase,
                                int &phase_n,
                                float &phase_wrapped);
        virtual void advance_phase(float increment);
        virtual void revert_phase();
        virtual void sync_reset(float phase);

      private:
        enum ir_type d_type;

      protected:
        interpolating_resampler(enum ir_type type, bool derivative = false);

        bool d_derivative;

        float d_phase;
        float d_phase_wrapped;
        int   d_phase_n;
        float d_prev_phase;
        float d_prev_phase_wrapped;
        int   d_prev_phase_n;
    };

    /*************************************************************************/

    class NWR_API interpolating_resampler_ccf : public interpolating_resampler
    {
      public:
        static interpolating_resampler_ccf *make(enum ir_type type,
                                                 bool derivative = false,
                                                 int nfilts = 32,
                                                 const std::vector<float> &taps
                                                        = std::vector<float>());

        virtual ~interpolating_resampler_ccf() {};

        virtual gr_complex interpolate(const gr_complex input[],
                                       float mu) const = 0;
        virtual gr_complex differentiate(const gr_complex input[],
                                         float mu) const = 0;

      protected:
        interpolating_resampler_ccf(enum ir_type type,
                                    bool derivative = false)
        : interpolating_resampler(type, derivative) {}
    };

    /*************************************************************************/

    class NWR_API interpolating_resampler_fff : public interpolating_resampler
    {
      public:
        static interpolating_resampler_fff *make(enum ir_type type,
                                                 bool derivative = false,
                                                 int nfilts = 32,
                                                 const std::vector<float> &taps
                                                        = std::vector<float>());

        virtual ~interpolating_resampler_fff() {};

        virtual float interpolate(const float input[], float mu) const = 0;
        virtual float differentiate(const float input[], float mu) const = 0;

      protected:
        interpolating_resampler_fff(enum ir_type type,
                                    bool derivative = false)
        : interpolating_resampler(type, derivative) {}
    };

    /*************************************************************************/

    class NWR_API interp_resampler_mmse_8tap_cc
                  : public interpolating_resampler_ccf
    {
      public:
        interp_resampler_mmse_8tap_cc(bool derivative = false);
        ~interp_resampler_mmse_8tap_cc();

        unsigned int ntaps() const;
        gr_complex interpolate(const gr_complex input[], float mu) const;
        gr_complex differentiate(const gr_complex input[], float mu) const;

      private:
        gr::filter::mmse_fir_interpolator_cc *d_interp;
        mmse_interp_differentiator_cc *d_interp_diff;
    };

    class NWR_API interp_resampler_mmse_8tap_ff
                  : public interpolating_resampler_fff
    {
      public:
        interp_resampler_mmse_8tap_ff(bool derivative = false);
        ~interp_resampler_mmse_8tap_ff();

        unsigned int ntaps() const;
        float interpolate(const float input[], float mu) const;
        float differentiate(const float input[], float mu) const;

      private:
        gr::filter::mmse_fir_interpolator_ff *d_interp;
        mmse_interp_differentiator_ff *d_interp_diff;
    };

    /*************************************************************************/

  } /* namespace nwr */
} /* namespace gr */

#endif /* INCLUDED_NWR_INTERPOLATING_RESAMPLER_H */
