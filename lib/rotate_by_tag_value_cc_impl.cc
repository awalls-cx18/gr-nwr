/* -*- c++ -*- */
/*
 * Copyright 2016 Free Software Foundation, Inc.
 * Copyright (C) 2016-2019  Andy Walls <awalls.cx18@gmail.com>
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

#include <rotate_by_tag_value_cc_impl.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>
#include <cmath>

namespace gr {
  namespace nwr {

    rotate_by_tag_value_cc::sptr
    rotate_by_tag_value_cc::make(const std::string& tag_name,
                                 float scale_factor,
                                 size_t vlen)
    {
      return gnuradio::get_initial_sptr
        (new rotate_by_tag_value_cc_impl(tag_name, scale_factor, vlen));
    }

    rotate_by_tag_value_cc_impl::rotate_by_tag_value_cc_impl(
                                                    const std::string& tag_name,
                                                    float scale_factor,
                                                    size_t vlen)
      : sync_block("rotate_by_tag_value_cc",
                   io_signature::make (1, 1, sizeof(gr_complex)*vlen),
                   io_signature::make (1, 1, sizeof(gr_complex)*vlen)),
        d_tag_key(pmt::intern(tag_name)),
        d_scale_factor(scale_factor),
	d_vlen(vlen),
        d_phase_inc(0.0),
        d_rotator()
    {
        const int alignment_multiple = volk_get_alignment()
                                       / sizeof(gr_complex);
        set_alignment(std::max(1, alignment_multiple));

        set_phase_angle(0.0);
        set_phase_inc(0.0);
    }

    rotate_by_tag_value_cc_impl::~rotate_by_tag_value_cc_impl()
    {
    }

    void rotate_by_tag_value_cc_impl::set_phase_angle(double phase_angle)
    {
        // We use a phase angle, but d_rotator wants a complex unit vector
        // in the direction of phase angle:
        // e^(0+ja) is a unit vector at angle a.
        d_rotator.set_phase(exp(gr_complex(0, phase_angle)));
    }

    void rotate_by_tag_value_cc_impl::set_phase_inc(double phase_inc)
    {
        // We use a phase increment, but d_rotator wants a complex
        // unit vector in the direction of the phase increment:
        // e^(0+ja) is a unit vector at angle a.
        d_rotator.set_phase_incr(exp(gr_complex(0, phase_inc)));
    }

    void rotate_by_tag_value_cc_impl::set_phase_inc_from_tag_value(
                                                          const pmt::pmt_t &val)
    {
        if (!pmt::is_number(val))
            return;
        d_phase_inc = pmt::to_double(val) * d_scale_factor;
        set_phase_inc(d_phase_inc);
    }

    float rotate_by_tag_value_cc_impl::phase_inc() const
    {
        return static_cast<float>(d_phase_inc);
    }

    int
    rotate_by_tag_value_cc_impl::work(int noutput_items,
                                      gr_vector_const_void_star &input_items,
                                      gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *) output_items[0];

        uint64_t offset = nitems_read(0);

        std::vector<tag_t> tags;
        get_tags_in_window(tags, 0, 0, noutput_items, d_tag_key);

        std::vector<tag_t>::iterator t;

        int start = 0, end;
        int count;
        // For each window before a target tag
        for (t = tags.begin(); t != tags.end(); ++t) {

            end = d_vlen * (t->offset - offset);
            count = end - start;

            // Rotate the input by the current phase increment setting
            if (count > 0) {
                if (d_phase_inc != 0.0)
                    d_rotator.rotateN(&out[start], &in[start], count);
                else
                    memcpy(&out[start], &in[start], count * sizeof(gr_complex));
            }

            // Point just past the data upon which we just operated
            start = end;

            // Extract new raw phase increment, scale it, and reconfigure the
            // rotator with the new phase increment.
            set_phase_inc_from_tag_value(t->value);
        }

        // For the final window after the last target tag
        end = d_vlen * noutput_items;
        count = end - start;

        // Rotate the input by the current phase increment setting
        if (count > 0) {
            if (d_phase_inc != 0.0)
                d_rotator.rotateN(&out[start], &in[start], count);
            else
                memcpy(&out[start], &in[start], count * sizeof(gr_complex));
        }

        return noutput_items;
    }

    void
    rotate_by_tag_value_cc_impl::setup_rpc()
    {
#ifdef GR_CTRLPORT
      add_rpc_variable(
        rpcbasic_sptr(new rpcbasic_register_get<rotate_by_tag_value_cc, float>(
	  alias(), "Constant",
	  &rotate_by_tag_value_cc::phase_inc,
	  pmt::from_double(-1024.0),
          pmt::from_double(1024.0),
          pmt::from_double(0.0),
	  "", "Current Phase Increment", RPC_PRIVLVL_MIN,
          DISPTIME | DISPOPTCPLX | DISPOPTSTRIP)));
#endif /* GR_CTRLPORT */
    }

  } /* namespace nwr */
} /* namespace gr */
