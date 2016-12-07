/* -*- c++ -*- */
/*
 * Copyright 2016 Free Software Foundation, Inc.
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

#include <multiply_by_tag_value_ff_impl.h>
#include <gnuradio/io_signature.h>
#include <volk/volk.h>

namespace gr {
  namespace nwr {

    multiply_by_tag_value_ff::sptr
    multiply_by_tag_value_ff::make(const std::string& tag_name,
                                   size_t vlen)
    {
      return gnuradio::get_initial_sptr
        (new multiply_by_tag_value_ff_impl(tag_name, vlen));
    }

    multiply_by_tag_value_ff_impl::multiply_by_tag_value_ff_impl(const std::string& tag_name,
                                                                 size_t vlen)
      : sync_block("multiply_by_tag_value_ff",
                   io_signature::make (1, 1, sizeof (float)*vlen),
                   io_signature::make (1, 1, sizeof (float)*vlen)),
	d_vlen(vlen), d_k(1.0f)
    {
      d_tag_key = pmt::intern(tag_name);

      const int alignment_multiple =
	volk_get_alignment() / sizeof(float);
      set_alignment(std::max(1,alignment_multiple));
    }

    multiply_by_tag_value_ff_impl::~multiply_by_tag_value_ff_impl()
    {
    }

    float
    multiply_by_tag_value_ff_impl::k() const
    {
      return d_k;
    }

    int
    multiply_by_tag_value_ff_impl::work(int noutput_items,
                                        gr_vector_const_void_star &input_items,
                                        gr_vector_void_star &output_items)
    {
      const float *in = (const float *) input_items[0];
      float *out = (float *) output_items[0];

      std::vector<tag_t> tags;
      get_tags_in_window(tags, 0, 0, noutput_items, d_tag_key);

      std::vector<tag_t>::iterator itag = tags.begin();

      int start = 0, end;
      while(itag != tags.end()) {
        end = itag->offset - nitems_read(0);
        end *= d_vlen;

        // Multiply based on the current value of k from 'start' to 'end'
        volk_32f_s32f_multiply_32f(&out[start], &in[start], d_k, (end-start));
        start = end;

        // Extract new value of k
        pmt::pmt_t k = itag->value;
        if(pmt::is_number(k)) {
          d_k = static_cast<float>(pmt::to_double(k));
        }
        else {
          GR_LOG_WARN(d_logger,
                      boost::format("Got key '%1%' with incompatible value of '%2%'") \
                      % pmt::write_string(d_tag_key) % pmt::write_string(k));
        }

        itag++;
      }

      volk_32f_s32f_multiply_32f(&out[start], &in[start], d_k,
                                 (d_vlen*noutput_items-start));

      return noutput_items;
    }

    void
    multiply_by_tag_value_ff_impl::setup_rpc()
    {
#ifdef GR_CTRLPORT
      add_rpc_variable(
        rpcbasic_sptr(new rpcbasic_register_get<multiply_by_tag_value_ff, float>(
	  alias(), "Constant",
	  &multiply_by_tag_value_ff::k,
	  pmt::from_double(-1024.0),
          pmt::from_double(1024.0),
          pmt::from_double(0.0),
	  "", "Constant to multiply", RPC_PRIVLVL_MIN,
          DISPTIME | DISPOPTCPLX | DISPOPTSTRIP)));
#endif /* GR_CTRLPORT */
    }

  } /* namespace nwr */
} /* namespace gr */
