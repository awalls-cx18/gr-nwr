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

#ifndef INCLUDED_ADD_TAG_VALUE_FF_IMPL_H
#define INCLUDED_ADD_TAG_VALUE_FF_IMPL_H

#include <nwr/add_tag_value_ff.h>

namespace gr {
  namespace nwr {

    class NWR_API add_tag_value_ff_impl
      : public add_tag_value_ff
    {
    private:
      size_t d_vlen;
      pmt::pmt_t d_tag_key;
      float d_k;

      void notvolk_32f_s32f_add_32f(float* cVector,
                                    const float* aVector,
                                    const float scalar,
                                    unsigned int num_points);

    public:
      add_tag_value_ff_impl(const std::string& tag_name, size_t vlen);
      ~add_tag_value_ff_impl();

      float k() const;

      void setup_rpc();

      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } /* namespace nwr */
} /* namespace gr */

#endif /* INCLUDED_ADD_TAG_VALUE_FF_IMPL_H */
