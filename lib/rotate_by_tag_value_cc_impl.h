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

#ifndef INCLUDED_ROTATE_BY_TAG_VALUE_CC_IMPL_H
#define INCLUDED_ROTATE_BY_TAG_VALUE_CC_IMPL_H

#include <nwr/rotate_by_tag_value_cc.h>
#include <gnuradio/blocks/rotator.h>

namespace gr {
  namespace nwr {

    class NWR_API rotate_by_tag_value_cc_impl
      : public rotate_by_tag_value_cc
    {
    private:
      size_t d_vlen;
      pmt::pmt_t d_tag_key;
      float d_scale_factor;
      double d_phase_inc;

      gr::blocks::rotator d_rotator;

      void set_phase_angle(double phase_angle);
      void set_phase_inc(double phase_inc);

      void set_phase_inc_from_tag_value(const pmt::pmt_t &val);


    public:
      rotate_by_tag_value_cc_impl(const std::string& tag_name,
                                  float scale_factor,
                                  size_t vlen);
      ~rotate_by_tag_value_cc_impl();

      float phase_inc() const;

      void setup_rpc();

      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } /* namespace nwr */
} /* namespace gr */

#endif /* INCLUDED_ROTATE_BY_TAG_VALUE_CC_IMPL_H */
