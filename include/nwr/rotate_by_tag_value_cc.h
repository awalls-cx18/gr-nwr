/* -*- c++ -*- */
/*
 * Copyright 2016 Free Software Foundation, Inc.
 * Copyright (C) 2016-2019  Andy Walls <awalls.cx18@gmail.com>
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_ROTATE_BY_TAG_VALUE_CC_H
#define INCLUDED_ROTATE_BY_TAG_VALUE_CC_H

#include <nwr/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace nwr {

    /*!
     * \brief output = input * e^{j * value * scale_factor * n}
     * \ingroup nwr
     *
     * \details
     * The value used by this block is found from a tag
     * with the name specified by \p tag_name. The tag must contain a
     * float or double PMT value that will be converted into a
     * float value. This value will be multiplied by the specified
     * \p scale_factor to generate the phase increment used by the
     * frequency spectrum rotator.  The overall phase increment should
     * be in the interval [-pi, pi].  All input data is rotated by this
     * phase increment until a new tag with an updated value is found.
     * The block starts with a value of '0.0' for the value constant.
     */
    class NWR_API rotate_by_tag_value_cc : virtual public sync_block
    {
    public:
      // gr::nwr::rotate_by_tag_value_cc::sptr
      typedef boost::shared_ptr<rotate_by_tag_value_cc> sptr;

      /*!
       * \brief Create an instance of rotate_by_tag_value_cc
       * \param tag_name Tag's key that it will use to get the
       *                 raw phase increment constant.
       * \param scale_factor A constant applied to the raw phase increment
       *                     to compute the applied phase increment constant.
       * \param vlen Vector length of incoming stream
       */
      static sptr make(const std::string& tag_name,
                       float scale_factor,
                       size_t vlen=1);

      /*!
       * Get the current phase increment, in normalized radians,
       * used by the rotator.  This block does not allow external setters.
       */
      virtual float phase_inc() const = 0;
    };

  } /* namespace nwr */
} /* namespace gr */

#endif /* INCLUDED_ROTATE_BY_TAG_VALUE_CC_H */
