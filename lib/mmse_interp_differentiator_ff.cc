/* -*- c++ -*- */
/*
 * Copyright 2002,2012 Free Software Foundation, Inc.
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
#include <config.h>
#endif

#include <nwr/mmse_interp_differentiator_ff.h>
#include <nwr/interp_differentiator_taps.h>
#include <stdexcept>

namespace gr {
  namespace nwr {

    mmse_interp_differentiator_ff::mmse_interp_differentiator_ff()
    {
      filters.resize (DNSTEPS + 1);

      for(int i = 0; i < DNSTEPS + 1; i++) {
	std::vector<float> t (&Dtaps[i][0], &Dtaps[i][DNTAPS]);
	filters[i] = new gr::filter::kernel::fir_filter_fff(1, t);
      }
    }

    mmse_interp_differentiator_ff::~mmse_interp_differentiator_ff()
    {
      for(int i = 0; i < DNSTEPS + 1; i++)
	delete filters[i];
    }

    unsigned
    mmse_interp_differentiator_ff::ntaps() const
    {
      return DNTAPS;
    }

    unsigned
    mmse_interp_differentiator_ff::nsteps() const
    {
      return DNSTEPS;
    }

    float
    mmse_interp_differentiator_ff::differentiate(const float input[],
					         float mu) const
    {
      int imu = (int)rint(mu * DNSTEPS);

      if((imu < 0) || (imu > DNSTEPS)) {
	throw std::runtime_error(
                         "mmse_interp_differentiator_ff: imu out of bounds.\n");
      }

      float r = filters[imu]->filter(input);
      return r;
    }

  }  /* namespace nwr */
}  /* namespace gr */
