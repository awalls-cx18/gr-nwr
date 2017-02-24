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

#include <nwr/mmse_interp_differentiator_cc.h>
#include <nwr/interp_differentiator_taps.h>
#include <stdexcept>

namespace gr {
  namespace nwr {

    mmse_interp_differentiator_cc::mmse_interp_differentiator_cc()
    {
      filters.resize (NSTEPS + 1);

      for(int i = 0; i < NSTEPS + 1; i++) {
	std::vector<float> t (&taps[i][0], &taps[i][NTAPS]);
	filters[i] = new gr::filter::kernel::fir_filter_ccf(1, t);
      }
    }

    mmse_interp_differentiator_cc::~mmse_interp_differentiator_cc()
    {
      for(int i = 0; i < NSTEPS + 1; i++)
	delete filters[i];
    }

    unsigned
    mmse_interp_differentiator_cc::ntaps() const
    {
      return NTAPS;
    }

    unsigned
    mmse_interp_differentiator_cc::nsteps() const
    {
      return NSTEPS;
    }

    gr_complex
    mmse_interp_differentiator_cc::differentiate(const gr_complex input[],
					         float mu) const
    {
      int imu = (int)rint(mu * NSTEPS);

      if((imu < 0) || (imu > NSTEPS)) {
	throw std::runtime_error(
                         "mmse_interp_differentiator_cc: imu out of bounds.\n");
      }

      gr_complex r = filters[imu]->filter(input);
      return r;
    }

  }  /* namespace nwr */
}  /* namespace gr */
