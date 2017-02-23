#!/bin/sh
#
# Copyright 2017 Andy Walls <awalls.cx18@gmail.com>
#
# This file is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this file; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

gcc      -c -o diff_objective_fct.o             diff_objective_fct.c
gfortran -c -o praxis.o                         praxis.f
gcc      -c -o gen_interp_differentiator_taps.o gen_interp_differentiator_taps.c

gfortran -o gen_interp_differentiator_taps \
        gen_interp_differentiator_taps.o praxis.o diff_objective_fct.o \
        -lgsl -lgslcblas

./gen_interp_differentiator_taps -n 128 -t 8 -B 0.333333333333 \
        > interp_differentiator_taps.h
