#!/bin/sh
#
# Copyright 2017 Andy Walls <awalls.cx18@gmail.com>
#

gcc      -c -o diff_objective_fct.o             diff_objective_fct.c
gfortran -c -o praxis.o                         praxis.f
gcc      -c -o gen_interp_differentiator_taps.o gen_interp_differentiator_taps.c

gfortran -o gen_interp_differentiator_taps \
        gen_interp_differentiator_taps.o praxis.o diff_objective_fct.o \
        -lgsl -lgslcblas

./gen_interp_differentiator_taps -n 128 -t 8 -B 0.333333333333 \
        > interp_differentiator_taps.h
