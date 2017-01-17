/* -*- c++ -*- */
/*
 * Copyright (C) 2017  Andy Walls <awalls.cx18@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <nwr/timing_error_detector.h>
#include <stdexcept>

namespace gr {
  namespace nwr {

    timing_error_detector *
    timing_error_detector::make(enum ted_type type,
                                digital::constellation_sptr constellation)
    {
        timing_error_detector *ret = NULL;
        switch (type) {
        case TED_NONE:
            break;
        case TED_MUELLER_AND_MULLER:
            ret = new ted_mueller_and_muller(constellation);
            break;
        case TED_MOD_MUELLER_AND_MULLER:
            break;
        case TED_ZERO_CROSSING:
            break;
        case TED_GARDNER:
            break;
        case TED_EARLY_LATE:
            break;
        case TED_DANDREA_AND_MENGALLI_GEN_MSK:
            break;
        default: 
            break;
        }
        return ret;
    }

    timing_error_detector::timing_error_detector(
                                      enum ted_type type,
                                      digital::constellation_sptr constellation)
      : d_type(type),
        d_constellation(constellation),
        d_error(0.0f),
        d_prev_error(0.0f),
        d_inputs_per_symbol(1) 
    {
        sync_reset_input_clock();

        if (d_constellation && d_constellation->dimensionality() != 1)
            throw std::invalid_argument(
                  "timing_error_detector: constellation dimensionality "
                  "(complex numbers per symbol) must be 1.");
        
        switch (type) {
        case TED_MUELLER_AND_MULLER:
        case TED_MOD_MUELLER_AND_MULLER:
        case TED_ZERO_CROSSING:
            if (!d_constellation)
                throw std::invalid_argument(
                   "timing_error_detector: slicer constellation required.");
            break;
        case TED_GARDNER:
        case TED_EARLY_LATE:
        case TED_DANDREA_AND_MENGALLI_GEN_MSK:
            break;
        case TED_NONE:
        default: 
            throw std::invalid_argument(
                  "timing_error_detector: invalid timing error detector type.");
            break;
        }
    }

    void
    timing_error_detector::revert_input_clock()
    {
        if (d_input_clock == 0)
            d_input_clock = d_inputs_per_symbol - 1;
        else
            d_input_clock--;
    }

    gr_complex
    timing_error_detector::slice(const gr_complex &x)
    {
        if (!d_constellation)
            return x;

        unsigned int index;
        gr_complex z(0.0f, 0.0f);

        index = d_constellation->decision_maker(&x);
        d_constellation->map_to_points(index, &z);
        return z;
    }

    /*************************************************************************/

    ted_mueller_and_muller::ted_mueller_and_muller(
                                      digital::constellation_sptr constellation)
      : timing_error_detector(timing_error_detector::TED_MUELLER_AND_MULLER,
                              constellation),
        d_prev_input(0.0f, 0.0f),
        d_prev2_input(0.0f, 0.0f)
    {
        d_prev_decision = slice(d_prev_input);
        d_prev2_decision = slice(d_prev2_input);

        d_inputs_per_symbol = 1;
        sync_reset_input_clock();
    }

    void
    ted_mueller_and_muller::input(const gr_complex &x)
    {
        //advance_input_clock();
        //if (d_input_clock == 0)
        //    compute error

        gr_complex curr_decision = slice(x);

        d_prev_error = d_error;

        d_error =   (d_prev_decision.real() * x.real()
                     - curr_decision.real() * d_prev_input.real())
                  + (d_prev_decision.imag() * x.imag()
                     - curr_decision.imag() * d_prev_input.imag());

        d_prev2_input = d_prev_input;
        d_prev_input = x;

        d_prev2_decision = d_prev_decision;
        d_prev_decision = curr_decision;
    }

    void
    ted_mueller_and_muller::input(float x)
    {
        //advance_input_clock();
        //if (d_input_clock == 0)
        //    compute error

        gr_complex curr_input(x, 0.0f);
        gr_complex curr_decision = slice(curr_input);

        d_prev_error = d_error;

        d_error =   d_prev_decision.real() * curr_input.real() 
                  - curr_decision.real() * d_prev_input.real();

        d_prev2_input = d_prev_input;
        d_prev_input = curr_input;

        d_prev2_decision = d_prev_decision;
        d_prev_decision = curr_decision;
    }

    void
    ted_mueller_and_muller::revert()
    {
        //revert_input_clock();
        //if reverting an error computation ...
        //   revert error
        d_error = d_prev_error;
        d_prev_input = d_prev2_input;
        d_prev_decision = d_prev2_decision;
    }

    void
    ted_mueller_and_muller::sync_reset()
    {
        gr_complex czero(0.0f, 0.0f);

        d_error = 0.0f;
        d_prev_error = 0.0f;
        d_prev_input = czero;
        d_prev2_input = czero;
        d_prev_decision = slice(d_prev_input);
        d_prev2_decision = slice(d_prev2_input);

        sync_reset_input_clock();
    }

  } /* namespace nwr */
} /* namespace gr */
