/* -*- c++ -*- */
/*
 * Copyright (C) 2017  Andy Walls <awalls.cx18@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <nwr/timing_error_detector.h>
#include <gnuradio/math.h>
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
            ret = new ted_mod_mueller_and_muller(constellation);
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
        d_input(2, gr_complex(0.0f, 0.0f)),
        d_decision()
    {
        std::deque<gr_complex>::iterator it;
        for (it = d_input.begin(); it != d_input.end(); ++it)
            d_decision.push_back(slice(*it));

        d_inputs_per_symbol = 1;
        sync_reset_input_clock();
    }

    void
    ted_mueller_and_muller::input(const gr_complex &x)
    {
        //advance_input_clock();
        //if (d_input_clock == 0)
        //    compute error

        d_input.push_front(x);
        d_decision.push_front(slice(d_input[0]));

        d_prev_error = d_error;

        d_error =   (  d_decision[1].real() * d_input[0].real()
                     - d_decision[0].real() * d_input[1].real())
                  + (  d_decision[1].imag() * d_input[0].imag()
                     - d_decision[0].imag() * d_input[1].imag());

        d_input.pop_back();
        d_decision.pop_back();
    }

    void
    ted_mueller_and_muller::input(float x)
    {
        //advance_input_clock();
        //if (d_input_clock == 0)
        //    compute error

        d_input.push_front(gr_complex(x, 0.0f));
        d_decision.push_front(slice(d_input[0]));

        d_prev_error = d_error;

        d_error =   (  d_decision[1].real() * d_input[0].real()
                     - d_decision[0].real() * d_input[1].real());

        d_input.pop_back();
        d_decision.pop_back();
    }

    void
    ted_mueller_and_muller::revert()
    {
        //revert_input_clock();
        //if reverting an error computation ...
        //   revert error
        d_error = d_prev_error;
        d_input.push_back(d_input.back());
        d_input.pop_front();
        d_decision.push_back(d_decision.back());
        d_decision.pop_front();
    }

    void
    ted_mueller_and_muller::sync_reset()
    {
        d_error = 0.0f;
        d_prev_error = 0.0f;

        d_input.assign(2, gr_complex(0.0f, 0.0f));

        std::deque<gr_complex>::iterator it;
        d_decision.clear();
        for (it = d_input.begin(); it != d_input.end(); ++it)
            d_decision.push_back(slice(*it));

        sync_reset_input_clock();
    }

    /*************************************************************************/

    ted_mod_mueller_and_muller::ted_mod_mueller_and_muller(
                                      digital::constellation_sptr constellation)
      : timing_error_detector(timing_error_detector::TED_MOD_MUELLER_AND_MULLER,
                              constellation),
        d_input(3, gr_complex(0.0f, 0.0f)),
        d_decision()
    {
        std::deque<gr_complex>::iterator it;
        for (it = d_input.begin(); it != d_input.end(); ++it)
            d_decision.push_back(slice(*it));

        d_inputs_per_symbol = 1;
        sync_reset_input_clock();
    }

    void
    ted_mod_mueller_and_muller::input(const gr_complex &x)
    {
        //advance_input_clock();
        //if (d_input_clock == 0)
        //    compute error

        gr_complex u;

        d_input.push_front(x);
        d_decision.push_front(slice(d_input[0]));

        d_prev_error = d_error;

        u = ((d_input[0]    - d_input[2]   ) * conj(d_decision[1]))
           -((d_decision[0] - d_decision[2]) * conj(d_input[1]   ));

        d_error = u.real();
        d_error = gr::branchless_clip(d_error, 1.0f);

        d_input.pop_back();
        d_decision.pop_back();
    }

    void
    ted_mod_mueller_and_muller::input(float x)
    {
        //advance_input_clock();
        //if (d_input_clock == 0)
        //    compute error

        float u;

        d_input.push_front(gr_complex(x, 0.0f));
        d_decision.push_front(slice(d_input[0]));

        d_prev_error = d_error;

        u = ((d_input[0].real() - d_input[2].real()) * d_decision[1].real())
           -((d_decision[0].real() - d_decision[2].real()) * d_input[1].real());

        d_error = u/2.0f;
        d_error = gr::branchless_clip(d_error, 1.0f);

        d_input.pop_back();
        d_decision.pop_back();
    }

    void
    ted_mod_mueller_and_muller::revert()
    {
        //revert_input_clock();
        //if reverting an error computation ...
        //   revert error
        d_error = d_prev_error;
        d_input.push_back(d_input.back());
        d_input.pop_front();
        d_decision.push_back(d_decision.back());
        d_decision.pop_front();
    }

    void
    ted_mod_mueller_and_muller::sync_reset()
    {
        d_error = 0.0f;
        d_prev_error = 0.0f;

        d_input.assign(3, gr_complex(0.0f, 0.0f));

        std::deque<gr_complex>::iterator it;
        d_decision.clear();
        for (it = d_input.begin(); it != d_input.end(); ++it)
            d_decision.push_back(slice(*it));

        sync_reset_input_clock();
    }
  } /* namespace nwr */
} /* namespace gr */
