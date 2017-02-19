/* -*- c++ -*- */
/*
 * Copyright (C) 2017  Andy Walls <awalls.cx18@gmail.com>
 */

#ifndef INCLUDED_NWR_TIMING_ERROR_DETECTOR_H
#define INCLUDED_NWR_TIMING_ERROR_DETECTOR_H

#include <nwr/api.h>

#include <gnuradio/gr_complex.h>
#include <gnuradio/digital/constellation.h>
#include <deque>

namespace gr {
  namespace nwr {

    class timing_error_detector;

    class NWR_API timing_error_detector
    {
      public:

        // Timing Error Detector types.
        // A decision directed algorithm requires a constellation.
        enum ted_type {
            TED_NONE                         = -1,
            TED_MUELLER_AND_MULLER           = 0, // Decision directed
            TED_MOD_MUELLER_AND_MULLER       = 1, // Decision directed
            TED_ZERO_CROSSING                = 2, // Decision directed     
            TED_GARDNER                      = 4,
            TED_EARLY_LATE                   = 5,
            TED_DANDREA_AND_MENGALI_GEN_MSK  = 6,
        };

        static timing_error_detector *make(
                                    enum ted_type type,
                                    digital::constellation_sptr constellation =
                                                 digital::constellation_sptr());

        virtual ~timing_error_detector() {};

        virtual int inputs_per_symbol() { return d_inputs_per_symbol; }

        virtual void input(const gr_complex &x);
        virtual void input(float x);
        virtual bool needs_lookahead() { return d_needs_lookahead; }
        virtual void input_lookahead(const gr_complex &x);
        virtual void input_lookahead(float x);
        virtual float error() { return d_error; }

        virtual void revert(bool preserve_error = false);
        virtual void sync_reset();

      private:
        enum ted_type d_type;

      protected:
        timing_error_detector(enum ted_type type,
                              int inputs_per_symbol,
                              int error_computation_depth,
                              bool needs_lookahead = false,
                              digital::constellation_sptr constellation =
                                                 digital::constellation_sptr());

        void advance_input_clock() {
            d_input_clock = (d_input_clock + 1) % d_inputs_per_symbol;
        }
        void revert_input_clock()
        {
            if (d_input_clock == 0)
                d_input_clock = d_inputs_per_symbol - 1;
            else
                d_input_clock--;
        }
        void sync_reset_input_clock() {
            d_input_clock = d_inputs_per_symbol - 1;
        }

        gr_complex slice(const gr_complex &x);
        virtual float compute_error_cf() = 0;
        virtual float compute_error_ff() = 0;

        digital::constellation_sptr d_constellation;
        float d_error;
        float d_prev_error;
        int d_inputs_per_symbol;
        int d_input_clock;
        int d_error_depth;
        std::deque<gr_complex> d_input;
        std::deque<gr_complex> d_decision;
        bool d_needs_lookahead;
    };

    class NWR_API ted_mueller_and_muller : public timing_error_detector
    {
      public:
        ted_mueller_and_muller(digital::constellation_sptr constellation)
          : timing_error_detector(timing_error_detector::TED_MUELLER_AND_MULLER,
                                  1, 2, false, constellation)
        {}
        ~ted_mueller_and_muller() {};

      private:
        float compute_error_cf();
        float compute_error_ff();
    };

    class NWR_API ted_mod_mueller_and_muller : public timing_error_detector
    {
      public:
        ted_mod_mueller_and_muller(digital::constellation_sptr constellation)
          : timing_error_detector(
                              timing_error_detector::TED_MOD_MUELLER_AND_MULLER,
                              1, 3, false, constellation)
        {}
        ~ted_mod_mueller_and_muller() {};

      private:
        float compute_error_cf();
        float compute_error_ff();
    };

    class NWR_API ted_zero_crossing : public timing_error_detector
    {
      public:
        ted_zero_crossing(digital::constellation_sptr constellation)
          : timing_error_detector(timing_error_detector::TED_ZERO_CROSSING,
                                  2, 3, false, constellation)
        {}
        ~ted_zero_crossing() {};

      private:
        float compute_error_cf();
        float compute_error_ff();
    };

    class NWR_API ted_gardner : public timing_error_detector
    {
      public:
        ted_gardner()
          : timing_error_detector(timing_error_detector::TED_GARDNER,
                                  2, 3, false, digital::constellation_sptr())
        {}
        ~ted_gardner() {};

      private:
        float compute_error_cf();
        float compute_error_ff();
    };

    class NWR_API ted_early_late : public timing_error_detector
    {
      public:
        ted_early_late()
          : timing_error_detector(timing_error_detector::TED_EARLY_LATE,
                                  2, 2, true, digital::constellation_sptr())
        {}
        ~ted_early_late() {};

      private:
        float compute_error_cf();
        float compute_error_ff();
    };

    class NWR_API ted_generalized_msk : public timing_error_detector
    {
      public:
        ted_generalized_msk()
          : timing_error_detector(
                         timing_error_detector::TED_DANDREA_AND_MENGALI_GEN_MSK,
                         2, 4, false, digital::constellation_sptr())
        {}
        ~ted_generalized_msk() {};

      private:
        float compute_error_cf();
        float compute_error_ff();
    };

  } /* namespace nwr */
} /* namespace gr */

#endif /* INCLUDED_NWR_TIMING_ERROR_DETECTOR_H */
