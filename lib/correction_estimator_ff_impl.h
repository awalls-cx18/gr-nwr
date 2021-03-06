/* -*- c++ -*- */
/*
 * Copyright (C) 2016  Andy Walls <awalls.cx18@gmail.com>
 *
 * This file was automatically generated by gr_modtool from GNU Radio
 *
 * This file was automatically generated from a template incorporating
 * data input by Andy Walls and subsequently hand edited by Andy Walls.
 * See http://www.gnu.org/licenses/gpl-faq.en.html#GPLOutput .
 */

#ifndef INCLUDED_NWR_CORRECTION_ESTIMATOR_FF_IMPL_H
#define INCLUDED_NWR_CORRECTION_ESTIMATOR_FF_IMPL_H

#include <nwr/correction_estimator_ff.h>
#include <pmt/pmt.h>

namespace gr {
  namespace nwr {

    class correction_estimator_ff_impl : public correction_estimator_ff
    {
     private:
      int d_inspection_len;
      int d_inspection_offset;
      float d_peak_ref;
      float d_trough_ref;
      int d_timing_win_start;
      int d_timing_win_end;
      pmt::pmt_t d_offset_corr_key;
      pmt::pmt_t d_scale_corr_key;
      pmt::pmt_t d_time_est_key;
      pmt::pmt_t d_clock_est_key;
      pmt::pmt_t d_sob_key;
      pmt::pmt_t d_eob_key;
      pmt::pmt_t d_eob_offset_corr;
      pmt::pmt_t d_eob_scale_corr;
      pmt::pmt_t d_src_id;

      std::vector<tag_t> d_tags;

      void compute_corrections(const float *in,
                               double &offset_corr, double &scale_corr);

      bool compute_timing_estimate(const float *in,
                                   uint64_t &n, double &fraction,
                                   double &clock_period);

     public:
      correction_estimator_ff_impl(int inspection_length,
                                   int inspection_offset,
                                   float peak_ref,
                                   float trough_ref,
                                   const std::string &offset_corr_key,
                                   const std::string &scale_corr_key,
                                   bool scale_eob_zero,
                                   int timing_win_start,
                                   int timing_win_end,
                                   const std::string &time_est_key,
                                   const std::string &clock_est_key,
                                   const std::string &sob_key,
                                   const std::string &eob_key);
      ~correction_estimator_ff_impl();

      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace nwr
} // namespace gr

#endif /* INCLUDED_NWR_CORRECTION_ESTIMATOR_FF_IMPL_H */

