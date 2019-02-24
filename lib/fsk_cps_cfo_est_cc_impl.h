/* -*- c++ -*- */
/* 
 * Copyright 2019 Andy Walls <awalls.cx18@gmail.com>.
 *
 * This file was automatically generated by gr_modtool from GNU Radio
 *
 * This file was automatically generated from a template incorporating
 * data input by Andy Walls and later hand edited by Andy Walls.
 * See http://www.gnu.org/licenses/gpl-faq.en.html#GPLOutput .
 */

#ifndef INCLUDED_NWR_FSK_CPS_CFO_EST_CC_IMPL_H
#define INCLUDED_NWR_FSK_CPS_CFO_EST_CC_IMPL_H

#include <nwr/fsk_cps_cfo_est_cc.h>
#include <pmt/pmt.h>
#include <gnuradio/fft/fft.h>

namespace gr {
  namespace nwr {

    class fsk_cps_cfo_est_cc_impl : public fsk_cps_cfo_est_cc
    {
     private:
        float d_samp_rate;
        float d_mod_index;
        float d_baud_rate;
        float d_tolerance;

        int                       d_fft_size;
        gr::fft::window::win_type d_window;
        float                     d_beta;

        pmt::pmt_t d_output_tag_key;
        pmt::pmt_t d_sob_tag_key;

        bool  d_periodic;
        float d_interval;

        float                *d_fft_window;
        gr_complex           *d_x2;
        gr::fft::fft_complex *d_fft0;
        gr::fft::fft_complex *d_fft1;
        gr_complex           *d_cps;
        float                *d_mag2;
        int                   d_n_bins;
        std::vector<uint32_t> d_bin_mag_order;
        float                 d_target_bin_delta;
        float                 d_max_bin_error;
        int                   d_max_bin_err_int;

        std::vector<tag_t> d_tags;
        pmt::pmt_t d_src_id;

        uint64_t d_out_interval;
        uint64_t d_out_next_off;

        double compute_estimate(const gr_complex *in, bool &valid);

     public:
      fsk_cps_cfo_est_cc_impl(float samp_rate,
                              float mod_index,
                              float baud_rate,
                              float tolerance,
                              int fft_size,
                              gr::fft::window::win_type window,
                              float beta,
                              const std::string &output_tag,
                              const std::string &sob_tag,
                              bool periodic,
                              float interval);
      ~fsk_cps_cfo_est_cc_impl();

      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace nwr
} // namespace gr

#endif /* INCLUDED_NWR_FSK_CPS_CFO_EST_CC_IMPL_H */
