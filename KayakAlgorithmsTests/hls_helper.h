/*
 * =====================================================================================
 *
 *       Filename:  hls_helper.h
 *
 *    Description:  hls help functions.
 *
 *        Version:  1.0
 *        Created:  8/17/2021 4:36:56 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Alexander Hsu (), chenyu.xu@surgnova.com
 *   Organization:  Surgnova
 *
 * =====================================================================================
 */

#ifndef __H_HLS_HELPER
#define __H_HLS_HELPER


namespace hls_helper {


    /**
     *  @brief Convert hls ap_fixed(W, I) to C style double
     *  @param unsigned int v, the 32 bits binary of ap_fixed(W, I)
     *  @param unsigned int W, the whole width of the ap_fixed number.
     *                         it should be >= 32
     *  @param unsigned int I, the whole width of the ap_fixed number.
     *                         it should be > W, and (W - I) <= 23
     *  @return the double of ap_fixed<W, I>
     */
double hls_apfixed_to_double(unsigned int v, unsigned int W, unsigned int I);

    /**
     *  @brief Convert the C style double to the hls ap_fixed<W, I>
     *  @param double v, the double value of ap_fixed(W, I)
     *  @param unsigned int W, the whole width of the ap_fixed number.
     *                         it should be >= 32
     *  @param unsigned int I, the whole width of the ap_fixed number.
     *                         it should be > W, and (W - I) <= 23
     *  @return the binary representation inside 32 bit integer
     *           of ap_fixed<W, I>
     */
unsigned int double_to_hls_apfixed(double v, unsigned int W, unsigned I);
} //hls_helper

#endif
