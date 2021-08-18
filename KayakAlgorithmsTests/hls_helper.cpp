/*
 * =====================================================================================
 *
 *       Filename:  hls_helper.cpp
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

#include "hls_helper.h"

#include <iostream>
#include <bitset>
using namespace std;

#include <string.h>
#include <math.h>
#include <stdint.h>

namespace hls_helper {

// Assuming that w won't larger then 32
double hls_apfixed_to_double(unsigned int v, unsigned int W, unsigned int I) {
    if (W > 32 || W == 0)
        return 0.0;

    unsigned int B = W - I;
    // Solve the integer part firstly
    int integer_part = (int)(v >> B);
    //cout << "Original:" << bitset<32>(integer_part) << endl;
    bool isNegative = (integer_part & (1 << (I - 1)));
    double value = 0.0;
    int i;
    if (isNegative) {
        for (i = I; i < 32; i++) {
            integer_part = integer_part | (1 << i);
        }
        //cout << "Signed:" << bitset<32>(integer_part) << endl;
    }
    value += integer_part;
    //cout << "value=" << value << endl;
    unsigned int count = 0;
    for (i = B - 1; i >= 0; i--) {
        bool current_bit = v & (1 << i);
        //cout << "The " << i << " bit=" << current_bit << endl;
        value += ((current_bit)? 1 : 0)*pow(0.5, static_cast<double>(count + 1));
        count++;
    }

    return value;
}

unsigned int double_to_hls_apfixed(double v, unsigned int W, unsigned I) {
    unsigned int B = W - I;
    bool isNegative = false;
    if (v < 0)
        isNegative = true;

    unsigned int ap_fixed_binary = 0;
    double abs_value = abs(v);
    int integer_part = (int)abs_value;
    ap_fixed_binary = integer_part << B;
    //cout << "ap_fixed_binary int=" << bitset<32>(ap_fixed_binary) << endl;
    float fraction_part = (float)(abs_value - integer_part);

    // I will steal the float(single precision) binary representaion
    // as my result. Below is the bits layout for float coding.
    // |s(1 bit)| E(8 bits)--------|M(23 bits)---------------|
    // The value of the float is (-1)^s*(1.M)*2^(E-127).
    // I only want the binary part of M, and ignore others,so I have to
    // add 1 to fractional part.
    fraction_part += 1;
    //cout << "integer_part=" << integer_part << endl;
    cout << "fraction_part =" << fraction_part - 1 << endl;

    uint32_t bin_float_fraction_part;
    memcpy(&bin_float_fraction_part, &fraction_part, sizeof(float));
    cout << "fraction_part binary=" << bitset<32>(bin_float_fraction_part) << endl;

    unsigned int fraction_binary = bin_float_fraction_part << 9;
    fraction_binary = fraction_binary >> (9 + (23 - B));
    //cout << "fraction_binary=" << bitset<32>(fraction_binary) << endl;

    ap_fixed_binary = ap_fixed_binary | fraction_binary;
    //cout << "ap_fixed_binary all=" << bitset<32>(ap_fixed_binary) << endl;

    if (isNegative) {
        ap_fixed_binary = ~ap_fixed_binary;
        for (int i = W; i < 32; i++)
            ap_fixed_binary = (ap_fixed_binary & ~(1 << i));
        //cout << "ap_fixed_binary negative=" << bitset<32>(ap_fixed_binary) << endl;
    }
    return ap_fixed_binary;
}

}
