// ----------------------------------------------------------------- //
//             The Speech Signal Processing Toolkit (SPTK)           //
//             developed by SPTK Working Group                       //
//             http://sp-tk.sourceforge.net/                         //
// ----------------------------------------------------------------- //
//                                                                   //
//  Copyright (c) 1984-2007  Tokyo Institute of Technology           //
//                           Interdisciplinary Graduate School of    //
//                           Science and Engineering                 //
//                                                                   //
//                1996-2019  Nagoya Institute of Technology          //
//                           Department of Computer Science          //
//                                                                   //
// All rights reserved.                                              //
//                                                                   //
// Redistribution and use in source and binary forms, with or        //
// without modification, are permitted provided that the following   //
// conditions are met:                                               //
//                                                                   //
// - Redistributions of source code must retain the above copyright  //
//   notice, this list of conditions and the following disclaimer.   //
// - Redistributions in binary form must reproduce the above         //
//   copyright notice, this list of conditions and the following     //
//   disclaimer in the documentation and/or other materials provided //
//   with the distribution.                                          //
// - Neither the name of the SPTK working group nor the names of its //
//   contributors may be used to endorse or promote products derived //
//   from this software without specific prior written permission.   //
//                                                                   //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND            //
// CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,       //
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF          //
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE          //
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS //
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,          //
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED   //
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     //
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON //
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,   //
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY    //
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           //
// POSSIBILITY OF SUCH DAMAGE.                                       //
// ----------------------------------------------------------------- //

#include "SPTK/normalizer/generalized_cepstrum_gain_normalization.h"

#include <algorithm>  // std::copy
#include <cmath>      // std::pow, std::exp
#include <cstddef>    // std::size_t

namespace sptk {

GeneralizedCepstrumGainNormalization::GeneralizedCepstrumGainNormalization(
    int num_order, double gamma)
    : num_order_(num_order), gamma_(gamma), is_valid_(true) {
  if (num_order_ < 0) {
    is_valid_ = false;
  }
}

bool GeneralizedCepstrumGainNormalization::Run(
    const std::vector<double>& generalized_cepstrum,
    std::vector<double>* normalized_generalized_cepstrum) const {
  if (!is_valid_ ||
      static_cast<std::size_t>(num_order_ + 1) != generalized_cepstrum.size() ||
      NULL == normalized_generalized_cepstrum) {
    return false;
  }

  if (normalized_generalized_cepstrum->size() !=
      static_cast<std::size_t>(num_order_ + 1)) {
    normalized_generalized_cepstrum->resize(num_order_ + 1);
  }

  if (0.0 != gamma_) {
    const double* c1(&generalized_cepstrum[0]);
    double* c2(&(*normalized_generalized_cepstrum)[0]);
    const double k(1.0 + gamma_ * c1[0]);
    for (int m(num_order_); 1 <= m; --m) {
      c2[m] = c1[m] / k;
    }
    c2[0] = std::pow(k, 1.0 / gamma_);
  } else {
    std::copy(generalized_cepstrum.begin() + 1, generalized_cepstrum.end(),
              normalized_generalized_cepstrum->begin() + 1);
    (*normalized_generalized_cepstrum)[0] = std::exp(generalized_cepstrum[0]);
  }

  return true;
}

}  // namespace sptk
