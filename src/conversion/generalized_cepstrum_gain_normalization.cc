// ------------------------------------------------------------------------ //
// Copyright 2021 SPTK Working Group                                        //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ------------------------------------------------------------------------ //

#include "SPTK/conversion/generalized_cepstrum_gain_normalization.h"

#include <algorithm>  // std::copy, std::transform
#include <cmath>      // std::exp, std::pow
#include <cstddef>    // std::size_t
#include <vector>     // std::vector

namespace sptk {

GeneralizedCepstrumGainNormalization::GeneralizedCepstrumGainNormalization(
    int num_order, double gamma)
    : num_order_(num_order), gamma_(gamma), is_valid_(true) {
  if (num_order_ < 0 || !sptk::IsValidGamma(gamma_)) {
    is_valid_ = false;
    return;
  }
}

bool GeneralizedCepstrumGainNormalization::Run(
    const std::vector<double>& generalized_cepstrum,
    std::vector<double>* normalized_generalized_cepstrum) const {
  // Check inputs.
  if (!is_valid_ ||
      generalized_cepstrum.size() != static_cast<std::size_t>(num_order_ + 1) ||
      NULL == normalized_generalized_cepstrum) {
    return false;
  }

  // Prepare memories.
  if (normalized_generalized_cepstrum->size() !=
      static_cast<std::size_t>(num_order_ + 1)) {
    normalized_generalized_cepstrum->resize(num_order_ + 1);
  }

  if (0.0 == gamma_) {
    (*normalized_generalized_cepstrum)[0] = std::exp(generalized_cepstrum[0]);
    std::copy(generalized_cepstrum.begin() + 1, generalized_cepstrum.end(),
              normalized_generalized_cepstrum->begin() + 1);
  } else {
    const double z(1.0 + gamma_ * generalized_cepstrum[0]);
    (*normalized_generalized_cepstrum)[0] = std::pow(z, 1.0 / gamma_);
    std::transform(generalized_cepstrum.begin() + 1, generalized_cepstrum.end(),
                   normalized_generalized_cepstrum->begin() + 1,
                   [z](double c) { return c / z; });
  }

  return true;
}

bool GeneralizedCepstrumGainNormalization::Run(
    std::vector<double>* input_and_output) const {
  if (NULL == input_and_output) return false;
  return Run(*input_and_output, input_and_output);
}

}  // namespace sptk
