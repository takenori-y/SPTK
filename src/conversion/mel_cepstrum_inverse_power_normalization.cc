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

#include "SPTK/conversion/mel_cepstrum_inverse_power_normalization.h"

#include <algorithm>  // std::copy
#include <cstddef>    // std::size_t

namespace sptk {

MelCepstrumInversePowerNormalization::MelCepstrumInversePowerNormalization(
    int num_order)
    : num_order_(num_order), is_valid_(true) {
  if (num_order_ < 0) {
    is_valid_ = false;
    return;
  }
}

bool MelCepstrumInversePowerNormalization::Run(
    const std::vector<double>& power_normalized_mel_cepstrum,
    std::vector<double>* mel_cepstrum) const {
  // Check inputs.
  if (!is_valid_ ||
      power_normalized_mel_cepstrum.size() !=
          static_cast<std::size_t>(num_order_ + 2) ||
      NULL == mel_cepstrum) {
    return false;
  }

  // Prepare memories.
  if (mel_cepstrum->size() != static_cast<std::size_t>(num_order_ + 1)) {
    mel_cepstrum->resize(num_order_ + 1);
  }

  // Convert.
  (*mel_cepstrum)[0] =
      power_normalized_mel_cepstrum[0] + power_normalized_mel_cepstrum[1];
  std::copy(power_normalized_mel_cepstrum.begin() + 2,
            power_normalized_mel_cepstrum.end(), mel_cepstrum->begin() + 1);

  return true;
}

}  // namespace sptk
