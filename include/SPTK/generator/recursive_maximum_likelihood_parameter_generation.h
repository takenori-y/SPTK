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

#ifndef SPTK_GENERATOR_RECURSIVE_MAXIMUM_LIKELIHOOD_PARAMETER_GENERATION_H_
#define SPTK_GENERATOR_RECURSIVE_MAXIMUM_LIKELIHOOD_PARAMETER_GENERATION_H_

#include <vector>  // std::vector

#include "SPTK/input/input_source_interface.h"
#include "SPTK/utils/sptk_utils.h"

namespace sptk {

class RecursiveMaximumLikelihoodParameterGeneration {
 public:
  //
  RecursiveMaximumLikelihoodParameterGeneration(
      int num_order, int num_past_frame,
      const std::vector<std::vector<double> >& window_coefficients,
      InputSourceInterface* input_source);

  //
  virtual ~RecursiveMaximumLikelihoodParameterGeneration() {
  }

  //
  int GetNumOrder() const {
    return num_order_;
  }

  //
  int GetNumPastFrame() const {
    return num_past_frame_;
  }

  //
  bool IsValid() const {
    return is_valid_;
  }

  //
  bool Get(std::vector<double>* smoothed_static_parameters);

 private:
  //
  struct Buffer {
    std::vector<double> static_and_dynamic_parameters;
    std::vector<std::vector<double> > stored_dynamic_mean_vectors;
    std::vector<std::vector<double> >
        stored_dynamic_diagonal_covariance_matrices;
    std::vector<std::vector<double> > pi;
    std::vector<std::vector<double> > k;
    std::vector<std::vector<std::vector<double> > > p;
    std::vector<std::vector<double> > c;
  };

  //
  bool Forward();

  //
  const int num_order_;

  //
  const int num_past_frame_;

  //
  const std::vector<std::vector<double> > window_coefficients_;

  //
  InputSourceInterface* input_source_;

  //
  bool is_valid_;

  //
  int calculation_field_;

  //
  int num_remaining_frame_;

  //
  int current_frame_;

  //
  Buffer buffer_;

  //
  DISALLOW_COPY_AND_ASSIGN(RecursiveMaximumLikelihoodParameterGeneration);
};

}  // namespace sptk

#endif  // SPTK_GENERATOR_RECURSIVE_MAXIMUM_LIKELIHOOD_PARAMETER_GENERATION_H_
