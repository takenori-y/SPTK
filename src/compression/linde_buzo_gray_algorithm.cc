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

#include "SPTK/compression/linde_buzo_gray_algorithm.h"

#include <cfloat>   // DBL_MAX
#include <cmath>    // std::fabs
#include <cstddef>  // std::size_t

#include "SPTK/generation/normal_distributed_random_value_generation.h"

namespace sptk {

LindeBuzoGrayAlgorithm::LindeBuzoGrayAlgorithm(
    int num_order, int seed, int initial_codebook_size,
    int target_codebook_size, int minimum_num_vector_in_cluster,
    int num_iteration, double convergence_threshold, double splitting_factor)
    : num_order_(num_order),
      seed_(seed),
      initial_codebook_size_(initial_codebook_size),
      target_codebook_size_(target_codebook_size),
      minimum_num_vector_in_cluster_(minimum_num_vector_in_cluster),
      num_iteration_(num_iteration),
      convergence_threshold_(convergence_threshold),
      splitting_factor_(splitting_factor),
      distance_calculator_(
          num_order_, DistanceCalculator::DistanceMetrics::kSquaredEuclidean),
      statistics_accumulator_(num_order_, 1),
      vector_quantization_(num_order_),
      is_valid_(true) {
  if (num_order_ < 0 || initial_codebook_size_ <= 0 ||
      target_codebook_size_ <= initial_codebook_size_ ||
      minimum_num_vector_in_cluster <= 0 || num_iteration_ <= 0 ||
      convergence_threshold < 0.0 || splitting_factor <= 0.0 ||
      !distance_calculator_.IsValid() || !statistics_accumulator_.IsValid() ||
      !vector_quantization_.IsValid()) {
    is_valid_ = false;
  }
}

bool LindeBuzoGrayAlgorithm::Run(
    const std::vector<std::vector<double> >& input_vectors,
    std::vector<std::vector<double> >* codebook_vectors,
    std::vector<int>* codebook_index) const {
  // check inputs
  const int num_input_vector(input_vectors.size());
  if (!is_valid_ ||
      num_input_vector <
          minimum_num_vector_in_cluster_ * target_codebook_size_ ||
      NULL == codebook_vectors || NULL == codebook_index ||
      codebook_vectors->size() !=
          static_cast<std::size_t>(initial_codebook_size_)) {
    return false;
  }

  // prepare memory
  if (codebook_index->size() != static_cast<std::size_t>(num_input_vector)) {
    codebook_index->resize(num_input_vector);
  }
  std::vector<StatisticsAccumulator::Buffer> buffers(target_codebook_size_);

  // prepare random value generator
  NormalDistributedRandomValueGeneration random_value_generation(seed_);

  // design codebook
  int current_codebook_size(initial_codebook_size_);
  while (2 * current_codebook_size <= target_codebook_size_) {
    // increase codebook size by two times
    codebook_vectors->resize(2 * current_codebook_size);
    for (std::vector<std::vector<double> >::iterator itr(
             codebook_vectors->begin() + current_codebook_size);
         itr != codebook_vectors->end(); ++itr) {
      itr->resize(num_order_ + 1);
    }
    for (int e(0); e < current_codebook_size; ++e) {
      for (int m(0); m <= num_order_; ++m) {
        double random_value;
        if (!random_value_generation.Get(&random_value)) {
          return false;
        }
        const double perturbation(splitting_factor_ * random_value);
        const int f(e + current_codebook_size);
        (*codebook_vectors)[f][m] = (*codebook_vectors)[e][m] - perturbation;
        (*codebook_vectors)[e][m] = (*codebook_vectors)[e][m] + perturbation;
      }
    }
    current_codebook_size *= 2;

    double prev_total_distance(DBL_MAX);
    for (int n(0); n < num_iteration_; ++n) {
      // initialize
      double total_distance(0.0);
      for (int e(0); e < current_codebook_size; ++e) {
        statistics_accumulator_.Clear(&(buffers[e]));
      }

      // accumulate statistics (E-step)
      for (int i(0); i < num_input_vector; ++i) {
        int index;
        if (!vector_quantization_.Run(input_vectors[i], *codebook_vectors,
                                      &index)) {
          return false;
        }
        (*codebook_index)[i] = index;

        if (!statistics_accumulator_.Run(input_vectors[i], &(buffers[index]))) {
          return false;
        }

        double distance;
        if (!distance_calculator_.Run(input_vectors[i],
                                      (*codebook_vectors)[index], &distance)) {
          return false;
        }
        total_distance += distance;
      }
      total_distance /= num_input_vector;

      // check convergence
      const double criterion_value(
          std::fabs(prev_total_distance - total_distance) / total_distance);
      if (0.0 == total_distance || criterion_value < convergence_threshold_) {
        break;
      }
      prev_total_distance = total_distance;

      // update codebook (M-step) and
      // search a cluster that contains maximum number of vectors
      int majority_index(-1);
      int maximum_num_vector_in_cluster(0);
      for (int e(0); e < current_codebook_size; ++e) {
        int num_vector;
        if (!statistics_accumulator_.GetNumData(buffers[e], &num_vector)) {
          return false;
        }

        if (minimum_num_vector_in_cluster_ <= num_vector) {
          if (!statistics_accumulator_.GetMean(buffers[e],
                                               &((*codebook_vectors)[e]))) {
            return false;
          }
        }

        if (maximum_num_vector_in_cluster < num_vector) {
          majority_index = e;
          maximum_num_vector_in_cluster = num_vector;
        }
      }

      for (int e(0); e < current_codebook_size; ++e) {
        int num_vector;
        if (!statistics_accumulator_.GetNumData(buffers[e], &num_vector)) {
          return false;
        }

        if (num_vector < minimum_num_vector_in_cluster_) {
          for (int m(0); m <= num_order_; ++m) {
            double random_value;
            if (!random_value_generation.Get(&random_value)) {
              return false;
            }
            const double perturbation(splitting_factor_ * random_value);
            const int f(majority_index);
            (*codebook_vectors)[e][m] =
                (*codebook_vectors)[f][m] - perturbation;
            (*codebook_vectors)[f][m] =
                (*codebook_vectors)[f][m] + perturbation;
          }
        }
      }
    }
  }

  // save final results
  for (int i(0); i < num_input_vector; ++i) {
    if (!vector_quantization_.Run(input_vectors[i], *codebook_vectors,
                                  &((*codebook_index)[i]))) {
      return false;
    }
  }

  return true;
}

}  // namespace sptk
