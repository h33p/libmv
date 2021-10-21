// Copyright (c) 2009 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include "libmv/descriptor/surf_descriptor.h"

#include "libmv/correspondence/feature.h"
#include "libmv/descriptor/descriptor.h"
#include "libmv/descriptor/vector_descriptor.h"
#include "libmv/image/convolve.h"
#include "libmv/image/image.h"
#include "libmv/image/integral_image.h"
#include "libmv/logging/logging.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
namespace descriptor {

template <typename TImage>
float HarrX(const TImage& integral_image, int row, int col, int scale) {
  // Ignore the center strip for odd scales.
  int HW = scale / 2, W = scale;
  int C = W % 2;
  return float(BoxIntegral(integral_image, row - HW, col + C, W, HW)) -
         float(BoxIntegral(integral_image, row - HW, col - HW, W, HW));
}

template <typename TImage>
float HarrY(const TImage& integral_image, int row, int col, int scale) {
  // Ignore the center strip for odd scales.
  int HW = scale / 2, W = scale;
  int C = W % 2;
  return float(BoxIntegral(integral_image, row + C, col - HW, W, HW)) -
         float(BoxIntegral(integral_image, row - HW, col - HW, W, HW));
}

// TODO(keir): Concievably, these template parameters could be exposed to
// experiment with SURF parameters. Try that!
// descriptor must be sized to 4 * blocks *blocks
template <int blocks,
          int samples_per_block,
          typename TImage,
          typename TPointFeature>
void USURFDescriptor(const TImage& integral_image,
                     const TPointFeature& feature,
                     Vecf* descriptor) {
  float x = feature.x();
  float y = feature.y();
  float scale = feature.scale;
  const int int_scale = lround(2 * scale);
  const int half_region = blocks * samples_per_block / 2;

  // Since the Gaussian is a separable filter, precompute it.
  Matrix<float, 2 * half_region, 1> gaussian;
  for (int i = -half_region; i < half_region; ++i) {
    gaussian(i + half_region) = Gaussian(i, 3.3 * scale);
  }

  int done_dims = 0;
  for (int row = -half_region; row < half_region; row += samples_per_block) {
    for (int col = -half_region; col < half_region; col += samples_per_block) {
      Vec4f components(0, 0, 0, 0);
      for (int r = row; r < row + samples_per_block; ++r) {
        for (int c = col; c < col + samples_per_block; ++c) {
          int sample_row = lround(y + scale * r);
          int sample_col = lround(x + scale * c);
          float weight = gaussian(r + half_region) * gaussian(c + half_region);
          Vec2f dxy;
          dxy << HarrX(integral_image, sample_row, sample_col, int_scale),
              HarrY(integral_image, sample_row, sample_col, int_scale);
          dxy *= weight;
          components.head<2>() += dxy.matrix();
          components.tail<2>() += dxy.array().abs().matrix();
        }
      }
      (*descriptor).segment<4>(done_dims) = components;
      done_dims += 4;
    }
  }
  descriptor->normalize();
}

// descriptor must be sized to 4 * blocks *blocks
template <int blocks,
          int samples_per_block,
          typename TImage,
          typename TPointFeature>
void MSURFDescriptor(const TImage& integral_image,
                     const TPointFeature& feature,
                     Vecf* descriptor) {
  float x = feature.x();
  float y = feature.y();
  float scale = feature.scale;
  const int int_scale = lround(2 * scale);
  // Allow overlap between blocks
  const int half_region = blocks * (samples_per_block - (blocks - 1)) / 2;

  // Since the Gaussian is a separable filter, precompute it.
  Matrix<float, 2 * half_region, 1> gaussian;
  for (int i = -half_region; i < half_region; ++i) {
    gaussian(i + half_region) = Gaussian(i, 2 * scale);
  }

  float co = 1.0f, si = 0.0f;  // Suppose Upright descriptor
  bool bUpright = false;       // TODO(pmoulon) set this variable as parameter
  if (!bUpright) {
    co = cos(feature.orientation);
    si = sin(feature.orientation);
  }

  int done_dims = 0;
  for (int row = -half_region; row < half_region - blocks;
       row += (samples_per_block - blocks)) {
    for (int col = -half_region; col < half_region - blocks;
         col += (samples_per_block - blocks)) {
      Vec4f components(0.0f, 0.0f, 0.0f, 0.0f);
      for (int r = row; r < row + samples_per_block; ++r) {
        for (int c = col; c < col + samples_per_block; ++c) {
          // Get coords of sample point on the rotated axis
          int sample_col = lround(x + (-r * scale * si + c * scale * co));
          int sample_row = lround(y + (r * scale * co + c * scale * si));

          float weight = gaussian(r + half_region) * gaussian(c + half_region);
          // Compute Harr response on rotated axis
          float rrx = HarrX(integral_image, sample_row, sample_col, int_scale);
          float rry = HarrY(integral_image, sample_row, sample_col, int_scale);

          Vec2f dxy;
          dxy << (co * rry - si * rrx), (si * rry + co * rrx);
          dxy *= weight;
          components.head<2>() += dxy;
          components.tail<2>().array() += dxy.array().abs();
        }
      }
      float gauss_BigBox =
          Gaussian2D(row / half_region, col / half_region, 1.0f);
      (*descriptor).segment<4>(done_dims).matrix() = components * gauss_BigBox;
      done_dims += 4;
    }
  }
  descriptor->normalize();
}

class SurfDescriber : public Describer {
 public:
  virtual void Describe(const vector<Feature*>& features,
                        const Image& image,
                        const detector::DetectorData* detector_data,
                        vector<Descriptor*>* descriptors) {
    // TODO(keir): Make the descriptor data the SURF detector integral image.
    (void)detector_data;

    Matu integral_image;
    IntegralImage(*(image.AsArray3Du()), &integral_image);

    descriptors->resize(features.size());
    for (int i = 0; i < features.size(); ++i) {
      PointFeature* point = dynamic_cast<PointFeature*>(features[i]);
      VecfDescriptor* descriptor = NULL;
      if (point) {
        descriptor = new VecfDescriptor(64);
        MSURFDescriptor<4, 9>(integral_image, *point, &descriptor->coords);
      }
      (*descriptors)[i] = descriptor;
    }
  }
};

Describer* CreateSurfDescriber() {
  return new SurfDescriber;
}

}  // namespace descriptor
}  // namespace libmv
