// Copyright (c) 2014 libmv authors.
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
//
// Author: mierle@gmail.com (Keir Mierle)

#ifndef LIBMV_AUTOTRACK_AUTOTRACK_H_
#define LIBMV_AUTOTRACK_AUTOTRACK_H_

#include "libmv/autotrack/tracks.h"

namespace mv {

class FrameAccessor;

// The coordinator of all tracking operations; keeps track of all state
// relating to tracking and reconstruction; for example, 2D tracks and motion
// models, reconstructed cameras, points, and planes; tracking settings; etc.
//
// TODO(keir): This class is not done yet! Much is to come.
class AutoTrack {


 private:
  Tracks tracks;  // Original tracks straight out of the camera.
  Tracks normalized_tracks;  // Tracks with distortion and focal removed.

  FrameAccessor* frame_accessor;
};

}  // namespace mv

#endif  // LIBMV_AUTOTRACK_AUTOTRACK_H_
