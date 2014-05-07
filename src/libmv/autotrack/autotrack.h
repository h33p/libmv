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
// Typical usage for full autotrack:
//
//   AutoTrack auto_track(image_accessor);
//   auto_track.SetNumFramesInClip(0, 10);
//   auto_track.SetNumFramesInClip(1, 54);
//   auto_track.AutoTrack()
//
// It is also possible to specify options to control the reconstruction.
// Furthermore, the individual methods of reconstruction are exposed to make it
// possible to interact with the pipeline as it runs. For example, to track one
// marker across frames,
//
//   AutoTrack auto_track(image_accessor);
//   auto_track.SetNumFramesInClip(0, 10);
//   auto_track.SetNumFramesInClip(1, 54);
//   auto_track.AddMarker(...);
//   auto_track.TrackMarkerToFrame(int clip1, int frame1,
//                                 int clip2, int frame2,
//                                 options?)
//
class AutoTrack {

  // Marker manipulation.
  // Clip manipulation.

  // Set the number of clips. These clips will get accessed from the frame
  // accessor, matches between frames found, and a reconstruction created.
  void SetNumFrames(int clip, int num_frames);

  // Tracking & Matching
  void TrackMarkerToFrame(const Marker& reference_marker,
                          int to_clip, int to_frame,
                          Marker* tracked_marker);

  // TODO(keir): Implement frame matching! This could be very cool for loop
  // closing and connecting across clips.
  void MatchFrames(int clip1, int frame1, int clip2, int frame2) {}

  // Reconstruct!
  // State machine? Have explicit state? Or determine state from existing data?
  // Probably determine state from existing data.
  // States
  //  - No tracks or markers

 private:
  Tracks tracks_;  // May be normalized camera coordinates or raw pixels.

  // TODO(keir): Should num_clips and num_frames get moved to FrameAccessor?
  FrameAccessor* frame_accessor_;
  int num_clips_;
  vector<int> num_frames_;  // Indexed by clip.
};

}  // namespace mv

#endif  // LIBMV_AUTOTRACK_AUTOTRACK_H_
