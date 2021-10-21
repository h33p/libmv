// Copyright (c) 2007, 2008 libmv authors.
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

#include <iostream>
#include <string>

#include "libmv/image/image.h"
#include "libmv/image/image_io.h"
#include "testing/testing.h"

using libmv::Array3Df;
using libmv::Array3Du;
using libmv::FloatImage;
using libmv::GetFormat;
using std::string;

#include <cstdio>

namespace {

class ImageIOTest : public testing::Test {
 public:
  string TmpFile(const char* filename) {
    string temp_filename = string(THIS_SOURCE_DIR) + "/image_test/" + filename;
    temp_files_.push_back(temp_filename);
    return temp_filename;
  }
  void TearDown() {
    for (size_t i = 0; i < temp_files_.size(); ++i) {
      fprintf(stderr, "unlinking: %s\n", temp_files_[i].c_str());
      unlink(temp_files_[i].c_str());
    }
  }

 private:
  std::vector<string> temp_files_;
};

TEST(ReadPnm, InvalidFiles) {
  Array3Du image;
  Array3Df float_image;
  string png_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.png";
  EXPECT_FALSE(ReadPnm(png_filename.c_str(), &image));
  EXPECT_FALSE(ReadPnm("hopefully_unexisting_file", &image));
  EXPECT_FALSE(ReadPnm(png_filename.c_str(), &float_image));
  EXPECT_FALSE(ReadPnm("hopefully_unexisting_file", &float_image));
}

TEST(ReadPnm, Pgm) {
  Array3Du image;
  string pgm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.pgm";
  EXPECT_TRUE(ReadPnm(pgm_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Depth());
  EXPECT_EQ(image(0, 0), (unsigned char)255);
  EXPECT_EQ(image(0, 1), (unsigned char)0);
}

TEST(ReadPnm, PgmComments) {
  Array3Du image;
  string pgm_filename =
      string(THIS_SOURCE_DIR) + "/image_test/two_pixels_gray.pgm";
  EXPECT_TRUE(ReadPnm(pgm_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Depth());
  EXPECT_EQ(image(0, 0), (unsigned char)255);
  EXPECT_EQ(image(0, 1), (unsigned char)0);
}

TEST(ReadPnm, PgmFloat) {
  FloatImage image;
  string pgm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.pgm";
  EXPECT_TRUE(ReadPnm(pgm_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Depth());
  EXPECT_EQ(image(0, 0), 1);
  EXPECT_EQ(image(0, 1), 0);
}

TEST_F(ImageIOTest, Pgm) {
  Array3Du image(1, 2);
  image(0, 0) = 255;
  image(0, 1) = 0;
  string out_filename = TmpFile("test_write_pnm.pgm");
  EXPECT_TRUE(WritePnm(image, out_filename.c_str()));

  Array3Du read_image;
  EXPECT_TRUE(ReadPnm(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
}

TEST(ReadPnm, Ppm) {
  Array3Du image;
  string ppm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.ppm";
  EXPECT_TRUE(ReadPnm(ppm_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(3, image.Depth());
  EXPECT_EQ(image(0, 0, 0), (unsigned char)255);
  EXPECT_EQ(image(0, 0, 1), (unsigned char)255);
  EXPECT_EQ(image(0, 0, 2), (unsigned char)255);
  EXPECT_EQ(image(0, 1, 0), (unsigned char)0);
  EXPECT_EQ(image(0, 1, 1), (unsigned char)0);
  EXPECT_EQ(image(0, 1, 2), (unsigned char)0);
}

TEST_F(ImageIOTest, Ppm) {
  Array3Du image(1, 2, 3);
  image(0, 0, 0) = 255;
  image(0, 0, 1) = 255;
  image(0, 0, 2) = 255;
  image(0, 1, 0) = 0;
  image(0, 1, 1) = 0;
  image(0, 1, 2) = 0;
  string out_filename = TmpFile("test_write_pnm.ppm");
  EXPECT_TRUE(WritePnm(image, out_filename.c_str()));

  Array3Du read_image;
  EXPECT_TRUE(ReadPnm(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
}

TEST(ReadPng, InvalidFiles) {
  Array3Du image;
  Array3Df float_image;
  string pnm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.ppm";
  EXPECT_FALSE(ReadPng(pnm_filename.c_str(), &image));
  EXPECT_FALSE(ReadPng("hopefully_unexisting_file", &image));
  EXPECT_FALSE(ReadPng(pnm_filename.c_str(), &float_image));
  EXPECT_FALSE(ReadPng("hopefully_unexisting_file", &float_image));
}

TEST(ReadPng, Png) {
  Array3Du image;
  string png_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.png";
  EXPECT_TRUE(ReadPng(png_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Depth());
  EXPECT_EQ(image(0, 0), (unsigned char)255);
  EXPECT_EQ(image(0, 1), (unsigned char)0);
}

TEST(ReadPng, PngFloat) {
  FloatImage image;
  string png_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.png";
  EXPECT_TRUE(ReadPng(png_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Depth());
  EXPECT_EQ(image(0, 0), 1);
  EXPECT_EQ(image(0, 1), 0);
}

TEST_F(ImageIOTest, Png) {
  Array3Du image(1, 2);
  image(0, 0) = 255;
  image(0, 1) = 0;
  string out_filename = TmpFile("test_write_png.png");
  EXPECT_TRUE(WritePng(image, out_filename.c_str()));

  Array3Du read_image;
  EXPECT_TRUE(ReadPng(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
}

TEST_F(ImageIOTest, InvalidFiles) {
  Array3Du image;
  Array3Df float_image;
  string pnm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.ppm";
  EXPECT_FALSE(ReadJpg(pnm_filename.c_str(), &image));
  EXPECT_FALSE(ReadJpg("hopefully_unexisting_file", &image));
  EXPECT_FALSE(ReadJpg(pnm_filename.c_str(), &float_image));
  EXPECT_FALSE(ReadJpg("hopefully_unexisting_file", &float_image));
}

TEST(ReadJpg, Jpg) {
  Array3Du image;
  string jpg_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.jpg";
  EXPECT_TRUE(ReadJpg(jpg_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Depth());
  EXPECT_EQ(image(0, 0), (unsigned char)255);
  EXPECT_EQ(image(0, 1), (unsigned char)0);
}

TEST(ReadJpg, JpgFloat) {
  FloatImage image;
  string jpg_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.jpg";
  EXPECT_TRUE(ReadJpg(jpg_filename.c_str(), &image));
  EXPECT_EQ(2, image.Width());
  EXPECT_EQ(1, image.Height());
  EXPECT_EQ(1, image.Depth());
  EXPECT_EQ(image(0, 0), 1);
  EXPECT_EQ(image(0, 1), 0);
}

TEST_F(ImageIOTest, Jpg) {
  Array3Du image(1, 2);
  image(0, 0) = 255;
  image(0, 1) = 0;
  string out_filename = TmpFile("test_write_jpg.jpg");
  EXPECT_TRUE(WriteJpg(image, out_filename.c_str(), 100));

  Array3Du read_image;
  EXPECT_TRUE(ReadJpg(out_filename.c_str(), &read_image));
  EXPECT_TRUE(read_image == image);
}

TEST(GetFormat, filenames) {
  EXPECT_EQ(GetFormat("something.jpg"), libmv::Jpg);
  EXPECT_EQ(GetFormat("something.png"), libmv::Png);
  EXPECT_EQ(GetFormat("something.pnm"), libmv::Pnm);
  EXPECT_EQ(GetFormat("/some/thing.JpG"), libmv::Jpg);
  EXPECT_EQ(GetFormat("/some/thing.pNG"), libmv::Png);
  EXPECT_EQ(GetFormat("some/thing.PNm"), libmv::Pnm);
  EXPECT_EQ(GetFormat(".s/o.m/e.t/h.i/n.g.JPG"), libmv::Jpg);
  EXPECT_EQ(GetFormat(".s/o.m/e.t/h.i/n.g.PNG"), libmv::Png);
  EXPECT_EQ(GetFormat(".s/o.m/e.t/h.i/n.g.PNM"), libmv::Pnm);
}

}  // namespace
