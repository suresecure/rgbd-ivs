#pragma once

namespace suresecure-rgbd-ivs{
struct BgSample{
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char grey;
  unsigned char grad;
  unsigned short d;
};

class RgbdCodebook{
  public:
    RgbdCodebook(int model_num, int height, int width);
    void Init(const Mat&rgb_frame, const Mat& depth_frame);
    void Process(const Mat&rgb_frame, const Mat& depth_frame);
    const Mat& GetFg()const;

  private:

    int height_;
    int width_;
    int model_num_;
    BgSample *bg_samples_;
};
}
