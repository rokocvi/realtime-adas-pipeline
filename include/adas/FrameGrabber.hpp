#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <string>

namespace adas {


class FrameGrabber {
public:
    explicit FrameGrabber(const std::string& source);

  
    FrameGrabber(const FrameGrabber&) = delete;
    FrameGrabber& operator=(const FrameGrabber&) = delete;

    
    [[nodiscard]] bool grab(cv::Mat& frame);

    [[nodiscard]] double fps() const noexcept { return fps_; }
    [[nodiscard]] cv::Size frameSize() const noexcept { return frameSize_; }

private:
    cv::VideoCapture capture_;
    double fps_{0.0};
    cv::Size frameSize_{};
};

}  // namespace adas