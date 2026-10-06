#include "adas/FrameGrabber.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace adas {

namespace {

bool isCameraIndex(const std::string& s) {
    return !s.empty() &&
           std::all_of(s.begin(), s.end(),
                       [](unsigned char c) { return std::isdigit(c) != 0; });
}
}  

FrameGrabber::FrameGrabber(const std::string& source) {
    if (isCameraIndex(source)) {
        capture_.open(std::stoi(source));
    } else {
        capture_.open(source);
    }

    if (!capture_.isOpened()) {
        throw std::runtime_error("Ne mogu otvoriti video izvor: " + source);
    }

    fps_ = capture_.get(cv::CAP_PROP_FPS);
    frameSize_ = cv::Size(static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH)),
                          static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT)));
}

bool FrameGrabber::grab(cv::Mat& frame) {
    return capture_.read(frame) && !frame.empty();
}

} 