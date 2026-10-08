#pragma once

#include "adas/Types.hpp"

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace adas {


struct DetectorConfig {
    std::string modelPath;
    float confidenceThreshold{0.40F};  // ispod ovoga detekciju odbacujemo
    float nmsThreshold{0.45F};         // koliko se boxovi smiju preklapati prije NMS-a
    int inputSize{640};                
    int numThreads{4};                 
};


struct DetectorTimings {
    double preprocessMs{0.0};
    double inferenceMs{0.0};
    double postprocessMs{0.0};
};

class Detector {
public:
    explicit Detector(DetectorConfig config);

    Detector(const Detector&) = delete;
    Detector& operator=(const Detector&) = delete;

    // Detektira objekte na frameu i upisuje ih u 'out' (ponovno koristi memoriju)
    void detect(const cv::Mat& frame, std::vector<Detection>& out);

    [[nodiscard]] const DetectorTimings& lastTimings() const noexcept { return timings_; }

private:
    struct Letterbox {
        float scale{1.0F};
        int padX{0};
        int padY{0};
    };

    Letterbox preprocess(const cv::Mat& frame);
    void postprocess(const Letterbox& lb, cv::Size frameSize, std::vector<Detection>& out);

    DetectorConfig config_;

    
    Ort::Env env_;
    Ort::SessionOptions sessionOptions_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memoryInfo_{nullptr};

    std::string inputName_;
    std::string outputName_;
    std::array<std::int64_t, 4> inputShape_{};

    
    cv::Mat letterboxed_;
    cv::Mat blob_;
    cv::Mat outputT_;
    std::vector<cv::Rect> candBoxes_;
    std::vector<float> candScores_;
    std::vector<int> candClassIds_;
    std::vector<int> keptIndices_;

    DetectorTimings timings_;
};

} 