#include "adas/Detector.hpp"
#include "adas/Timer.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace adas {


Detector::Detector(DetectorConfig config)
    : config_(std::move(config)),
      env_(ORT_LOGGING_LEVEL_WARNING, "adas") {
   
    sessionOptions_.SetIntraOpNumThreads(config_.numThreads);
    sessionOptions_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    
    const std::filesystem::path modelPath(config_.modelPath);
    if (!std::filesystem::exists(modelPath)) {
        throw std::runtime_error("Model ne postoji: " + config_.modelPath);
    }
    session_ = Ort::Session(env_, modelPath.c_str(), sessionOptions_);
    memoryInfo_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  
    Ort::AllocatorWithDefaultOptions allocator;
    inputName_  = session_.GetInputNameAllocated(0, allocator).get();
    outputName_ = session_.GetOutputNameAllocated(0, allocator).get();

    
    const auto shape = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (shape.size() != 4 || shape[1] != 3 || shape[2] != config_.inputSize ||
        shape[3] != config_.inputSize) {
        throw std::runtime_error("Neočekivan oblik ulaza modela");
    }
    inputShape_ = {1, 3, config_.inputSize, config_.inputSize};
}


Detector::Letterbox Detector::preprocess(const cv::Mat& frame) {
    const int target = config_.inputSize;

    // 1) Koliko moramo smanjiti da cijela slika stane u 640x640
    Letterbox lb;
    lb.scale = std::min(static_cast<float>(target) / static_cast<float>(frame.cols),
                        static_cast<float>(target) / static_cast<float>(frame.rows));

    const int newW = static_cast<int>(std::round(static_cast<float>(frame.cols) * lb.scale));
    const int newH = static_cast<int>(std::round(static_cast<float>(frame.rows) * lb.scale));
    lb.padX = (target - newW) / 2;
    lb.padY = (target - newH) / 2;

    // 2) Sivo platno 640x640 (memorija se alocira samo prvi put)
    letterboxed_.create(target, target, CV_8UC3);
    letterboxed_.setTo(cv::Scalar(114, 114, 114));

    // 3) Smanjenu sliku upiši direktno u sredinu platna
    cv::Mat roi = letterboxed_(cv::Rect(lb.padX, lb.padY, newW, newH));
    cv::resize(frame, roi, roi.size(), 0, 0, cv::INTER_LINEAR);

    // 4) BGR->RGB, 0-255 -> 0-1, HWC -> CHW, sve u jednom pozivu
    cv::dnn::blobFromImage(letterboxed_, blob_, 1.0 / 255.0, cv::Size(), cv::Scalar(),
                           /*swapRB=*/true, /*crop=*/false, CV_32F);
    return lb;
}

// =====================================================================
//  Pomoćne stvari (vidljive samo u ovoj datoteci)
// =====================================================================
namespace {

constexpr int kNumBoxValues = 4;  // cx, cy, w, h

// COCO id -> naša klasa; std::nullopt = ta klasa nas ne zanima
std::optional<ObjectClass> mapCocoClass(int cocoId) noexcept {
    switch (cocoId) {
        case 0: return ObjectClass::Person;
        case 1: return ObjectClass::Bicycle;
        case 2: return ObjectClass::Car;
        case 3: return ObjectClass::Motorcycle;
        case 5: return ObjectClass::Bus;
        case 7: return ObjectClass::Truck;
        default: return std::nullopt;
    }
}

}  // namespace

// =====================================================================
//  detect(): cijeli tok za jedan frame
// =====================================================================
void Detector::detect(const cv::Mat& frame, std::vector<Detection>& out) {
    out.clear();
    if (frame.empty()) {
        return;
    }

    // ---- 1) Preprocessing ----
    Timer timer;
    const Letterbox lb = preprocess(frame);
    timings_.preprocessMs = timer.elapsedMs();

    // ---- 2) Inferencija ----
    timer.reset();

    // Tenzor koji samo "gleda" u memoriju blob_-a (bez kopiranja)
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo_, blob_.ptr<float>(), blob_.total(),
        inputShape_.data(), inputShape_.size());

    const std::array<const char*, 1> inputNames{inputName_.c_str()};
    const std::array<const char*, 1> outputNames{outputName_.c_str()};

    auto outputs = session_.Run(Ort::RunOptions{nullptr},
                                inputNames.data(), &inputTensor, 1,
                                outputNames.data(), 1);
    timings_.inferenceMs = timer.elapsedMs();

    // ---- 3) Postprocessing ----
    timer.reset();

    const auto outShape = outputs.front().GetTensorTypeAndShapeInfo().GetShape();  // [1, 84, 8400]
    const int numChannels   = static_cast<int>(outShape[1]);
    const int numCandidates = static_cast<int>(outShape[2]);

    // Izlaz "okrenemo": [84 x 8400] -> [8400 x 84], svaki red = jedan kandidat
    const cv::Mat raw(numChannels, numCandidates, CV_32F,
                      outputs.front().GetTensorMutableData<float>());
    cv::transpose(raw, outputT_);

    postprocess(lb, frame.size(), out);
    timings_.postprocessMs = timer.elapsedMs();
}

// =====================================================================
//  Postprocessing: 8400 kandidata -> par detekcija
// =====================================================================
void Detector::postprocess(const Letterbox& lb, cv::Size frameSize,
                           std::vector<Detection>& out) {
    candBoxes_.clear();
    candScores_.clear();
    candClassIds_.clear();

    const int numClasses = outputT_.cols - kNumBoxValues;
    const cv::Rect frameRect(0, 0, frameSize.width, frameSize.height);

    for (int i = 0; i < outputT_.rows; ++i) {
        const float* row = outputT_.ptr<float>(i);

        // a) Koja klasa ima najveću vjerojatnost?
        const float* scoresBegin = row + kNumBoxValues;
        const float* best = std::max_element(scoresBegin, scoresBegin + numClasses);
        const float score = *best;

        // b) Filtriranje: preslab ili klasa koja nas ne zanima -> preskoči
        if (score < config_.confidenceThreshold) {
            continue;
        }
        const int cocoId = static_cast<int>(best - scoresBegin);
        if (!mapCocoClass(cocoId)) {
            continue;
        }

        // c) Box: centar -> gornji lijevi kut, 640x640 -> originalni frame
        const float cx = row[0];
        const float cy = row[1];
        const float w  = row[2];
        const float h  = row[3];

        const float x = (cx - 0.5F * w - static_cast<float>(lb.padX)) / lb.scale;
        const float y = (cy - 0.5F * h - static_cast<float>(lb.padY)) / lb.scale;

        cv::Rect box(static_cast<int>(x), static_cast<int>(y),
                     static_cast<int>(w / lb.scale), static_cast<int>(h / lb.scale));
        box &= frameRect;  // odreži dio koji viri izvan slike
        if (box.area() <= 0) {
            continue;
        }

        candBoxes_.push_back(box);
        candScores_.push_back(score);
        candClassIds_.push_back(cocoId);
    }

    // d) NMS: od više preklapajućih boxova iste klase zadrži najjači
    cv::dnn::NMSBoxesBatched(candBoxes_, candScores_, candClassIds_,
                             config_.confidenceThreshold, config_.nmsThreshold,
                             keptIndices_);

    // e) Napuni rezultat
    for (const int idx : keptIndices_) {
        const auto k = static_cast<std::size_t>(idx);
        // mapCocoClass ovdje sigurno vraća vrijednost (filtrirali smo u koraku b)
        out.push_back(Detection{candBoxes_[k], candScores_[k], *mapCocoClass(candClassIds_[k])});
    }
}

}  // namespace adas