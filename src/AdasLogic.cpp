#include "adas/AdasLogic.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace adas {

namespace {

// Indeksi točaka ROI trapeza (vidi redoslijed u AdasConfig)
constexpr std::size_t kBottomLeft  = 0;
constexpr std::size_t kBottomRight = 1;
constexpr std::size_t kTopRight    = 2;
constexpr std::size_t kTopLeft     = 3;

// x koordinata na pravcu od a do b, za zadanu visinu y (linearna interpolacija)
float xAtY(const cv::Point& a, const cv::Point& b, float y) noexcept {
    const float dy = static_cast<float>(b.y - a.y);
    if (std::abs(dy) < 1e-3F) {
        return static_cast<float>(a.x);
    }
    const float t = (y - static_cast<float>(a.y)) / dy;
    return static_cast<float>(a.x) + t * static_cast<float>(b.x - a.x);
}

}  // namespace

AdasLogic::AdasLogic(AdasConfig config) : config_(std::move(config)) {}

// ROI iz postotaka pretvori u piksele (samo kad se promijeni veličina framea)
void AdasLogic::updateRoi(cv::Size frameSize) {
    if (frameSize == cachedSize_ && !roiPixels_.empty()) {
        return;
    }
    cachedSize_ = frameSize;
    roiPixels_.clear();
    for (const auto& p : config_.roi) {
        roiPixels_.emplace_back(static_cast<int>(p.x * static_cast<float>(frameSize.width)),
                                static_cast<int>(p.y * static_cast<float>(frameSize.height)));
    }
}

// Procjena rizika za jedan objekt
ObjectRisk AdasLogic::assess(const Detection& det, cv::Size frameSize) const {
    ObjectRisk risk;

    const cv::Point& bottomLeft  = roiPixels_[kBottomLeft];
    const cv::Point& bottomRight = roiPixels_[kBottomRight];
    const cv::Point& topRight    = roiPixels_[kTopRight];
    const cv::Point& topLeft     = roiPixels_[kTopLeft];

        // 0) Ignoriraj sve što počinje ispod ruba armature: to je naš vlastiti auto
    //    (pravi auto ispred nas uvijek seže IZNAD tog ruba)
    const float egoY = config_.egoLine * static_cast<float>(frameSize.height);
    if (static_cast<float>(det.box.y) >= egoY * 0.95F) {
        return risk;
    }

    // 1) Visina na kojoj objekt dodiruje cestu (donji rub boxa)
    const float groundY = static_cast<float>(det.box.y + det.box.height);

    // Iznad gornjeg ruba trapeza = predaleko, ne zanima nas
    if (groundY < static_cast<float>(topLeft.y)) {
        return risk;
    }
    const float y = std::min(groundY, static_cast<float>(bottomLeft.y));

    // 2) Gdje su lijevi i desni rub naše trake na toj visini
    const float laneLeft  = xAtY(bottomLeft, topLeft, y);
    const float laneRight = xAtY(bottomRight, topRight, y);

    // 3) Koliki dio širine objekta je unutar trake
    const float boxLeft  = static_cast<float>(det.box.x);
    const float boxRight = static_cast<float>(det.box.x + det.box.width);
    const float overlap  = std::max(0.0F, std::min(boxRight, laneRight) - std::max(boxLeft, laneLeft));
    const float overlapRatio = overlap / static_cast<float>(det.box.width);

    risk.inRoi = overlapRatio >= config_.minLaneOverlap;
    if (!risk.inRoi) {
        return risk;  // većina objekta je izvan naše trake -> None
    }

    // 4) U našoj je traci -> barem Caution
    risk.level = RiskLevel::Caution;

    // 5) Je li blizu? (veličina boxa u odnosu na frame)
    const bool isClose = isVulnerable(det.cls)
        ? static_cast<float>(det.box.height) / static_cast<float>(frameSize.height)
              >= config_.vruWarnHeightRatio
        : static_cast<float>(det.box.width) / static_cast<float>(frameSize.width)
              >= config_.vehicleWarnWidthRatio;

    if (isClose) {
        risk.level = RiskLevel::Warning;
    }
    return risk;
}

// Procjena za cijeli frame + stabilizacija upozorenja
void AdasLogic::evaluate(const std::vector<Detection>& detections, cv::Size frameSize,
                         AdasResult& result) {
    updateRoi(frameSize);

    result.perObject.clear();
    result.overall = RiskLevel::None;

    for (const auto& det : detections) {
        const ObjectRisk r = assess(det, frameSize);
        result.perObject.push_back(r);
        result.overall = std::max(result.overall, r.level);
    }

    // Stabilizacija: uključi nakon N uzastopnih frameova, drži još M frameova
    if (result.overall == RiskLevel::Warning) {
        ++riskStreak_;
    } else {
        riskStreak_ = 0;
    }

    if (riskStreak_ >= config_.framesToActivate) {
        holdCounter_ = config_.framesToHold;
    } else if (holdCounter_ > 0) {
        --holdCounter_;
    }

    result.warningActive = holdCounter_ > 0;
}

}  // namespace adas