#include "adas/AdasLogic.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <utility>

namespace adas {

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

    // 1) Točka gdje objekt dodiruje cestu: sredina donjeg ruba boxa
    const cv::Point2f groundPoint(
        static_cast<float>(det.box.x) + 0.5F * static_cast<float>(det.box.width),
        static_cast<float>(det.box.y + det.box.height));

    risk.inRoi = cv::pointPolygonTest(roiPixels_, groundPoint, false) >= 0.0;
    if (!risk.inRoi) {
        return risk;  // nije u našoj traci -> None
    }

    // 2) U našoj je traci -> barem Caution
    risk.level = RiskLevel::Caution;

    // 3) Je li blizu? (veličina boxa u odnosu na frame)
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