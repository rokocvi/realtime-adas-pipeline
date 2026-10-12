#include "adas/AdasLogic.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace adas {

namespace {

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

AdasLogic::AdasLogic(AdasConfig config) : config_(std::move(config)) {
    trackForDetection_.reserve(64);
}

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

ObjectRisk AdasLogic::assess(const Detection& det, const Track* track, cv::Size frameSize) const {
    ObjectRisk risk;
    if (track != nullptr) {
        risk.trackId = track->id;
        risk.ttc = timeToCollision(*track);
    }

    // 0) Ignoriraj sve što počinje ispod ruba armature: to je naš vlastiti auto
    const float egoY = config_.egoLine * static_cast<float>(frameSize.height);
    if (static_cast<float>(det.box.y) >= egoY * 0.95F) {
        return risk;
    }

    // 1) Je li objekt u našoj traci? (preklapanje širine s ROI-jem na visini kotača)
    const cv::Point& bottomLeft  = roiPixels_[kBottomLeft];
    const cv::Point& bottomRight = roiPixels_[kBottomRight];
    const cv::Point& topRight    = roiPixels_[kTopRight];
    const cv::Point& topLeft     = roiPixels_[kTopLeft];

    const float groundY = static_cast<float>(det.box.y + det.box.height);
    if (groundY < static_cast<float>(std::min(topLeft.y, topRight.y))) {
        return risk;  // predaleko
    }
    const float y = std::min(groundY, static_cast<float>(std::max(bottomLeft.y, bottomRight.y)));

    const float laneLeft  = xAtY(bottomLeft, topLeft, y);
    const float laneRight = xAtY(bottomRight, topRight, y);
    const float boxLeft   = static_cast<float>(det.box.x);
    const float boxRight  = static_cast<float>(det.box.x + det.box.width);
    const float overlap   = std::max(0.0F, std::min(boxRight, laneRight) - std::max(boxLeft, laneLeft));

    risk.inRoi = overlap / static_cast<float>(det.box.width) >= config_.minLaneOverlap;
    if (!risk.inRoi) {
        return risk;
    }
    risk.level = RiskLevel::Caution;

    // 2) Vozilo koje brzo ide bočno prolazi pored nas (npr. iz suprotnog smjera).
    //    Pješake NE filtriramo: pješak koji prelazi cestu ispred nas JEST opasan.
    if (track != nullptr && !isVulnerable(det.cls)) {
        const float lateralRatio =
            std::abs(track->lateralRate) / static_cast<float>(frameSize.width);
        if (lateralRatio > config_.maxLateralRatio) {
            return risk;  // ostaje Caution
        }
    }

    // 3a) Glavni kriterij: vrijeme do sudara
    const bool ttcDanger = risk.ttc < config_.ttcWarnSeconds;

    // 3b) Rezervni kriterij: jako blizu po veličini boxa
    const bool isClose = isVulnerable(det.cls)
        ? static_cast<float>(det.box.height) / static_cast<float>(frameSize.height)
              >= config_.vruWarnHeightRatio
        : static_cast<float>(det.box.width) / static_cast<float>(frameSize.width)
              >= config_.vehicleWarnWidthRatio;

    if (ttcDanger || isClose) {
        risk.level = RiskLevel::Warning;
    }
    return risk;
}

void AdasLogic::evaluate(const std::vector<Detection>& detections,
                         const std::vector<Track>& tracks, cv::Size frameSize,
                         AdasResult& result) {
    updateRoi(frameSize);

    // Za svaku detekciju pronađi njen POTVRĐENI trag (ako postoji)
    trackForDetection_.assign(detections.size(), nullptr);
    for (const auto& t : tracks) {
        if (t.confirmed && t.detectionIndex >= 0) {
            trackForDetection_[static_cast<std::size_t>(t.detectionIndex)] = &t;
        }
    }

    result.perObject.clear();
    result.overall = RiskLevel::None;

    for (std::size_t i = 0; i < detections.size(); ++i) {
        const ObjectRisk r = assess(detections[i], trackForDetection_[i], frameSize);
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