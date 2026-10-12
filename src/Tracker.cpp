#include "adas/Tracker.hpp"

#include <algorithm>
#include <utility>

namespace adas {

namespace {

// IoU (Intersection over Union): koliko se dva boxa preklapaju, 0.0 - 1.0
float iou(const cv::Rect& a, const cv::Rect& b) noexcept {
    const int interArea = (a & b).area();
    if (interArea <= 0) {
        return 0.0F;
    }
    const int unionArea = a.area() + b.area() - interArea;
    return static_cast<float>(interArea) / static_cast<float>(unionArea);
}

// Centar boxa po x osi
float centerX(const cv::Rect& r) noexcept {
    return static_cast<float>(r.x) + 0.5F * static_cast<float>(r.width);
}

// Ažuriraj izglađenu širinu/centar i brzine traga na temelju novog boxa
void updateMotion(Track& t, const cv::Rect& newBox, double dtSeconds, float smoothing) noexcept {
    // Ako trag nije viđen nekoliko frameova, od zadnjeg viđenja prošlo je više vremena
    const float dt = static_cast<float>(dtSeconds) * static_cast<float>(t.missed + 1);
    if (dt <= 0.0F) {
        return;
    }
    const float a = smoothing;  // koliko "starog" zadržavamo (0.6 = 60%)

    // 1) Izgladi širinu i centar (EMA)
    const float prevWidth   = t.smoothedWidth;
    const float prevCenterX = t.smoothedCenterX;
    t.smoothedWidth   = a * prevWidth   + (1.0F - a) * static_cast<float>(newBox.width);
    t.smoothedCenterX = a * prevCenterX + (1.0F - a) * centerX(newBox);

    // 2) Trenutne brzine: promjena izglađenih vrijednosti po sekundi [px/s]
    const float instWidthRate   = (t.smoothedWidth - prevWidth) / dt;
    const float instLateralRate = (t.smoothedCenterX - prevCenterX) / dt;

    // 3) Izgladi i same brzine (derivacija pojačava šum)
    t.widthRate   = a * t.widthRate   + (1.0F - a) * instWidthRate;
    t.lateralRate = a * t.lateralRate + (1.0F - a) * instLateralRate;
}

}  // namespace

Tracker::Tracker(TrackerConfig config) : config_(std::move(config)) {
    tracks_.reserve(64);
    candidates_.reserve(256);
}

void Tracker::update(const std::vector<Detection>& detections, double dtSeconds) {
    // ---- 1) Priprema: nijedan trag još nema detekciju, nijedna detekcija nije zauzeta ----
    for (auto& t : tracks_) {
        t.detectionIndex = -1;
    }
    detectionUsed_.assign(detections.size(), 0);

    // ---- 2) Svi mogući parovi (trag, detekcija): ista klasa i dovoljno preklapanje ----
    candidates_.clear();
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
        for (std::size_t di = 0; di < detections.size(); ++di) {
            if (tracks_[ti].cls != detections[di].cls) {
                continue;
            }
            const float overlap = iou(tracks_[ti].box, detections[di].box);
            if (overlap >= config_.minIou) {
                candidates_.push_back(Match{ti, di, overlap});
            }
        }
    }

    // ---- 3) Pohlepno povezivanje: najbolji parovi prvi ----
    std::sort(candidates_.begin(), candidates_.end(),
              [](const Match& a, const Match& b) { return a.overlap > b.overlap; });

    for (const auto& m : candidates_) {
        Track& t = tracks_[m.track];
        if (t.detectionIndex != -1 || detectionUsed_[m.detection] != 0) {
            continue;  // trag ili detekcija su već povezani s nečim boljim
        }
        t.detectionIndex = static_cast<int>(m.detection);
        detectionUsed_[m.detection] = 1;
    }

    // ---- 4) Ažuriraj postojeće tragove ----
    for (auto& t : tracks_) {
        if (t.detectionIndex >= 0) {
            const Detection& det = detections[static_cast<std::size_t>(t.detectionIndex)];
            updateMotion(t, det.box, dtSeconds, config_.smoothing);  // brzine (T3)
            t.box = det.box;
            ++t.hits;
            t.missed = 0;
            if (t.hits >= config_.minHitsToConfirm) {
                t.confirmed = true;
            }
        } else {
            ++t.missed;
        }
    }

    // ---- 5) Obriši tragove koji predugo nisu viđeni ----
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [this](const Track& t) {
                                     return t.missed > config_.maxMissedFrames;
                                 }),
                  tracks_.end());

    // ---- 6) Detekcije koje nisu pripale nijednom tragu -> novi tragovi ----
    for (std::size_t di = 0; di < detections.size(); ++di) {
        if (detectionUsed_[di] != 0) {
            continue;
        }
        Track t;
        t.id = nextId_++;
        t.cls = detections[di].cls;
        t.box = detections[di].box;
        t.smoothedWidth = static_cast<float>(t.box.width);
        t.smoothedCenterX = centerX(t.box);
        t.hits = 1;
        t.confirmed = (config_.minHitsToConfirm <= 1);
        t.detectionIndex = static_cast<int>(di);
        tracks_.push_back(t);
    }
}

}  // namespace adas