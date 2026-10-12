#pragma once

#include "adas/Types.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <vector>

namespace adas {

struct AdasConfig {
    // ROI trapez u normaliziranim koordinatama (0.0 - 1.0 od širine/visine framea)
    // redom: dolje-lijevo, dolje-desno, gore-desno, gore-lijevo
    // Kalibrirano za kameru gdje se cesta vidi između ~55% i ~70% visine slike
        std::array<cv::Point2f, 4> roi{{
        {0.33F, 0.69F}, {0.76F, 0.69F}, {0.60F, 0.56F}, {0.48F, 0.55F}
    }};

    float egoLine{0.69F};

    float minLaneOverlap{0.5F};  // barem 50% širine objekta mora biti u nasoj traci

    float vehicleWarnWidthRatio{0.22F};  // vozilo šire od 22% framea -> blizu
    float vruWarnHeightRatio{0.20F};     // pješak/biciklist viši od 20% framea -> blizu

    int framesToActivate{3};             // uzastopni frameovi rizika prije upozorenja
    int framesToHold{10};                // upozorenje ostaje još toliko frameova
};

// Procjena za jedan objekt
struct ObjectRisk {
    RiskLevel level{RiskLevel::None};
    bool inRoi{false};
};

// Rezultat za cijeli frame
struct AdasResult {
    RiskLevel overall{RiskLevel::None};  // najveći rizik u ovom frameu
    bool warningActive{false};           // stabilizirano upozorenje (za prikaz)
    std::vector<ObjectRisk> perObject;   // isti redoslijed kao detekcije
};

class AdasLogic {
public:
    explicit AdasLogic(AdasConfig config = {});

    void evaluate(const std::vector<Detection>& detections, cv::Size frameSize,
                  AdasResult& result);

    // ROI u pikselima (za crtanje)
    [[nodiscard]] const std::vector<cv::Point>& roiPolygon() const noexcept { return roiPixels_; }

private:
    [[nodiscard]] ObjectRisk assess(const Detection& det, cv::Size frameSize) const;
    void updateRoi(cv::Size frameSize);

    AdasConfig config_;
    std::vector<cv::Point> roiPixels_;
    cv::Size cachedSize_{};
    int riskStreak_{0};
    int holdCounter_{0};
};

}  // namespace adas