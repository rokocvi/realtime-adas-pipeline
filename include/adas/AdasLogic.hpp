#pragma once

#include "adas/Types.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <vector>

namespace adas {

struct AdasConfig {
    // ROI trapez u normaliziranim koordinatama (0.0 - 1.0 od širine/visine framea)
    // redom: dolje-lijevo, dolje-desno, gore-desno, gore-lijevo
    std::array<cv::Point2f, 4> roi{{
        {0.20F, 1.00F}, {0.80F, 1.00F}, {0.56F, 0.62F}, {0.44F, 0.62F}
    }};

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