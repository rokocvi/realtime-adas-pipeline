#pragma once

#include "adas/Tracker.hpp"
#include "adas/Types.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace adas {

struct AdasConfig {
    // ROI trapez u normaliziranim koordinatama (0.0 - 1.0 od širine/visine framea)
    // REDOSLIJED JE BITAN: dolje-lijevo, dolje-desno, gore-desno, gore-lijevo
    // Kalibrirano s: adas_pipeline.exe --calibrate
    std::array<cv::Point2f, 4> roi{{
        {0.33F, 0.69F}, {0.76F, 0.69F}, {0.60F, 0.56F}, {0.48F, 0.55F}
    }};

    float egoLine{0.69F};                // ispod ove linije je armatura/hauba našeg auta
    float minLaneOverlap{0.50F};         // barem 50% širine objekta mora biti u našoj traci

    // Glavni kriterij: vrijeme do sudara
    float ttcWarnSeconds{2.5F};          // upozori ako je TTC manji od ovoga
    float maxLateralRatio{0.15F};        // vozilo koje se bočno kreće brže od 15% širine
                                         // framea u sekundi prolazi pored nas
    float pathHalfWidthMeters{1.5F};     // objekt dalje od 1.5 m bočno nije na našoj putanji

    // Rezervni kriterij: jako blizu (npr. stojimo iza auta, TTC = beskonačno)
    float vehicleWarnWidthRatio{0.22F};
    float vruWarnHeightRatio{0.20F};

    int framesToActivate{3};
    int framesToHold{10};
};

// Procjena za jedan objekt
struct ObjectRisk {
    RiskLevel level{RiskLevel::None};
    bool inRoi{false};
    std::uint32_t trackId{0};                                 // 0 = nema potvrđenog traga
    float ttc{std::numeric_limits<float>::infinity()};       // sekunde do sudara
    float lateralOffsetM{0.0F};                               // bočni pomak od naše putanje [m]
};

// Rezultat za cijeli frame
struct AdasResult {
    RiskLevel overall{RiskLevel::None};
    bool warningActive{false};
    std::vector<ObjectRisk> perObject;   // isti redoslijed kao detekcije
};

class AdasLogic {
public:
    explicit AdasLogic(AdasConfig config = {});

    // tracks MORAJU biti ažurirani s ISTIM detections u ovom frameu
    void evaluate(const std::vector<Detection>& detections, const std::vector<Track>& tracks,
                  cv::Size frameSize, AdasResult& result);

    [[nodiscard]] const std::vector<cv::Point>& roiPolygon() const noexcept { return roiPixels_; }

private:
    [[nodiscard]] ObjectRisk assess(const Detection& det, const Track* track,
                                    cv::Size frameSize) const;
    void updateRoi(cv::Size frameSize);

    AdasConfig config_;
    std::vector<cv::Point> roiPixels_;
    cv::Size cachedSize_{};

    // Za svaku detekciju: pokazivač na njen potvrđeni trag (nullptr = nema)
    std::vector<const Track*> trackForDetection_;

    int riskStreak_{0};
    int holdCounter_{0};
};

}  // namespace adas