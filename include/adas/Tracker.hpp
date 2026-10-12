#pragma once

#include "adas/Types.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <limits>
#include <vector>

namespace adas {

struct TrackerConfig {
    float minIou{0.30F};       // minimalno preklapanje da detekcija pripada tragu
    int minHitsToConfirm{3};   // koliko puta trag mora biti viđen da postane potvrđen
    int maxMissedFrames{5};    // koliko frameova trag smije "nestati" prije brisanja
    float smoothing{0.6F};     // EMA faktor (0-1): veće = glađe, ali sporije reagira
};

// Jedan praćeni objekt kroz više frameova
struct Track {
    std::uint32_t id{0};               // jedinstveni broj traga
    ObjectClass cls{ObjectClass::Car};
    cv::Rect box;                      // zadnji poznati box

    float smoothedWidth{0.0F};         // izglađena širina boxa [px]
    float smoothedCenterX{0.0F};       // izglađeni centar po x [px]
    float widthRate{0.0F};             // brzina promjene širine [px/s], + = približava se
    float lateralRate{0.0F};           // bočna brzina centra [px/s], + = udesno

    int hits{0};                       // koliko puta je viđen ukupno
    int missed{0};                     // koliko frameova zaredom NIJE viđen
    bool confirmed{false};             // viđen dovoljno puta da mu vjerujemo

    int detectionIndex{-1};            // koja detekcija u OVOM frameu mu pripada (-1 = nijedna)
};

class Tracker {
public:
    explicit Tracker(TrackerConfig config = {});

    // Poveži nove detekcije s postojećim tragovima.
    // dtSeconds = vrijeme od prošlog framea (za video: 1 / fps)
    void update(const std::vector<Detection>& detections, double dtSeconds);

    [[nodiscard]] const std::vector<Track>& tracks() const noexcept { return tracks_; }

private:
    TrackerConfig config_;
    std::vector<Track> tracks_;
    std::uint32_t nextId_{1};

    // Prealocirani bufferi za povezivanje (bez alokacija u petlji)
    std::vector<std::uint8_t> detectionUsed_;
};

// TTC u sekundama; beskonačno ako se objekt ne približava
[[nodiscard]] inline float timeToCollision(const Track& t) noexcept {
    constexpr float kMinRate = 1.0F;  // ispod 1 px/s smatramo da ne raste (šum)
    if (t.widthRate < kMinRate) {
        return std::numeric_limits<float>::infinity();
    }
    return t.smoothedWidth / t.widthRate;
}

}  