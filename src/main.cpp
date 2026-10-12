#include "adas/AdasLogic.hpp"
#include "adas/Detector.hpp"
#include "adas/FrameGrabber.hpp"
#include "adas/Tracker.hpp"
#include "adas/Timer.hpp"
#include "adas/Types.hpp"
#include <array>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

namespace {

constexpr const char* kWindowName = "Embedded Real-Time ADAS Pipeline";
constexpr int kEscKey = 27;

const cv::Scalar kGreen{0, 200, 0};      // OpenCV boje su BGR, ne RGB!
const cv::Scalar kYellow{0, 220, 255};
const cv::Scalar kRed{0, 0, 255};
const cv::Scalar kWhite{255, 255, 255};
const cv::Scalar kBlack{0, 0, 0};

// Vremena svih faza jednog framea
struct FrameTimings {
    double grabMs{0.0};
    double preMs{0.0};
    double inferMs{0.0};
    double postMs{0.0};
    double logicMs{0.0};
    double totalMs{0.0};
};

// Skupljanje latencija za statistiku na kraju
class LatencyStats {
public:
    explicit LatencyStats(std::size_t expectedFrames) { samples_.reserve(expectedFrames); }

    void add(double ms) { samples_.push_back(ms); }

    void printSummary() {
        if (samples_.empty()) {
            return;
        }
        std::sort(samples_.begin(), samples_.end());
        const double sum = std::accumulate(samples_.begin(), samples_.end(), 0.0);
        const auto p95Index =
            static_cast<std::size_t>(0.95 * static_cast<double>(samples_.size() - 1));

        std::cout << std::fixed << std::setprecision(1)
                  << "Latency [ms]  avg " << sum / static_cast<double>(samples_.size())
                  << " | min " << samples_.front()
                  << " | max " << samples_.back()
                  << " | p95 " << samples_[p95Index] << '\n';
    }

private:
    std::vector<double> samples_;
};

// ---------------- Crtanje ----------------

// Veličina teksta/linija prilagođena rezoluciji (da je čitljivo i na 1080p)
double uiScale(const cv::Mat& frame) {
    return std::max(1.0, static_cast<double>(frame.cols) / 1280.0);
}

cv::Scalar colorFor(adas::RiskLevel level) {
    switch (level) {
        case adas::RiskLevel::Warning: return kRed;
        case adas::RiskLevel::Caution: return kYellow;
        case adas::RiskLevel::None:    break;
    }
    return kGreen;
}

void drawRoi(cv::Mat& frame, const std::vector<cv::Point>& roi) {
    const double s = uiScale(frame);
    cv::polylines(frame, roi, /*isClosed=*/true, kWhite, static_cast<int>(2 * s), cv::LINE_AA);
}

void drawDetections(cv::Mat& frame, const std::vector<adas::Detection>& detections,
                    const adas::AdasResult& result) {
    const double s = uiScale(frame);
    for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];
        const auto& risk = result.perObject[i];
        const cv::Scalar color = colorFor(risk.level);

        cv::rectangle(frame, det.box, color, static_cast<int>(2 * s));

        std::string label = cv::format("%s %.2f", adas::toString(det.cls).data(),
                                       static_cast<double>(det.confidence));
        if (risk.trackId != 0) {
            label += cv::format(" #%u", static_cast<unsigned>(risk.trackId));
            if (std::isfinite(risk.ttc) && risk.ttc < 10.0F) {
                if(risk.inRoi){
                    label += cv::format(" (%.2fm)", static_cast<double>(risk.lateralOffsetM));
                }
                label += cv::format(" TTC %.1fs", static_cast<double>(risk.ttc));
            }
        }

        const cv::Point org(det.box.x, std::max(det.box.y - 6, static_cast<int>(16 * s)));
        cv::putText(frame, label, org, cv::FONT_HERSHEY_SIMPLEX, 0.5 * s, color,
                    static_cast<int>(2 * s));
    }
}

void drawHud(cv::Mat& frame, const FrameTimings& t, double fps, int frameIdx) {
    const double s = uiScale(frame);
    const auto px = [s](int v) { return static_cast<int>(v * s); };
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    cv::rectangle(frame, cv::Rect(px(10), px(10), px(470), px(95)), kBlack, cv::FILLED);

    cv::putText(frame, cv::format("Pipeline latency: %.1f ms", t.totalMs),
                cv::Point(px(20), px(38)), font, 0.7 * s, kWhite, std::max(1, px(2)));
    cv::putText(frame,
                cv::format("grab %.1f | pre %.1f | infer %.1f | post %.1f | logic %.2f",
                           t.grabMs, t.preMs, t.inferMs, t.postMs, t.logicMs),
                cv::Point(px(20), px(65)), font, 0.48 * s, kWhite, std::max(1, px(1)));
    cv::putText(frame, cv::format("FPS: %.1f   Frame: %d", fps, frameIdx),
                cv::Point(px(20), px(92)), font, 0.6 * s, kWhite, std::max(1, px(1)));
}

void drawWarning(cv::Mat& frame) {
    const std::string text = "WARNING: COLLISION RISK";
    const double s = uiScale(frame);
    const double fontScale = 1.2 * s;
    const int thickness = static_cast<int>(3 * s);
    const int pad = static_cast<int>(12 * s);

    int baseline = 0;
    const cv::Size ts =
        cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, fontScale, thickness, &baseline);
    const cv::Point org((frame.cols - ts.width) / 2, static_cast<int>(160 * s));

    cv::rectangle(frame,
                  cv::Rect(org.x - pad, org.y - ts.height - pad, ts.width + 2 * pad,
                           ts.height + baseline + 2 * pad),
                  kRed, cv::FILLED);
    cv::putText(frame, text, org, cv::FONT_HERSHEY_DUPLEX, fontScale, kWhite, thickness,
                cv::LINE_AA);
}

// ---------------- Kalibracija ROI-ja (klikanjem mišem) ----------------

constexpr const char* kCalibWindowName = "ROI calibration";

struct CalibrationState {
    std::vector<cv::Point> clicks;
};

// OpenCV poziva ovu funkciju na svaki događaj mišem u prozoru
void onCalibrationMouse(int event, int x, int y, int /*flags*/, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }
    auto* state = static_cast<CalibrationState*>(userdata);
    if (state->clicks.size() < 4) {
        state->clicks.emplace_back(x, y);
    }
}

int runCalibration(const std::string& source) {
    adas::FrameGrabber grabber(source);
    cv::Mat frame;
    if (!grabber.grab(frame)) {
        std::cerr << "Cannot read first frame\n";
        return 1;
    }

    CalibrationState state;
    cv::namedWindow(kCalibWindowName, cv::WINDOW_NORMAL);
    cv::setMouseCallback(kCalibWindowName, onCalibrationMouse, &state);

    const std::array<const char*, 4> names{
        "BOTTOM-LEFT of your lane", "BOTTOM-RIGHT of your lane",
        "TOP-RIGHT of your lane", "TOP-LEFT of your lane"};

    const double s = uiScale(frame);
    while (state.clicks.size() < 4) {
        cv::Mat view = frame.clone();  // samo za kalibraciju, nije u glavnoj petlji
        for (const auto& p : state.clicks) {
            cv::circle(view, p, static_cast<int>(8 * s), kRed, cv::FILLED);
        }
        if (state.clicks.size() >= 2) {
            cv::polylines(view, state.clicks, false, kYellow, static_cast<int>(3 * s));
        }
        cv::putText(view,
                    cv::format("Click %s (%d/4)   ESC = cancel",
                               names[state.clicks.size()],
                               static_cast<int>(state.clicks.size()) + 1),
                    cv::Point(static_cast<int>(30 * s), static_cast<int>(60 * s)),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0 * s, kYellow, static_cast<int>(2 * s));
        cv::imshow(kCalibWindowName, view);
        if ((cv::waitKey(30) & 0xFF) == kEscKey) {
            return 1;
        }
    }
    cv::destroyAllWindows();

    // Ispiši rezultat kao C++ kod za AdasConfig
    const double w = static_cast<double>(frame.cols);
    const double h = static_cast<double>(frame.rows);
    std::cout << std::fixed << std::setprecision(2)
              << "\nPaste this into AdasConfig in AdasLogic.hpp:\n\n"
              << "    std::array<cv::Point2f, 4> roi{{\n        ";
    for (std::size_t i = 0; i < state.clicks.size(); ++i) {
        std::cout << '{' << state.clicks[i].x / w << "F, " << state.clicks[i].y / h << "F}"
                  << (i + 1 < state.clicks.size() ? ", " : "");
    }
    const double egoLine =
        std::max(state.clicks[0].y, state.clicks[1].y) / h;
    std::cout << "\n    }};\n\n"
              << "    float egoLine{" << egoLine << "F};\n";
    return 0;
}

}  // namespace

// =====================================================================
int main(int argc, char** argv) {
        // Način rada za kalibraciju: adas_pipeline.exe --calibrate [video]
    if (argc > 1 && std::string(argv[1]) == "--calibrate") {
        const std::string calibSource = argc > 2 ? argv[2] : "videos/dashcam.mp4";
        try {
            return runCalibration(calibSource);
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << '\n';
            return 1;
        }
    }

    const std::string modelPath = argc > 1 ? argv[1] : "models/yolov8n.onnx";
    const std::string source    = argc > 2 ? argv[2] : "videos/dashcam.mp4";

    try {
        // ---- 1) Inicijalizacija (sve alokacije se događaju ovdje) ----
        adas::FrameGrabber grabber(source);
        std::cout << "Video: " << grabber.frameSize() << " @ " << grabber.fps() << " FPS\n";

        adas::DetectorConfig detectorConfig;
        detectorConfig.modelPath = modelPath;
        adas::Detector detector(std::move(detectorConfig));
        adas::AdasLogic logic;
        adas::Tracker tracker;

        // Vrijeme između frameova u VIDEU (ne vrijeme obrade!)
        const double videoFps = grabber.fps() > 0.0 ? grabber.fps() : 30.0;
        const double frameDt  = 1.0 / videoFps;
        std::cout << "Model loaded: " << modelPath << '\n';

        // ---- 2) Warm-up: prvi prolazi kroz model su sporiji, ne mjerimo ih ----
        {
            const cv::Mat dummy(grabber.frameSize(), CV_8UC3, cv::Scalar::all(0));
            std::vector<adas::Detection> tmp;
            for (int i = 0; i < 3; ++i) {
                detector.detect(dummy, tmp);
            }
        }

        // ---- 3) Prealokacija za glavnu petlju ----
        cv::Mat frame;
        std::vector<adas::Detection> detections;
        detections.reserve(64);
        adas::AdasResult result;
        result.perObject.reserve(64);
        LatencyStats stats(10000);

        cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);

        adas::Timer loopTimer;
        double fps = 0.0;
        int frameIdx = 0;
        bool wasWarning = false;

        // ---- 4) Glavna petlja ----
        while (true) {
            FrameTimings t;
            adas::Timer pipelineTimer;

            if (!grabber.grab(frame)) {
                break;  // kraj videa
            }
            t.grabMs = pipelineTimer.elapsedMs();

            detector.detect(frame, detections);
            const auto& dt = detector.lastTimings();
            t.preMs = dt.preprocessMs;
            t.inferMs = dt.inferenceMs;
            t.postMs = dt.postprocessMs;

            adas::Timer logicTimer;
            tracker.update(detections, frameDt);
            logic.evaluate(detections, tracker.tracks(), frame.size(), result);
            t.logicMs = logicTimer.elapsedMs();

            t.totalMs = pipelineTimer.elapsedMs();
            stats.add(t.totalMs);

            // Ispis u konzolu samo kad upozorenje PROMIJENI stanje (ne svaki frame)
            if (result.warningActive && !wasWarning) {
                std::cout << "[frame " << frameIdx << "] WARNING: COLLISION RISK\n";
            }
            wasWarning = result.warningActive;

            // ---- Vizualizacija (nije dio mjerene latencije) ----
            drawRoi(frame, logic.roiPolygon());
            drawDetections(frame, detections, result);
            drawHud(frame, t, fps, frameIdx);
            if (result.warningActive) {
                drawWarning(frame);
            }

            cv::imshow(kWindowName, frame);
            const int key = cv::waitKey(1);
            if ((key & 0xFF) == kEscKey ||
                cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE) < 1.0) {
                break;  // ESC ili zatvoren prozor
            }

            // FPS cijele petlje, izglađen da ne skače
            const double loopMs = loopTimer.elapsedMs();
            loopTimer.reset();
            if (loopMs > 0.0) {
                const double instantFps = 1000.0 / loopMs;
                fps = (fps == 0.0) ? instantFps : 0.9 * fps + 0.1 * instantFps;
            }
            ++frameIdx;
        }

        cv::destroyAllWindows();
        std::cout << "Processed frames: " << frameIdx << '\n';
        stats.printSummary();
        return 0;

    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime error: " << e.what() << '\n';
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << '\n';
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
    }
    return 1;
}