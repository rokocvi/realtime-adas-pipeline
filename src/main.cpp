#include "adas/AdasLogic.hpp"
#include "adas/Detector.hpp"
#include "adas/FrameGrabber.hpp"
#include "adas/Timer.hpp"
#include "adas/Types.hpp"

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
        const cv::Scalar color = colorFor(result.perObject[i].level);

        cv::rectangle(frame, det.box, color, static_cast<int>(2 * s));

        const std::string label = cv::format("%s %.2f", adas::toString(det.cls).data(),
                                             static_cast<double>(det.confidence));
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

}  // namespace

// =====================================================================
int main(int argc, char** argv) {
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
            logic.evaluate(detections, frame.size(), result);
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