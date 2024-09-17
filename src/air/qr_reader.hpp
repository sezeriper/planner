#pragma once

#include "video_recorder.hpp"
#include <chrono>
#include <functional>
#include <opencv2/core/cvstd_wrapper.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/wechat_qrcode.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <thread>


namespace rota {
class qr_reader {
public:
    qr_reader(video_recorder& vid) :
        _vid(vid),
        detector(
            cv::makePtr<cv::wechat_qrcode::WeChatQRCode>(cv::wechat_qrcode::WeChatQRCode(
                "resources/wechat_qrcode/detect.prototxt",
                "resources/wechat_qrcode/detect.caffemodel",
                "resources/wechat_qrcode/sr.prototxt",
                "resources/wechat_qrcode/sr.caffemodel"
            ))
        ),
        _should_detect(false),
        _thread([this](std::stop_token stoken) { thread_func(stoken); } ) {
    }

    ~qr_reader() {
        _thread.request_stop();
    }

    void start() { _should_detect = true; }
    void end() { _should_detect = false; }

    void on_read(std::function<void(const std::string&)> callback) {
        _callback = callback;
    }

private:
    video_recorder& _vid;
    cv::Ptr<cv::wechat_qrcode::WeChatQRCode> detector;
    std::atomic_bool _should_detect;
    std::jthread _thread;
    std::function<void(const std::string&)> _callback;

    void thread_func(std::stop_token& stoken) {
        if (stoken.stop_requested()) return;

        while (!stoken.stop_requested()) {
            if (_should_detect) {
                if(!read()) _thread.request_stop();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(34));
        }
    }

    void draw_bbox(cv::Mat &im, const cv::Mat &bbox) { int n = bbox.rows;
        for (int i = 0; i < n; i++)
        {
                line(im, cv::Point2i(bbox.at<float>(i,0), bbox.at<float>(i,1)), 
                cv::Point2i( bbox.at<float>((i+1) % n,0), 
                        bbox.at<float>((i+1) % n,1)), cv::Scalar(0,255,0), 3);
        }
    }

    bool read() {
        cv::Mat frame = _vid.get_frame();
        if (frame.empty()) {
            spdlog::error("QR Reader: Blank frame grabbed.");
            return false;
        }

        process_frame(frame);

        return true;
    }

    void process_frame(cv::Mat& frame) {
        spdlog::stopwatch sw;
        std::vector<cv::Mat> points;
        auto result = detector->detectAndDecode(frame, points);
        spdlog::info("Reading takes {}ms", sw.elapsed_ms().count());

        if (result.empty()) {
            spdlog::info("QR Reader: No QR code detected.");
            return;
        }

        for (const auto& value : result) {
            spdlog::info("QR Reader: QR code detected: {}", value);
            if (_callback) {
                _callback(value);
                _callback = nullptr;
            }
        }

        sw.reset();
        cv::Mat1f bbox;
        for (int i = 0; i < points[0].size().height; ++i) {
            bbox.push_back(points[0].row(i));
        }
        spdlog::info("Processing takes {}ms", sw.elapsed_ms().count());


        draw_bbox(frame, bbox);
    }
};
}