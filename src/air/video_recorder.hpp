#pragma once

#include "../utils.hpp"
#include "path_tracker.hpp"

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
namespace rota {
class video_recorder 
{
public:
    video_recorder(plane_control& pc) :
        _pc(pc),
        _cap(gstreamer_pipeline(), cv::CAP_GSTREAMER),
        _writer(std::to_string(time(nullptr)) + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(640, 480)),
        _thread([this](std::stop_token stoken) { thread_func(stoken); } )
    {
    }

    cv::Mat get_frame() {
        return _frame.read([](const cv::Mat& frame) {
            return frame;
        });
    }

    ~video_recorder() {
        _writer.release();
        _cap.release();
        _thread.request_stop();
    }

private:
    plane_control& pc;
    cv::VideoCapture _cap;
    cv::VideoWriter _writer;

    mutex_guarded<cv::Mat> _frame;

    std::jthread _thread;

    std::string gstreamer_pipeline() {
        return
            "nvarguscamerasrc !"
            "video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,"
            "format=(string)NV12, framerate=(fraction)30/1 ! "
            "nvvidconv flip-method={flip_method} ! "
            "video/x-raw, width=(int)640, height=(int)640, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }

    void thread_func(std::stop_token& stoken) {
        if (stoken.stop_requested()) return;

        _cap.open(0);
        if (!_cap.isOpened()) {
            spdlog::error("Failed to open camera.");
        }

        while (!stoken.stop_requested()) {
            cv::Mat frame;
            _cap.read(frame);
            if (frame.empty()) {
                spdlog::error("QR Reader: Blank frame grabbed.");
                return;
            }

            _frame.write([frame](cv::Mat& f) {
                f = frame;
            });

            auto t = unix_to_time(pc.get_unix_epoch_time());

            auto str = std::to_string(t.hour) + ':' + std::to_string(t.minute) + ':' + std::to_string(t.second) + ':' + std::to_string(t.millisecond);

            cv::putText(frame, str, cv::Point(10, 100), );
            _writer.write(frame);
        }
    }
    struct time_t {
        // int day;
        int hour;
        int minute;
        int second;
        int millisecond;
    };

    static time_t unix_to_time(std::uint64_t unix_time) {
        std::time_t t = unix_time / 1000000ull;
        auto date = *std::gmtime(&t);

        time_t time;
        // time.day = date.tm_mday;
        time.hour = date.tm_hour;
        time.minute = date.tm_min;
        time.second = date.tm_sec;
        time.millisecond = (unix_time / 1000ull) % 1000ull;

        return time;
    }
};
}