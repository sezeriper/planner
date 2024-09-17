#pragma once

#include "path_tracker.hpp"
#include "video_recorder.hpp"
#include <chrono>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <ratio>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <thread>

using namespace std;
using namespace cv;
using namespace cv::dnn;

namespace rota {
class uav_detector {
    static constexpr float CONFIDENCE_THRESHOLD = 0.5f;
 
    std::vector<std::string> classes;

public:
    uav_detector(video_recorder& vid, plane_control& plane) :
        _plane(plane),
        _vid(vid),
        _net(readNet("resources/rota_yolov10m.onnx")),
        _should_detect(false),
        _locked(false),
        _lock_start(0),
        _lock_end(0),
        _thread([this](std::stop_token stoken) { thread_func(stoken); })
    {}

    ~uav_detector() {
        _thread.request_stop();
    }

    void start() { _should_detect = true; }
    void end() { _should_detect = false; }

    void on_read(std::function<void(std::uint64_t, std::uint64_t)> callback) {
        _callback = callback;
    }

private:
    plane_control& _plane;
    video_recorder& _vid;
    Net _net;
    std::atomic_bool _should_detect;
    bool _locked;
    std::uint64_t _lock_start;
    std::uint64_t _lock_end;
    std::function<void(std::uint64_t, std::uint64_t)> _callback;
    std::jthread _thread;

    void thread_func(std::stop_token& stoken) {
        if (stoken.stop_requested()) return;

        while (!stoken.stop_requested()) {
            if (_should_detect) {
                if(!read()) _thread.request_stop();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(34));
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

    void process_frame(cv::Mat image) {
        Size size(640, 640);

        resize(image, image, size);
        Image2BlobParams params{};
        params.scalefactor = 1.0 / 255.0;
        params.size = size;
        params.swapRB = true;

        auto inp = blobFromImageWithParams(image, params);

        std::vector<Mat> outs{};
        std::vector<int> keep_classIds{};
        std::vector<float> keep_confidences{};
        std::vector<Rect2d> keep_boxes{};
        std::vector<Rect> boxes{};

        _net.setInput(inp);
        _net.forward(outs, _net.getUnconnectedOutLayersNames());

        yoloPostProcessing(outs, keep_classIds, keep_confidences, keep_boxes, CONFIDENCE_THRESHOLD, 0.4, "yolov10", 1);

        if (keep_boxes.empty()) {
            if (_locked) {
                _locked = false;
                _lock_end = _plane.get_unix_epoch_time();
                spdlog::info("Locked for {} seconds.", static_cast<double>(_lock_end - _lock_start) / 1000000.0);
                if (_callback) _callback(_lock_start, _lock_end);
            }
            return;
        }

        if (_locked == false)  {
            _lock_start = _plane.get_unix_epoch_time();
            _locked = true;
        }


        for (auto box : keep_boxes)
        {
            boxes.push_back(Rect(cvFloor(box.x), cvFloor(box.y), cvFloor(box.width - box.x), cvFloor(box.height - box.y)));
        }

        params.blobRectsToImageRects(boxes, boxes, image.size());

        for (size_t idx = 0; idx < boxes.size(); ++idx)
        {
            Rect box = boxes[idx];
            drawPrediction(keep_classIds[idx], keep_confidences[idx], box.x, box.y,
                    box.width + box.x, box.height + box.y, image);
        }

        // imshow("UAV Detector", image);
        // if (waitKey(10) == 27) {
        //     return;
        // }
    }

void drawPrediction(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

    std::string label = std::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ": " + label;
    }

    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - labelSize.height),
              Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}

void yoloPostProcessing(
    std::vector<Mat>& outs,
    std::vector<int>& keep_classIds,
    std::vector<float>& keep_confidences,
    std::vector<Rect2d>& keep_boxes,
    float conf_threshold,
    float iou_threshold,
    const std::string& model_name,
    const int nc=80)
{

    // Retrieve
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<Rect2d> boxes;

    if (model_name == "yolov8" || model_name == "yolov10" ||
        model_name == "yolov9")
    {
        cv::transposeND(outs[0], {0, 2, 1}, outs[0]);
    }

    if (model_name == "yolonas")
    {
        // outs contains 2 elemets of shape [1, 8400, 80] and [1, 8400, 4]. Concat them to get [1, 8400, 84]
        Mat concat_out;
        // squeeze the first dimension
        outs[0] = outs[0].reshape(1, outs[0].size[1]);
        outs[1] = outs[1].reshape(1, outs[1].size[1]);
        cv::hconcat(outs[1], outs[0], concat_out);
        outs[0] = concat_out;
        // remove the second element
        outs.pop_back();
        // unsqueeze the first dimension
        outs[0] = outs[0].reshape(0, std::vector<int>{1, 8400, nc + 4});
    }

    // assert if last dim is 85 or 84
    // CV_CheckEQ(outs[0].dims, 3, "Invalid output shape. The shape should be [1, #anchors, 85 or 84]");
    // CV_CheckEQ((outs[0].size[2] == nc + 5 || outs[0].size[2] == 80 + 4), true, "Invalid output shape: ");

    for (auto preds : outs)
    {
        preds = preds.reshape(1, preds.size[1]); // [1, 8400, 85] -> [8400, 85]
        for (int i = 0; i < preds.rows; ++i)
        {
            // filter out non object
            float obj_conf = (model_name == "yolov8" || model_name == "yolonas" ||
                              model_name == "yolov9" || model_name == "yolov10") ? 1.0f : preds.at<float>(i, 4) ;
            if (obj_conf < conf_threshold)
                continue;

            Mat scores = preds.row(i).colRange((model_name == "yolov8" || model_name == "yolonas" || model_name == "yolov9" || model_name == "yolov10") ? 4 : 5, preds.cols);
            double conf;
            Point maxLoc;
            minMaxLoc(scores, 0, &conf, 0, &maxLoc);

            conf = (model_name == "yolov8" || model_name == "yolonas" || model_name == "yolov9" || model_name == "yolov10") ? conf : conf * obj_conf;
            if (conf < conf_threshold)
                continue;

            // get bbox coords
            float* det = preds.ptr<float>(i);
            double cx = det[0];
            double cy = det[1];
            double w = det[2];
            double h = det[3];

            // [x1, y1, x2, y2]
            if (model_name == "yolonas" || model_name == "yolov10"){
                boxes.push_back(Rect2d(cx, cy, w, h));
            } else {
                boxes.push_back(Rect2d(cx - 0.5 * w, cy - 0.5 * h,
                                        cx + 0.5 * w, cy + 0.5 * h));
            }
            classIds.push_back(maxLoc.x);
            confidences.push_back(static_cast<float>(conf));
        }
    }

    // NMS
    std::vector<int> keep_idx;
    NMSBoxes(boxes, confidences, conf_threshold, iou_threshold, keep_idx);

    for (auto i : keep_idx)
    {
        keep_classIds.push_back(classIds[i]);
        keep_confidences.push_back(confidences[i]);
        keep_boxes.push_back(boxes[i]);
    }
}
};
}