#include <chrono>
#include <string>
#include <thread>
#include <torch/script.h>

#include "helpers.h"
#include "i3d.h"
#include "io.h"
#include "kinect.h"
#include "yolov5.h"
#include "macros.hpp"

void clusterRegion(std::shared_ptr<I3d>& sptr_i3d)
{
    int minPoints = 4;
    const float epsilon = 3.170;
    sptr_i3d->clusterRegion(epsilon, minPoints, sptr_i3d);
}

void segmentRegion(std::shared_ptr<I3d>& sptr_i3d)
{
    sptr_i3d->segmentRegion(sptr_i3d);
}

void proposeRegion(std::shared_ptr<I3d>& sptr_i3d)
{
    sptr_i3d->proposeRegion(sptr_i3d);
}

void buildPcl(std::shared_ptr<I3d>& sptr_i3d)
{
    sptr_i3d->buildPCloud(sptr_i3d);
}

void k4aCapture(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<I3d>& sptr_i3d)
{
    START
    sptr_kinect->capture();
    sptr_kinect->depthCapture();
    int w = k4a_image_get_width_pixels(sptr_kinect->m_depth);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_depth);

    while (sptr_i3d->isRun()) {
        START_TIMER
        sptr_kinect->capture();
        sptr_kinect->depthCapture();
        sptr_kinect->pclCapture();
        sptr_kinect->imgCapture();
        sptr_kinect->c2dCapture();
        sptr_kinect->transform(RGB_TO_DEPTH);

        auto* ptr_k4aImgData = k4a_image_get_buffer(sptr_kinect->m_c2d);
        auto* ptr_k4aPCloudData
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_pcl);
        auto* ptr_k4aDepthData
            = (uint16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_depth);
        auto* ptr_k4aTableData
            = (k4a_float2_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_xyT);

        // share k4a resources with intact
        sptr_i3d->setDepthWidth(w);
        sptr_i3d->setDepthHeight(h);
        sptr_i3d->setSensorImgData(ptr_k4aImgData);
        sptr_i3d->setSensorTableData(ptr_k4aTableData);
        sptr_i3d->setSensorDepthData(ptr_k4aDepthData);
        sptr_i3d->setSensorPCloudData(ptr_k4aPCloudData);

        // release k4a resources
        sptr_kinect->releaseK4aImages();
        sptr_kinect->releaseK4aCapture();
        RAISE_SENSOR_RESOURCES_READY_FLAG
        EXIT_CALLBACK
        STOP_TIMER(" k4a driver thread: runtime @ ")
    }
}

int main(int argc, char* argv[])
{
    logger(argc, argv);
    LOG(INFO) << "-- 3DINTACT is currently unstable and should only be used "
                 "for academic purposes!";
    LOG(INFO) << "-- press ESC to exit";

    // initialize k4a kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // initialize the 3dintact
    std::shared_ptr<I3d> sptr_i3d(new I3d());
    sptr_i3d->raiseRunFlag();

    // capture using k4a depth sensor
    std::thread k4aCaptureWorker(
        k4aCapture, std::ref(sptr_kinect), std::ref(sptr_i3d));

    // build point cloud
    std::thread buildPCloudWorker(buildPcl, std::ref(sptr_i3d));

    // propose region
    std::thread proposeRegionWorker(proposeRegion, std::ref(sptr_i3d));

    // segment region
    std::thread segmentRegionWorker(segmentRegion, std::ref(sptr_i3d));

    // cluster segmented region
    std::thread clusterRegionWorker(clusterRegion, std::ref(sptr_i3d));

    // ------> do stuff with tabletop environment <------

    SLEEP_UNTIL_SENSOR_DATA_READY
    std::vector<std::string> classnames;
    torch::jit::script::Module module;
    utils::configTorch(classnames, module);

    uint8_t* ptr_img;
    while (sptr_i3d->isRun()) {
        clock_t start = clock();
        int w = sptr_i3d->getDepthWidth();
        int h = sptr_i3d->getDepthHeight();

        ptr_img = *sptr_i3d->getSensorImgData();

        // create image frame
        cv::Mat img;
        cv::Mat frame
            = cv::Mat(h, w, CV_8UC4, (void*)ptr_img, cv::Mat::AUTO_STEP)
                  .clone();

        // format frame for tensor input
        cv::resize(frame, img, cv::Size(640, 384));
        cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
        torch::Tensor imgTensor = torch::from_blob(
            img.data, { img.rows, img.cols, 3 }, torch::kByte);
        imgTensor = imgTensor.permute({ 2, 0, 1 });
        imgTensor = imgTensor.toType(torch::kFloat);
        imgTensor = imgTensor.div(255);
        imgTensor = imgTensor.unsqueeze(0);

        torch::Tensor preds // preds: [?, 15120, 9]
            = module.forward({ imgTensor }).toTuple()->elements()[0].toTensor();
        std::vector<torch::Tensor> dets
            = yolo::non_max_suppression(preds, 0.4, 0.5);

        // show objects
        if (!dets.empty()) {
            for (size_t i = 0; i < dets[0].sizes()[0]; ++i) {
                float left
                    = dets[0][i][0].item().toFloat() * (float)frame.cols / 640;
                float top
                    = dets[0][i][1].item().toFloat() * (float)frame.rows / 384;
                float right
                    = dets[0][i][2].item().toFloat() * (float)frame.cols / 640;
                float bottom
                    = dets[0][i][3].item().toFloat() * (float)frame.rows / 384;
                float score = dets[0][i][4].item().toFloat();
                int classID = dets[0][i][5].item().toInt();

                cv::rectangle(frame,
                    cv::Rect(left, top, (right - left), (bottom - top)),
                    cv::Scalar(0, 255, 0), 2);
                cv::putText(frame,
                    classnames[classID] + ": " + cv::format("%.2f", score),
                    cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX,
                    (right - left) / 200, cv::Scalar(0, 255, 0), 2);
            }
        }
        utils::cvDisplay(frame, sptr_i3d, start);
    }
    // ------> do stuff with tabletop environment <------

    k4aCaptureWorker.join();
    buildPCloudWorker.join();
    proposeRegionWorker.join();
    segmentRegionWorker.join();
    clusterRegionWorker.join();
    return 0;
}
