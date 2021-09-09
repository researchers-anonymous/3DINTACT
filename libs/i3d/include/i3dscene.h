#ifndef SCENE_H
#define SCENE_H

#include <chrono>
#include <string>
#include <thread>

#include "i3d.h"
#include "i3dmacros.hpp"
#include "io.h"
#include "kinect.h"

namespace i3dscene {

static std::thread k4aCaptureWorker;    // camera capture thread
static std::thread buildPCloudWorker;   // point-cloud builder thread
static std::thread proposeRegionWorker; // propose thread
static std::thread segmentRegionWorker; // segment thread
static std::thread clusterRegionWorker; // cluster thread

void clusterRegion(std::shared_ptr<i3d>& sptr_i3d)
{
    int minPoints = 4;
    const float epsilon = 3.170;
    sptr_i3d->clusterRegion(epsilon, minPoints, sptr_i3d);
}

void segmentRegion(std::shared_ptr<i3d>& sptr_i3d)
{
    sptr_i3d->segmentRegion(sptr_i3d);
}

void proposeRegion(std::shared_ptr<i3d>& sptr_i3d)
{
    sptr_i3d->proposeRegion(sptr_i3d);
}

void buildPcl(std::shared_ptr<i3d>& sptr_i3d)
{
    sptr_i3d->buildPCloud(sptr_i3d);
}

void k4aCapture(
    std::shared_ptr<kinect>& sptr_kinect, std::shared_ptr<i3d>& sptr_i3d)
{
    START
    sptr_kinect->capture();
    sptr_kinect->depthCapture();
    int w = k4a_image_get_width_pixels(sptr_kinect->m_depth);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_depth);

    while (RUN) {
        START_TIMER
        sptr_kinect->capture();
        sptr_kinect->depthCapture();
        sptr_kinect->pclCapture();
        sptr_kinect->imgCapture();
        sptr_kinect->c2dCapture();
        sptr_kinect->transform(RGB_TO_DEPTH);

        auto* ptr_k4aImgData = k4a_image_get_buffer(sptr_kinect->m_img);
        auto* ptr_k4aC2DImgData = k4a_image_get_buffer(sptr_kinect->m_c2d);
        auto* ptr_k4aPCloudData
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_pcl);
        auto* ptr_k4aDepthData
            = (uint16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_depth);
        auto* ptr_k4aTableData
            = (k4a_float2_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_xyT);

        // share k4a resources with intact
        sptr_i3d->setDWidth(w);
        sptr_i3d->setDHeight(h);
        sptr_i3d->setBGRAData(ptr_k4aImgData);
        sptr_i3d->setC2DBGRAData(ptr_k4aC2DImgData);
        sptr_i3d->setXYTableData(ptr_k4aTableData);
        sptr_i3d->setDepthData(ptr_k4aDepthData);
        sptr_i3d->setXYZData(ptr_k4aPCloudData);

        // release k4a resources
        sptr_kinect->releaseK4aImages();
        sptr_kinect->releaseK4aCapture();
        RAISE_SENSOR_RESOURCES_READY_FLAG
        EXIT_CALLBACK
        STOP_TIMER(" k4a driver thread: runtime @ ")
    }
}

void context(
    std::shared_ptr<kinect>& sptr_kinect, std::shared_ptr<i3d> sptr_i3d)
{
    sptr_i3d->raiseRunFlag();

    k4aCaptureWorker
        = std::thread(k4aCapture, std::ref(sptr_kinect), std::ref(sptr_i3d));
    buildPCloudWorker = std::thread(buildPcl, std::ref(sptr_i3d));
    proposeRegionWorker = std::thread(proposeRegion, std::ref(sptr_i3d));
    segmentRegionWorker = std::thread(segmentRegion, std::ref(sptr_i3d));
    clusterRegionWorker = std::thread(clusterRegion, std::ref(sptr_i3d));

    // expedite main thread resumption
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    k4aCaptureWorker.join();
    buildPCloudWorker.join();
    proposeRegionWorker.join();
    segmentRegionWorker.join();
    clusterRegionWorker.join();
}

}
#endif /* SCENE_H */
