#include <chrono>
#include <string>
#include <thread>
#include <knn.h>

#include "i3d.h"
#include "io.h"
#include "kinect.h"
#include "macros.hpp"

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
    sptr_kinect->imgCapture();
    sptr_kinect->depthCapture();
    int imgWidth = k4a_image_get_width_pixels(sptr_kinect->m_img);
    int imgHeight = k4a_image_get_height_pixels(sptr_kinect->m_img);
    int depthWidth = k4a_image_get_width_pixels(sptr_kinect->m_depth);
    int depthHeight = k4a_image_get_height_pixels(sptr_kinect->m_depth);

    while (sptr_i3d->isRun()) {
        START_TIMER
        sptr_kinect->capture();
        sptr_kinect->depthCapture();
        sptr_kinect->pclCapture();
        sptr_kinect->imgCapture();
        sptr_kinect->c2dCapture();
        sptr_kinect->transform(RGB_TO_DEPTH);

        auto* ptr_k4aC2dData = k4a_image_get_buffer(sptr_kinect->m_c2d);
        auto* ptr_k4aImgData = k4a_image_get_buffer(sptr_kinect->m_img);
        auto* ptr_k4aPCloudData
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_pcl);
        auto* ptr_k4aDepthData
            = (uint16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_depth);
        auto* ptr_k4aTableData
            = (k4a_float2_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_xyT);

        // share k4a resources with intact
        sptr_i3d->setImgWidth(imgWidth);
        sptr_i3d->setImgHeight(imgHeight);
        sptr_i3d->setDepthWidth(depthWidth);
        sptr_i3d->setDepthHeight(depthHeight);
        sptr_i3d->setSensorImgData(ptr_k4aImgData);
        sptr_i3d->setSensorC2DImgData(ptr_k4aC2dData);
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

    // ------> do stuff with tabletop environment <------
    SLEEP_UNTIL_POINTCLOUD_READY
    std::vector<Point> pCloud = *sptr_i3d->getPCloud2x2Bin();

    SLEEP_UNTIL_PROPOSAL_READY
    std::vector<Point> pCloudSeg = *sptr_i3d->getPCloudSeg2x2Bin();
    std::vector<Point> queryPoints = pCloudSeg;

    // heap the distance to the 4th nearest
    // neighbour of each segment point
    std::vector<float> heapDists = knn::compute(pCloudSeg, queryPoints, 4);

    // sort heaped distances and output them for plotting
    // once plot, use the value that corresponds to the
    // 'elbow curve' as a proxy to the epsilon parameter
    std::sort(heapDists.begin(), heapDists.end(), std::greater<>());
    io::write(heapDists);

    STOP
    // ------> do stuff with tabletop environment <------

    proposeRegionWorker.join();
    k4aCaptureWorker.join();
    buildPCloudWorker.join();
    return 0;
}
