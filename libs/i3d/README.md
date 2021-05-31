### :mortar_board: :mortar_board: :mortar_board: :mortar_board:

#### an API for extracting surfaces and surface objects from images and 3 point clouds

*   the interface

```cpp

#include <memory>
#include <mutex>
#include <torch/script.h>
#include <vector>

#include "kinect.h"
#include "point.h"

class I3d {

public:
    int m_depthWidth {};
    int m_depthHeight {};

private:
    std::mutex m_depthDimensions;

    std::mutex m_sensorTableDataMutex;
    std::mutex m_sensorImgDataMutex;
    std::mutex m_sensorDepthDataMutex;
    std::mutex m_sensorPCloudDataMutex;

    std::mutex m_pCloudMutex;
    std::mutex m_pCloudSegMutex;

    std::mutex m_pCloudFrameMutex;
    std::mutex m_pCloudSegFrameMutex;

    std::mutex m_pCloud2x2BinMutex;
    std::mutex m_pCloudSeg2x2BinMutex;

    std::mutex m_imgFrameMutex_GL;

    std::mutex m_imgSegFrameMutex_GL;
    std::mutex m_imgSegFrameMutex_CV;

    std::mutex m_boundaryMutex;

    std::mutex m_flagMutex;
    std::mutex m_pCloudClusterMutex;

    std::shared_ptr<std::vector<Point>> sptr_pCloud = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_pCloudSeg = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_pCloud2x2Bin = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_pCloudSeg2x2Bin = nullptr;

    k4a_float2_t* ptr_sensorTableData = nullptr;
    std::shared_ptr<uint8_t*> sptr_sensorImgData = nullptr;
    std::shared_ptr<int16_t*> sptr_sensorPCloudData = nullptr;
    std::shared_ptr<uint16_t*> sptr_sensorDepthData = nullptr;

    std::pair<Point, Point> m_boundary {};
    std::shared_ptr<std::vector<uint8_t>> sptr_imgFrame_CV = nullptr;
    std::shared_ptr<std::vector<uint8_t>> sptr_imgFrame_GL = nullptr;
    std::shared_ptr<std::vector<int16_t>> sptr_pCloudFrame = nullptr;
    std::shared_ptr<std::vector<int16_t>> sptr_pCloudSegFrame = nullptr;
    std::shared_ptr<std::vector<uint8_t>> sptr_imgFrameSeg_GL = nullptr;

    typedef std::pair<std::vector<Point>,
        std::vector<std::vector<unsigned long>>>
        t_clusters;
    std::shared_ptr<t_clusters> sptr_pCloudClusters = nullptr;

    std::shared_ptr<bool> sptr_run;
    std::shared_ptr<bool> sptr_stop;
    std::shared_ptr<bool> sptr_clustered;
    std::shared_ptr<bool> sptr_segmented;
    std::shared_ptr<bool> sptr_pCloudReady;
    std::shared_ptr<bool> sptr_proposalReady;
    std::shared_ptr<bool> sptr_resourcesReady;

public:
    /** i3d
     *   Constructs instance of 3dintact
     */
    I3d();

    /** proposeRegion
     *   Proposes a region/segment of an orthogonal
     *   planar surface from a 3D point cloud.
     *
     * @param sptr_i3d
     *   Instance of API call.
     */
    static void proposeRegion(std::shared_ptr<I3d>& sptr_i3d);

    /** buildPCloud
     *   Builds a point cloud suited to i3d computations
     *
     * @param sptr_i3d
     *   Instance of API call.
     */
    static void buildPCloud(std::shared_ptr<I3d>& sptr_i3d);

    /** clusterRegion
     *   Does spatial clustering of the extracted
     *   planar surface.
     *
     * @param epsilon
     *   Epsilon parameter.
     * @param minPoints
     *   Number of epsilon-neighbourhood neighbours.
     * @param sptr_i3d
     *   Instance of API call.
     */
    static void clusterRegion(const float& epsilon, const int& minPoints,
        std::shared_ptr<I3d>& sptr_i3d);

    /** segmentRegion
     *   Creates point cloud and image data frames
     *   suited to i3d computations.
     *
     * @param sptr_i3d
     *   Instance of API call.
     */
    static void segmentRegion(std::shared_ptr<I3d>& sptr_i3d);

    void stop();
    bool isRun();
    bool isStop();
    bool isSegmented();
    bool isClustered();
    bool isProposalReady();
    bool isPCloudReady();
    bool isSensorReady();

    void raiseRunFlag();
    void raiseStopFlag();
    void raiseSegmentedFlag();
    void raiseClusteredFlag();
    void raiseSensorReadyFlag();
    void raiseProposalReadyFlag();
    void raisePCloudReadyFlag();

    int getDepthWidth();
    int getDepthHeight();
    void setDepthWidth(const int& width);
    void setDepthHeight(const int& height);

    void setSensorPCloudData(int16_t* ptr_pclData);
    std::shared_ptr<int16_t*> getSensorPCloudData();

    void setSensorTableData(k4a_float2_t* ptr_table);
    k4a_float2_t* getSensorTableData();

    void setSensorImgData(uint8_t* ptr_imgData);
    std::shared_ptr<uint8_t*> getSensorImgData();

    void setSensorDepthData(uint16_t* ptr_depth);
    std::shared_ptr<uint16_t*> getSensorDepthData();

    void setPCloud2x2Bin(const std::vector<Point>& points);
    std::shared_ptr<std::vector<Point>> getPCloud2x2Bin();

    void setImgFrame_CV(const std::vector<uint8_t>& frame);
    std::shared_ptr<std::vector<uint8_t>> getImgFrame_CV();

    void setPCloudFrame(const std::vector<int16_t>& frame);
    std::shared_ptr<std::vector<int16_t>> getPCloudFrame();

    void setImgFrame_GL(const std::vector<uint8_t>& frame);
    std::shared_ptr<std::vector<uint8_t>> getImgFrame_GL();

    void setPCloudSegFrame(const std::vector<int16_t>& frame);
    std::shared_ptr<std::vector<int16_t>> getPCloudSegFrame();

    void setImgSegFrame_GL(const std::vector<uint8_t>& frame);
    std::shared_ptr<std::vector<uint8_t>> getImgSegFrame_GL();

    void setI3dBoundary(std::pair<Point, Point>& boundary);
    std::pair<Point, Point> getBoundary();

    void setPCloud(const std::vector<Point>& points);
    __attribute__((unused)) std::shared_ptr<std::vector<Point>> getPCloud();

    void setPCloudSeg(const std::vector<Point>& points);
    std::shared_ptr<std::vector<Point>> getPCloudSeg();

    void setPCloudSeg2x2Bin(const std::vector<Point>& points);
    __attribute__((unused)) std::shared_ptr<std::vector<Point>>
    getPCloudSeg2x2Bin();

    void setPCloudClusters(const t_clusters& clusters);
    __attribute__((unused)) std::shared_ptr<t_clusters> getPCloudClusters();
};

```

*   basic setup (see [`examples`](./examples) for practical examples)

```cpp
#include <chrono>
#include <string>
#include <thread>

#include "i3d.h"
#include "io.h"
#include "kinect.h"
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

    // ------> do stuff with tabletop environment <------

    k4aCaptureWorker.join();
    buildPCloudWorker.join();
    proposeRegionWorker.join();
    segmentRegionWorker.join();
    clusterRegionWorker.join();
    return 0;
}
```

###### Checkout:

*   the [`calibration`](https://github.com/edisonslightbulbs/calibration)  submodule dependency
*   the [`dbscan`](https://github.com/edisonslightbulbs/dbscan)  submodule dependency
*   the [`edge`](https://github.com/edisonslightbulbs/edge)  submodule dependency
*   the [`outliers`](https://github.com/edisonslightbulbs/outliers)  submodule dependency
*   the [`svd`](https://github.com/edisonslightbulbs/svd)  submodule dependency
*   the [`kinect`](https://github.com/edisonslightbulbs/kinect)  submodule dependency
*   the [`knn`](https://github.com/edisonslightbulbs/knn)  submodule dependency
*   the [`or`](https://github.com/edisonslightbulbs/or)  submodule dependency
*   the [`point`](https://github.com/edisonslightbulbs/point)  submodule dependency
*   the [`segment`](https://github.com/edisonslightbulbs/segment)  submodule dependency
*   the [`viewer`](https://github.com/edisonslightbulbs/viewer)  submodule dependency
