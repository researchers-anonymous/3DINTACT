#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <thread>

#include "helpers.h"
#include "i3d.h"
#include "io.h"
#include "kinect.h"
#include "macros.hpp"
#include "svd.h"

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
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<i3d>& sptr_i3d)
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
    std::shared_ptr<i3d> sptr_i3d(new i3d());
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

    SLEEP_UNTIL_CLUSTERS_READY
    while (sptr_i3d->isRun()) {
        clock_t start = clock(); // <- frame-rate timestamp
        auto clusters = sptr_i3d->getPCloudClusters(); // <-get clusters
        auto indexClusters = clusters->second;
        std::vector<Point> points = clusters->first;

        // define chromakey color for tabletop surface
        uint8_t rgb[3] = { chromagreen[0], chromagreen[1], chromagreen[2] };
        uint8_t bgra[4] = { chromagreen[2], chromagreen[1], chromagreen[0], 0 };

        // cast 'index clusters' to 'point clusters'
        std::vector<std::vector<Point>> pointClusters;
        for (const auto& cluster : indexClusters) {

            std::vector<Point> heap;
            for (const auto& index : cluster) {
                heap.emplace_back(points[index]);
            }

            pointClusters.emplace_back(heap);
        }

        // extract the lower height of the vacant surface
        std::vector<float> zMeasures(pointClusters[0].size());
        for (int i = 0; i < pointClusters.size(); i++) {
            zMeasures[i] = pointClusters[0][i].m_xyz[2];
        }
        int16_t lowerLimit
            = *std::min_element(zMeasures.begin(), zMeasures.end());
        int16_t upperLimit
            = *std::max_element(zMeasures.begin(), zMeasures.end());

        /** use SVD norm to find stitch vacant tabletop surface */
        // config flags for svd computation
        int flag = Eigen::ComputeThinU | Eigen::ComputeThinV;

        // heap the normals of each cluster
        std::vector<Eigen::Vector3d> normals;
        for (const auto& cluster : pointClusters) {
            SVD usv(cluster, flag);
            normals.emplace_back(usv.getV3Normal());
        }

        Eigen::Vector3d n1 = normals[0]; // n1 = vacantSurfaceNorm
        normals.erase(normals.begin());

        // evaluate coplanarity between clusters
        std::vector<Point> vacantTabletop = pointClusters[0];

        int index = 0;
        const int GIVE_WAY = 2; // height of vacant surface in mm
        const float ARGMIN = 0.4;
        for (const auto& n2 : normals) {
            double numerator = n1.dot(n2);
            double denominator = n1.norm() * n2.norm();
            double solution = std::acos(numerator / denominator);

            if (!std::isnan(solution) && solution < ARGMIN
                && solution > -ARGMIN) {

                for (const auto& point : pointClusters[index]) {
                    if (point.m_xyz[2] > upperLimit - GIVE_WAY) {
                        vacantTabletop.emplace_back(point);
                    }
                }
            }
            index++;
        }

        std::vector<Point> frame = *sptr_i3d->getPCloud();

        // chromakey vacant tabletop surface
        for (const auto& point : vacantTabletop) {
            int id = point.m_id;
            frame[id].setPixel_GL(rgb);
            frame[id].setPixel_CV(bgra);
        }
        int w = sptr_i3d->getDepthWidth();
        int h = sptr_i3d->getDepthHeight();

        uint8_t img_CV[w * h * 4];
        for (int i = 0; i < w * h; i++) {
            utils::stitch(i, frame[i], img_CV);
        }

        cv::Mat img = cv::Mat(h, w, CV_8UC4, (void*)img_CV, cv::Mat::AUTO_STEP);

        utils::cvDisplay(img, sptr_i3d, start);
    }

    // ------> do stuff with tabletop environment <------

    k4aCaptureWorker.join();
    buildPCloudWorker.join();
    proposeRegionWorker.join();
    segmentRegionWorker.join();
    clusterRegionWorker.join();
    return 0;
}
