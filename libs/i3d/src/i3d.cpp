#include <Eigen/Dense>
#include <chrono>
#include <k4a/k4a.hpp>
#include <opencv2/core.hpp>
#include <thread>
#include <utility>

#include "dbscan.h"
#include "helpers.h"
#include "i3d.h"
#include "kinect.h"
#include "macros.hpp"
#include "outliers.h"
#include "region.h"
#include "svd.h"

I3d::I3d()
{
    Point i3dMaxBound(SHRT_MAX, SHRT_MAX, SHRT_MAX);
    Point i3dMinBound(SHRT_MIN, SHRT_MIN, SHRT_MIN);

    m_boundary = { i3dMaxBound, i3dMinBound };

    sptr_run = std::make_shared<bool>(false);
    sptr_stop = std::make_shared<bool>(false);
    sptr_clustered = std::make_shared<bool>(false);
    sptr_segmented = std::make_shared<bool>(false);
    sptr_proposalReady = std::make_shared<bool>(false);
    sptr_pCloudReady = std::make_shared<bool>(false);
    sptr_resourcesReady = std::make_shared<bool>(false);
}

bool I3d::isRun()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    return *sptr_run;
}

void I3d::raiseRunFlag()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    *sptr_run = true;
}

bool I3d::isSensorReady()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    return *sptr_resourcesReady;
}

void I3d::raiseSensorReadyFlag()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    *sptr_resourcesReady = true;
}

bool I3d::isSegmented()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    return *sptr_segmented;
}

bool I3d::isProposalReady()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    return *sptr_proposalReady;
}

void I3d::raiseProposalReadyFlag()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    *sptr_proposalReady = true;
}

void I3d::raiseSegmentedFlag()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    *sptr_segmented = true;
}

bool I3d::isPCloudReady()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    return *sptr_pCloudReady;
}

void I3d::raisePCloudReadyFlag()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    *sptr_pCloudReady = true;
}

bool I3d::isClustered()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    return *sptr_clustered;
}

void I3d::raiseClusteredFlag()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    *sptr_clustered = true;
}

bool I3d::isStop()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    return *sptr_stop;
}

void I3d::raiseStopFlag()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    *sptr_stop = true;
}

void I3d::stop()
{
    std::lock_guard<std::mutex> lck(m_flagMutex);
    *sptr_run = false;
}

void I3d::setDepthHeight(const int& height)
{
    std::lock_guard<std::mutex> lck(m_depthDimensions);
    m_depthHeight = height;
}

void I3d::setDepthWidth(const int& width)
{
    std::lock_guard<std::mutex> lck(m_depthDimensions);
    m_depthWidth = width;
}

int I3d::getDepthWidth()
{
    std::lock_guard<std::mutex> lck(m_depthDimensions);
    return m_depthWidth;
}

int I3d::getDepthHeight()
{
    std::lock_guard<std::mutex> lck(m_depthDimensions);
    return m_depthHeight;
}

void I3d::setPCloud2x2Bin(const std::vector<Point>& points)
{
    std::lock_guard<std::mutex> lck(m_pCloud2x2BinMutex);
    sptr_pCloud2x2Bin = std::make_shared<std::vector<Point>>(points);
}

std::shared_ptr<std::vector<Point>> I3d::getPCloud2x2Bin()
{
    std::lock_guard<std::mutex> lck(m_pCloud2x2BinMutex);
    return sptr_pCloud2x2Bin;
}

void I3d::setPCloudSeg2x2Bin(const std::vector<Point>& points)
{
    std::lock_guard<std::mutex> lck(m_pCloudSeg2x2BinMutex);
    sptr_pCloudSeg2x2Bin = std::make_shared<std::vector<Point>>(points);
}

__attribute__((unused)) std::shared_ptr<std::vector<Point>>
I3d::getPCloudSeg2x2Bin()
{
    std::lock_guard<std::mutex> lck(m_pCloudSeg2x2BinMutex);
    return sptr_pCloudSeg2x2Bin;
}

void I3d::setSensorImgData(uint8_t* ptr_imgData)
{
    std::lock_guard<std::mutex> lck(m_sensorImgDataMutex);
    sptr_sensorImgData = std::make_shared<uint8_t*>(ptr_imgData);
}

std::shared_ptr<uint8_t*> I3d::getSensorImgData()
{
    std::lock_guard<std::mutex> lck(m_sensorImgDataMutex);
    return sptr_sensorImgData;
}
void I3d::setSensorPCloudData(int16_t* ptr_pclData)
{
    std::lock_guard<std::mutex> lck(m_sensorPCloudDataMutex);
    sptr_sensorPCloudData = std::make_shared<int16_t*>(ptr_pclData);
}

std::shared_ptr<int16_t*> I3d::getSensorPCloudData()
{
    std::lock_guard<std::mutex> lck(m_sensorPCloudDataMutex);
    return sptr_sensorPCloudData;
}

void I3d::setSensorDepthData(uint16_t* ptr_depth)
{
    std::lock_guard<std::mutex> lck(m_sensorDepthDataMutex);
    sptr_sensorDepthData = std::make_shared<uint16_t*>(ptr_depth);
}

std::shared_ptr<uint16_t*> I3d::getSensorDepthData()
{
    std::lock_guard<std::mutex> lck(m_sensorDepthDataMutex);
    return sptr_sensorDepthData;
}

void I3d::setSensorTableData(k4a_float2_t* ptr_table)
{
    std::lock_guard<std::mutex> lck(m_sensorTableDataMutex);
    ptr_sensorTableData = ptr_table;
}

k4a_float2_t* I3d::getSensorTableData()
{
    std::lock_guard<std::mutex> lck(m_sensorTableDataMutex);
    return ptr_sensorTableData;
}

void I3d::setPCloud(const std::vector<Point>& points)
{
    std::lock_guard<std::mutex> lck(m_pCloudMutex);
    sptr_pCloud = std::make_shared<std::vector<Point>>(points);
}

__attribute__((unused)) std::shared_ptr<std::vector<Point>> I3d::getPCloud()
{
    std::lock_guard<std::mutex> lck(m_pCloudMutex);
    return sptr_pCloud;
}

void I3d::setPCloudSeg(const std::vector<Point>& points)
{
    std::lock_guard<std::mutex> lck(m_pCloudSegMutex);
    sptr_pCloudSeg = std::make_shared<std::vector<Point>>(points);
}

std::shared_ptr<std::vector<Point>> I3d::getPCloudSeg()
{
    std::lock_guard<std::mutex> lck(m_pCloudSegMutex);
    return sptr_pCloudSeg;
}

void I3d::setImgFrame_GL(const std::vector<uint8_t>& frame)
{
    std::lock_guard<std::mutex> lck(m_imgFrameMutex_GL);
    sptr_imgFrame_GL = std::make_shared<std::vector<uint8_t>>(frame);
}

std::shared_ptr<std::vector<uint8_t>> I3d::getImgFrame_GL()
{
    std::lock_guard<std::mutex> lck(m_imgFrameMutex_GL);
    return sptr_imgFrame_GL;
}

void I3d::setI3dBoundary(std::pair<Point, Point>& boundary)
{
    std::lock_guard<std::mutex> lck(m_boundaryMutex);
    m_boundary = boundary;
}

std::pair<Point, Point> I3d::getBoundary()
{
    std::lock_guard<std::mutex> lck(m_boundaryMutex);
    return m_boundary;
}

void I3d::setPCloudFrame(const std::vector<int16_t>& frame)
{
    std::lock_guard<std::mutex> lck(m_pCloudFrameMutex);
    sptr_pCloudFrame = std::make_shared<std::vector<int16_t>>(frame);
}

std::shared_ptr<std::vector<int16_t>> I3d::getPCloudFrame()
{
    std::lock_guard<std::mutex> lck(m_pCloudFrameMutex);
    return sptr_pCloudFrame;
}

void I3d::setPCloudSegFrame(const std::vector<int16_t>& frame)
{
    std::lock_guard<std::mutex> lck(m_pCloudSegFrameMutex);
    sptr_pCloudSegFrame = std::make_shared<std::vector<int16_t>>(frame);
}

std::shared_ptr<std::vector<int16_t>> I3d::getPCloudSegFrame()
{
    std::lock_guard<std::mutex> lck(m_pCloudSegFrameMutex);
    return sptr_pCloudSegFrame;
}

void I3d::setImgSegFrame_GL(const std::vector<uint8_t>& frame)
{
    std::lock_guard<std::mutex> lck(m_imgSegFrameMutex_GL);
    sptr_imgFrameSeg_GL = std::make_shared<std::vector<uint8_t>>(frame);
}

std::shared_ptr<std::vector<uint8_t>> I3d::getImgSegFrame_GL()
{
    std::lock_guard<std::mutex> lck(m_imgSegFrameMutex_GL);
    return sptr_imgFrameSeg_GL;
}

void I3d::setImgFrame_CV(const std::vector<uint8_t>& frame)
{
    std::lock_guard<std::mutex> lck(m_imgSegFrameMutex_CV);
    sptr_imgFrame_CV = std::make_shared<std::vector<uint8_t>>(frame);
}

std::shared_ptr<std::vector<uint8_t>> I3d::getImgFrame_CV()
{
    std::lock_guard<std::mutex> lck(m_imgSegFrameMutex_CV);
    return sptr_imgFrame_CV;
}

void I3d::setPCloudClusters(const t_clusters& clusters)
{
    std::lock_guard<std::mutex> lck(m_pCloudClusterMutex);
    sptr_pCloudClusters = std::make_shared<t_clusters>(clusters);
}

__attribute__((unused)) std::shared_ptr<I3d::t_clusters>
I3d::getPCloudClusters()
{
    std::lock_guard<std::mutex> lck(m_pCloudClusterMutex);
    return sptr_pCloudClusters;
}

void I3d::setColClusters(const std::pair<int16_t*, uint8_t*>& colClusters)
{
    std::lock_guard<std::mutex> lck(m_colClusterMutex);
    sptr_colClusters
        = std::make_shared<std::pair<int16_t*, uint8_t*>>(colClusters);
}

std::shared_ptr<std::pair<int16_t*, uint8_t*>> I3d::getColClusters()
{
    std::lock_guard<std::mutex> lck(m_colClusterMutex);
    return sptr_colClusters;
}

void I3d::buildPCloud(std::shared_ptr<I3d>& sptr_i3d)
{
#if BUILD_POINTCLOUD == 1
    SLEEP_UNTIL_SENSOR_DATA_READY
    START
    int w = sptr_i3d->getDepthWidth();
    int h = sptr_i3d->getDepthHeight();

    uint16_t* ptr_depthData;
    k4a_float2_t* ptr_tableData;

    std::vector<Point> pCloud(w * h);
    while (sptr_i3d->isRun()) {

        START_TIMER
        ptr_depthData = *sptr_i3d->getSensorDepthData();
        ptr_tableData = sptr_i3d->getSensorTableData();

        int index = 0;
        for (int i = 0; i < w * h; i++) {
            if (utils::invalid(i, ptr_tableData, ptr_depthData)) {
                continue;
            }
            int16_t x = ptr_tableData[i].xy.x * (float)ptr_depthData[i];
            int16_t y = ptr_tableData[i].xy.y * (float)ptr_depthData[i];
            int16_t z = (float)ptr_depthData[i];
            Point point(x, y, z);
            pCloud[index] = point;
            index++;
        }
        std::vector<Point> optimizedPCloud(
            pCloud.begin(), pCloud.begin() + index);

        sptr_i3d->setPCloud2x2Bin(optimizedPCloud);
        RAISE_POINTCLOUD_READY_FLAG
        STOP_TIMER(" build point cloud thread: runtime @ ")
    }
#endif
}

void I3d::proposeRegion(std::shared_ptr<I3d>& sptr_i3d)
{
#if PROPOSE_REGION == 1
    SLEEP_UNTIL_POINTCLOUD_READY
    START
    while (sptr_i3d->isRun()) {
        START_TIMER
        std::vector<Point> pCloud = *sptr_i3d->getPCloud2x2Bin();
        std::vector<Point> pCloudSeg = region::segment(pCloud);
        std::pair<Point, Point> boundary = utils::queryBoundary(pCloudSeg);
        sptr_i3d->setPCloudSeg2x2Bin(pCloudSeg);
        sptr_i3d->setI3dBoundary(boundary);
        RAISE_PROPOSAL_READY_FLAG
        STOP_TIMER(" propose region thread: runtime @ ")
    }
#endif
}

void I3d::segmentRegion(std::shared_ptr<I3d>& sptr_i3d)
{
#if SEGMENT_REGION == 1
    SLEEP_UNTIL_PROPOSAL_READY_FLAG
    START
    int w = sptr_i3d->getDepthWidth();
    int h = sptr_i3d->getDepthHeight();

    std::vector<Point> pCloud(w * h);
    std::vector<Point> pCloudSeg(w * h);

    std::vector<int16_t> pCloudFrame(w * h * 3);
    std::vector<uint8_t> imgFrame_GL(w * h * 4);
    std::vector<uint8_t> imgFrame_CV(w * h * 4);

    std::vector<int16_t> pCloudSegFrame(w * h * 3);
    std::vector<uint8_t> imgSegFrame_GL(w * h * 4);
    std::vector<uint8_t> imgSegFrame_CV(w * h * 4);

    uint8_t* ptr_sensorImgData;
    int16_t* ptr_sensorPCloudData;

    while (sptr_i3d->isRun()) {
        START_TIMER
        ptr_sensorPCloudData = *sptr_i3d->getSensorPCloudData();
        ptr_sensorImgData = *sptr_i3d->getSensorImgData();

        int index = 0;
        for (int i = 0; i < w * h; i++) {
            Point point;

            // create unsegmented assets
            if (utils::invalid(i, ptr_sensorPCloudData, ptr_sensorImgData)) {
                utils::addXYZ(i, pCloudFrame);
                utils::addPixel_GL(i, imgFrame_GL);
                utils::addPixel_CV(i, imgFrame_CV);
            } else {
                utils::addXYZ(i, pCloudFrame, ptr_sensorPCloudData);
                utils::addPixel_GL(i, imgFrame_GL, ptr_sensorImgData);
                utils::addPixel_CV(i, imgFrame_CV, ptr_sensorImgData);
            }
            utils::adapt(i, point, pCloudFrame, imgFrame_CV);
            pCloud[i] = point;

            // create segmented assets
            if (utils::inSegment(i, pCloudFrame, sptr_i3d->getBoundary().first,
                    sptr_i3d->getBoundary().second)) {
                utils::addXYZ(i, pCloudSegFrame, ptr_sensorPCloudData);
                utils::addPixel_GL(i, imgSegFrame_GL, ptr_sensorImgData);
                utils::addPixel_CV(i, imgSegFrame_CV, ptr_sensorImgData);
                pCloudSeg[index] = point;
                index++;
            } else {
                utils::addXYZ(i, pCloudSegFrame);
                utils::addPixel_GL(i, imgSegFrame_GL);
            }
        }
        std::vector<Point> optimizedPCloudSeg(
            pCloudSeg.begin(), pCloudSeg.begin() + index);

        sptr_i3d->setPCloud(pCloud);
        sptr_i3d->setPCloudFrame(pCloudFrame);
        sptr_i3d->setImgFrame_GL(imgFrame_GL);
        sptr_i3d->setImgFrame_CV(imgFrame_CV);
        sptr_i3d->setPCloudSeg(optimizedPCloudSeg);
        sptr_i3d->setPCloudSegFrame(pCloudSegFrame);
        sptr_i3d->setImgSegFrame_GL(imgSegFrame_GL);
        RAISE_SEGMENT_READY_FLAG
        STOP_TIMER(" frame region thread: runtime @ ")
    }
#endif
}

void I3d::clusterRegion(
    const float& epsilon, const int& minPoints, std::shared_ptr<I3d>& sptr_i3d)
{
#if CLUSTER_REGION == 1
    SLEEP_UNTIL_SEGMENT_READY
    int w = sptr_i3d->getDepthWidth();
    int h = sptr_i3d->getDepthHeight();
    START
    while (sptr_i3d->isRun()) {
        START_TIMER
        std::vector<Point> points = *sptr_i3d->getPCloudSeg();

        // dbscan::cluster clusters candidate interaction regions
        //   from the segmented tabletop surface. It returns a
        //   collection of clusters, more specifically, a list of
        //   index collections. The indexes correspond to clustered
        //   3D points specified as part of function arguments.
        //
        auto indexClusters = dbscan::cluster(points, epsilon, minPoints);

        // sorting the clusters is one possible approach to
        //   extracting the vacant or non occupied space on
        //   the tabletop surface, this is of cause assuming
        //   non occupied space is prevalent on the tabletop
        //   environment.
        //
        std::sort(indexClusters.begin(), indexClusters.end(),
            [](const std::vector<unsigned long>& a,
                const std::vector<unsigned long>& b) {
                return a.size() > b.size();
            });

        // cast *index clusters to *point clusters
        std::vector<std::vector<Point>> pointClusters;
        for (const auto& cluster : indexClusters) {
            std::vector<Point> heap;
            for (const auto& index : cluster) {
                heap.emplace_back(points[index]);
            }
            pointClusters.emplace_back(heap);
        }

        // extract the distance to the tabletop surface (f)
        std::vector<float> depth(pointClusters[0].size());
        for (int i = 0; i < pointClusters.size(); i++) {
            depth[i] = pointClusters[0][i].m_xyz[2];
        }
        int16_t f = *std::max_element(depth.begin(), depth.end());

        // one approach to analyzing the clusters is to study
        // the face normals.
        //

        // config flags for svd computation
        int flag = Eigen::ComputeThinU | Eigen::ComputeThinV;

        // compute and heap the normals of each cluster
        std::vector<Eigen::Vector3d> normals(pointClusters.size());
        int index = 0;
        for (const auto& cluster : pointClusters) {
            SVD usv(cluster, flag);
            normals[index] = usv.getV3Normal();
            index++;
        }

        // extract the vacant space and corresponding normal
        std::vector<Point> vacantSpace = pointClusters[0];
        Eigen::Vector3d n1 = normals[0];

        pointClusters.erase(pointClusters.begin());
        normals.erase(normals.begin());
        const float ARGMIN = -0.00000008;

        std::vector<std::vector<Point>> objectClusters;

        // find coplanar clusters
        index = 0;
        const float GIVE_WAY = 10; // height restriction in mm
        for (const auto& n2 : normals) {
            double numerator = n1.dot(n2);
            double denominator = n1.norm() * n2.norm();
            double solution = std::acos(numerator / denominator);

            if (!std::isnan(solution) && solution < ARGMIN
                && solution > -ARGMIN) {
                for (const auto& point : pointClusters[index]) {
                    vacantSpace.emplace_back(point);
                }
            } else {
                objectClusters.emplace_back(pointClusters[index]);
            }
            index++;
        }

        // colorize objects on surface
        std::vector<uint8_t*> colors;
        utils::add(colors);

        // stitch colorized segment
        std::vector<Point> pCloudSegment;
        index = 0;
        for (auto& object : objectClusters) {
            if (index < colors.size()) {
                for (auto& point : object) {
                    // point.setPixel_GL(colors[index]);
                    pCloudSegment.emplace_back(point);
                }
                index++;
            }
        }
        uint8_t col[4] = { 0, 0, 0, 0 };
        for (auto& point : vacantSpace) {
            point.setPixel_GL(col);
            pCloudSegment.emplace_back(point);
        }

        int16_t pCloudFrame[w * h * 3];
        uint8_t imgFrame_GL[w * h * 4];
        uint8_t imgFrame_CV[w * h * 4];

        index = 0;
        for (const auto& point : pCloudSegment) {
            utils::stitch(index, pCloudSegment[index], pCloudFrame, imgFrame_GL,
                imgFrame_CV);
            index++;
        }

        sptr_i3d->setColClusters({ pCloudFrame, imgFrame_GL });

        // writePoints(pCloudSegment);
        // writePoints(vacantSpace);

        sptr_i3d->setPCloudClusters({ points, indexClusters });
        RAISE_CLUSTERS_READY_FLAG
        STOP_TIMER(" cluster region thread: runtime @ ")
    }
#endif
}
