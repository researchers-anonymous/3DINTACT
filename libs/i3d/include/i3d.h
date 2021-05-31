#ifndef I3D_H
#define I3D_H

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
#endif /* I3D_H */
