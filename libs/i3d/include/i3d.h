#ifndef I3D_H
#define I3D_H

#include <memory>
#include <mutex>
#include <torch/script.h>
#include <vector>

#include "kinect.h"
#include "point.h"

class i3d {

public:
    int m_imgWidth {};
    int m_imgHeight {};

    int m_depthWidth {};
    int m_depthHeight {};

private:
    std::mutex m_dImgMutex;
    std::mutex m_dDepthMutex;

    std::mutex m_depthMutex;
    std::mutex m_XYZRawMutex;
    std::mutex m_XYTableMutex;
    std::mutex m_BGRARawMutex;

    std::mutex m_pCloudMutex;
    std::mutex m_pCloudSegMutex;
    std::mutex m_optimizedPCloudSegMutex;

    std::mutex m_XYZMutex;
    std::mutex m_XYZSegMutex;
    std::mutex m_pCloud2x2BinMutex;
    std::mutex m_pCloudSeg2x2BinMutex;

    std::mutex m_RGBAMutex;
    std::mutex m_BGRAMutex;

    std::mutex m_RGBASegMutex;

    std::mutex m_boundaryMutex;

    std::mutex m_flagMutex;
    std::mutex m_pCloudClusterMutex;
    std::mutex m_colorizedClustersMutex;

    std::shared_ptr<std::vector<Point>> sptr_pCloud = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_pCloudSeg = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_pCloud2x2Bin = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_pCloudSeg2x2Bin = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_optimizedPCloudSeg = nullptr;

    k4a_float2_t* ptr_XYTableData = nullptr; // todo: cross check
    std::shared_ptr<int16_t*> sptr_XYZRaw = nullptr;
    std::shared_ptr<uint8_t*> sptr_BGRARaw = nullptr;
    std::shared_ptr<uint8_t*> sptr_BGRAC2DRaw = nullptr;
    std::shared_ptr<uint16_t*> sptr_XYZDepth = nullptr;

    std::pair<Point, Point> m_segBoundary {};
    std::shared_ptr<std::vector<int16_t>> sptr_XYZ = nullptr;
    std::shared_ptr<std::vector<uint8_t>> sptr_BGRA = nullptr;
    std::shared_ptr<std::vector<uint8_t>> sptr_RGBA = nullptr;
    std::shared_ptr<std::vector<int16_t>> sptr_XYZSeg = nullptr;
    std::shared_ptr<std::vector<uint8_t>> sptr_RGBASeg = nullptr;

    typedef std::pair<std::vector<Point>,
        std::vector<std::vector<unsigned long>>>
        t_clusters;

    std::shared_ptr<t_clusters> sptr_pCloudClusters = nullptr;
    std::shared_ptr<std::pair<int16_t*, uint8_t*>> sptr_colorizedClusters
        = nullptr;

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
    i3d();

    /** proposeRegion
     *   Proposes a region/segment of an orthogonal
     *   planar surface from a 3D point cloud.
     *
     * @param sptr_i3d
     *   Instance of API call.
     */
    static void proposeRegion(std::shared_ptr<i3d>& sptr_i3d);

    /** buildPCloud
     *   Builds a point cloud suited to i3d computations
     *
     * @param sptr_i3d
     *   Instance of API call.
     */
    static void buildPCloud(std::shared_ptr<i3d>& sptr_i3d);

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
        std::shared_ptr<i3d>& sptr_i3d);

    /** segmentRegion
     *   Creates point cloud and image data frames
     *   suited to i3d computations.
     *
     * @param sptr_i3d
     *   Instance of API call.
     */
    static void segmentRegion(std::shared_ptr<i3d>& sptr_i3d);

    void stop();
    bool isRun();
    bool isStop();
    bool isSegmented();
    bool isClustered();
    bool isProposalReady();
    bool isPCloudReady();
    bool isSensorReady();

    void raiseRunFlag();
    // void raiseStopFlag();
    void raiseSegmentedFlag();
    void raiseClusteredFlag();
    void raiseSensorReadyFlag();
    void raiseProposalReadyFlag();
    void raisePCloudReadyFlag();

    int getRGBAWidth();
    int getRGBAHeight();
    void setRGBAWidth(const int& width);
    void setRGBAHeight(const int& height);

    int getDWidth();
    int getDHeight();
    void setDWidth(const int& width);
    void setDHeight(const int& height);

    void setXYZData(int16_t* xyz);
    std::shared_ptr<int16_t*> getXYZData();

    void setXYTableData(k4a_float2_t* ptr_table);
    k4a_float2_t* getXYTableData();

    void setC2DBGRAData(uint8_t* bgra);
    std::shared_ptr<uint8_t*> getC2DBGRAData();

    void setBGRAData(uint8_t* bgra);
    std::shared_ptr<uint8_t*> getBGRAData();

    void setDepthData(uint16_t* xyz);
    std::shared_ptr<uint16_t*> getDepthData();

    void setPCloud2x2Bin(const std::vector<Point>& points);
    std::shared_ptr<std::vector<Point>> getPCloud2x2Bin();

    void setBGRA(const std::vector<uint8_t>& bgra);
    std::shared_ptr<std::vector<uint8_t>> getBGRA();

    void setXYZ(const std::vector<int16_t>& xyz);
    std::shared_ptr<std::vector<int16_t>> getXYZ();

    void setRGBA(const std::vector<uint8_t>& rgba);
    std::shared_ptr<std::vector<uint8_t>> getRGBA();

    void setXYZSeg(const std::vector<int16_t>& xyz);
    std::shared_ptr<std::vector<int16_t>> getXYZSeg();

    void setRGBASeg(const std::vector<uint8_t>& rgba);
    std::shared_ptr<std::vector<uint8_t>> getRGBASeg();

    void setSegBoundary(std::pair<Point, Point>& boundary);
    std::pair<Point, Point> getSegBoundary();

    void setPCloud(const std::vector<Point>& points);
    __attribute__((unused)) std::shared_ptr<std::vector<Point>> getPCloud();

    void setPCloudSeg(const std::vector<Point>& points);
    std::shared_ptr<std::vector<Point>> getPCloudSeg();

    void setOptimizedPCloudSeg(const std::vector<Point>& points);
    std::shared_ptr<std::vector<Point>> getOptimizedPCloudSeg();

    void setPCloudSeg2x2Bin(const std::vector<Point>& points);
    __attribute__((unused)) std::shared_ptr<std::vector<Point>>
    getPCloudSeg2x2Bin();

    void setPCloudClusters(const t_clusters& clusters);
    std::shared_ptr<t_clusters> getPCloudClusters();

    void setColorizedClusters(
        const std::pair<int16_t*, uint8_t*>& colorizedClusters);

    std::shared_ptr<std::pair<int16_t*, uint8_t*>> getColorizedClusters();
};
#endif /* I3D_H */
