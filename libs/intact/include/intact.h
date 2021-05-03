#ifndef INTACT_H
#define INTACT_H

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <vector>

#include "kinect.h"
#include "point.h"

class Intact {

public:
    /** number of raw point cloud points */
    int m_numPoints;

    /** raw point cloud */
    std::shared_ptr<std::vector<float>> sptr_raw = nullptr;
    std::shared_ptr<std::vector<uint8_t>> sptr_rawColor = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_rawPoints = nullptr;

    /** raw point cloud segment */
    std::shared_ptr<std::vector<float>> sptr_segment = nullptr;
    std::shared_ptr<std::vector<uint8_t>> sptr_segmentColor = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_segmentPoints = nullptr;

    /** region */
    std::shared_ptr<std::vector<float>> sptr_region = nullptr;
    std::shared_ptr<std::vector<uint8_t>> sptr_regionColor = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_regionPoints = nullptr;

    /** object */
    std::shared_ptr<std::vector<float>> sptr_object = nullptr;
    std::shared_ptr<std::vector<uint8_t>> sptr_objectColor = nullptr;
    std::shared_ptr<std::vector<Point>> sptr_objectPoints = nullptr;

    /** mutual exclusion */
    std::mutex m_mutex;
    std::shared_mutex s_mutex;

    /** flow-control semaphores */
    std::shared_ptr<bool> sptr_run;
    std::shared_ptr<bool> sptr_stop;
    std::shared_ptr<bool> sptr_isCalibrated;
    std::shared_ptr<bool> sptr_isKinectReady;
    std::shared_ptr<bool> sptr_isContextClustered;
    std::shared_ptr<bool> sptr_isEpsilonComputed;
    std::shared_ptr<bool> sptr_isContextSegmented;

    std::pair<Point, Point> m_segmentBoundary {};

    void setSegmentBoundary(std::pair<Point, Point>& boundary)
    {
        std::lock_guard<std::mutex> lck(m_mutex);
        m_segmentBoundary = boundary;
    }

    /** initialize API */
    explicit Intact(int& numPoints)
        : m_numPoints(numPoints)
    {
        /** initialize infinite boundary */
        Point lower(__FLT_MIN__, __FLT_MIN__, __FLT_MIN__);
        Point upper(__FLT_MAX__, __FLT_MAX__, __FLT_MAX__);

        m_segmentBoundary = { lower, upper };

        sptr_run = std::make_shared<bool>(false);
        sptr_stop = std::make_shared<bool>(false);
        sptr_isCalibrated = std::make_shared<bool>(false);
        sptr_isKinectReady = std::make_shared<bool>(false);
        sptr_isEpsilonComputed = std::make_shared<bool>(false);
        sptr_isContextClustered = std::make_shared<bool>(false);
        sptr_isContextSegmented = std::make_shared<bool>(false);

        sptr_raw = std::make_shared<std::vector<float>>(m_numPoints * 3);
        sptr_rawColor = std::make_shared<std::vector<uint8_t>>(m_numPoints * 3);
        sptr_rawPoints = std::make_shared<std::vector<Point>>(m_numPoints * 3);

        sptr_segment = std::make_shared<std::vector<float>>(m_numPoints * 3);
        sptr_segmentColor
            = std::make_shared<std::vector<uint8_t>>(m_numPoints * 3);
        sptr_segmentPoints
            = std::make_shared<std::vector<Point>>(m_numPoints * 3);

        sptr_region = std::make_shared<std::vector<float>>(m_numPoints * 3);
        sptr_regionColor
            = std::make_shared<std::vector<uint8_t>>(m_numPoints * 3);
        sptr_regionPoints
            = std::make_shared<std::vector<Point>>(m_numPoints * 3);

        sptr_object = std::make_shared<std::vector<float>>(m_numPoints * 3);
        sptr_objectColor
            = std::make_shared<std::vector<uint8_t>>(m_numPoints * 3);
        sptr_objectPoints
            = std::make_shared<std::vector<Point>>(m_numPoints * 3);
    }
    /**
     * segment
     *   Segments context.
     *
     * @param sptr_intact
     *   Instance of API call.
     */
    void segment(std::shared_ptr<Intact>& sptr_intact);

    static void calibrate(std::shared_ptr<Intact>& sptr_intact);

    /**
     * render
     *   Renders point cloud.
     *
     * @param sptr_intact
     *   Instance of API call.
     */
    static void render(std::shared_ptr<Intact>& sptr_intact);
    /**
     * cluster
     *   Clusters segmented context.
     *
     * @param E
     *   Epsilon parameter.
     *
     * @param N
     *   Number of epsilon-neighbourhood neighbours.
     *
     * @param sptr_intact
     *   Instance of API call.
     */
    static void cluster(
        const float& E, const int& N, std::shared_ptr<Intact>& sptr_intact);

    /**
     * estimateEpsilon
     *   Estimates size of epsilon neighbourhood using knn.
     *
     * @param K
     *   K parameter.
     *
     * @param sptr_intact
     *   Instance of API call.
     */
    static void estimateEpsilon(
        const int& K, std::shared_ptr<Intact>& sptr_intact);

    /** Thread-safe setters */
    void setObject(const std::vector<float>& points);

    void setRegion(const std::vector<float>& points);

    void setRawPoints(const std::vector<Point>& points);

    void setObjectColor(const std::vector<uint8_t>& color);

    void setObjectPoints(const std::vector<Point>& points);

    void setRegionPoints(const std::vector<Point>& points);

    void setRegionColor(const std::vector<uint8_t>& color);

    void setSegmentPoints(const std::vector<Point>& points);

    /** Thread-safe getters */

    int getNumPoints();

    std::shared_ptr<std::vector<float>> getRaw();

    std::shared_ptr<std::vector<float>> getRegion();

    std::shared_ptr<std::vector<float>> getSegment();

    std::shared_ptr<std::vector<Point>> getRawPoints();

    std::shared_ptr<std::vector<uint8_t>> getRawColor();

    std::shared_ptr<std::vector<Point>> getRegionPoints();

    std::shared_ptr<std::vector<uint8_t>> getRegionColor();

    std::shared_ptr<std::vector<Point>> getSegmentPoints();

    std::shared_ptr<std::vector<uint8_t>> getSegmentColor();

    /** Thread-safe semaphore queries */
    bool isRun();

    bool isStop();

    bool isSegmented();

    bool isCalibrated();

    bool isClustered();

    bool isKinectReady();

    bool isEpsilonComputed();

    /** Thread-safe semaphore control */
    void stop();

    void raiseRunFlag();

    void raiseStopFlag();

    void raiseCalibratedFlag();

    void raiseEpsilonFlag();

    void raiseSegmentedFlag();

    void raiseClusteredFlag();

    void raiseKinectReadyFlag();

    std::shared_ptr<std::vector<float>> getObject();

    std::shared_ptr<std::vector<uint8_t>> getObjectColor();

    std::pair<Point, Point> getSegmentBoundary();

    void setRaw(const std::vector<float>& pcl);

    void setRawColor(const std::vector<uint8_t>& color);

    void setSegment(const std::vector<float>& segment);

    void setSegmentColor(const std::vector<uint8_t>& segment);
};
#endif /* INTACT_H */
