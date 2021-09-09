#ifndef INTACT_MACROS_HPP
#define INTACT_MACROS_HPP

#include "i3dpcl.h"
#include "i3dtimer.h"
#include "logger.h"

#define SLEEP_UNTIL_SENSOR_DATA_READY                                          \
    while (!sptr_i3d->isSensorReady()) {                                       \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define RAISE_SENSOR_RESOURCES_READY_FLAG                                      \
    if (init) {                                                                \
        init = false;                                                          \
        LOG(INFO) << "-- k4a driver running";                                  \
        sptr_i3d->raiseSensorReadyFlag();                                      \
    }

#define WAIT_FOR_FAST_POINTCLOUD                                               \
    while (!sptr_i3d->isPCloudReady()) {                                       \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define RAISE_POINTCLOUD_READY_FLAG                                            \
    if (init) {                                                                \
        init = false;                                                          \
        LOG(INFO) << "-- building point cloud";                                \
        sptr_i3d->raisePCloudReadyFlag();                                      \
    }

#define WAIT_FOR_PROPOSAL                                                      \
    while (!sptr_i3d->isProposalReady()) {                                     \
        std::this_thread::sleep_for(std::chrono::microseconds(1));             \
    }

#define FAST_POINTCLOUD_READY                                                  \
    if (init) {                                                                \
        init = false;                                                          \
        LOG(INFO) << "-- proposing region ";                                   \
        sptr_i3d->raiseProposalReadyFlag();                                    \
    }

#define WAIT_FOR_C2D_POINTCLOUD                                                \
    while (!sptr_i3d->isSegmented()) {                                         \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define WAIT_FOR_C2D_TRANSFORMATION                                            \
    while (!sptr_i3d->isSegmented()) {                                         \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define WAIT_FOR_SEGMENT                                                       \
    while (!sptr_i3d->isSegmented()) {                                         \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define SEGMENT_READY                                                          \
    if (init) {                                                                \
        init = false;                                                          \
        LOG(INFO) << "-- segmenting region";                                   \
        sptr_i3d->raiseSegmentedFlag();                                        \
    }

#define WAIT_FOR_CLUSTERS                                                      \
    while (!sptr_i3d->isClustered()) {                                         \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define CLUSTERS_READY                                                         \
    if (init) {                                                                \
        init = false;                                                          \
        LOG(INFO) << "-- clustering region";                                   \
        sptr_i3d->raiseClusteredFlag();                                        \
    }

#define EXIT_CALLBACK                                                          \
    if (sptr_i3d->isStop()) {                                                  \
        sptr_i3d->stop();                                                      \
    }

#define START bool init = true;
#define RUN sptr_i3d->isRun() && !sptr_i3d->isStop()

// #define STOP                                                                   \
//     sptr_i3d->stop();                                                          \
//     sptr_i3d->raiseStopFlag();                                                 \
//     std::this_thread::sleep_for(std::chrono::seconds(3));                      \
//     exit(0);

#define BENCHMARK 0
#if BENCHMARK == 1
#define START_TIMER Timer timer;

#define STOP_TIMER(str) LOG(INFO) << str << timer.getDuration() << " ms";
#else
#define START_TIMER
#define STOP_TIMER(str)
#endif

#define BUILD_POINTCLOUD 1
#define PROPOSE_REGION 1
#define SEGMENT_REGION 1
#define CLUSTER_REGION 1

#endif /*INTACT_MACROS_HPP*/
