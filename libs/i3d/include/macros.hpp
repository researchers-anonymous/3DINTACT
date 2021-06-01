#ifndef INTACT_MACROS_HPP
#define INTACT_MACROS_HPP

#include "logger.h"
#include "ply.h"
#include "timer.h"

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

#define SLEEP_UNTIL_POINTCLOUD_READY                                           \
    while (!sptr_i3d->isPCloudReady()) {                                       \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define RAISE_POINTCLOUD_READY_FLAG                                            \
    if (init) {                                                                \
        init = false;                                                          \
        LOG(INFO) << "-- building point cloud";                                \
        sptr_i3d->raisePCloudReadyFlag();                                      \
    }

#define SLEEP_UNTIL_PROPOSAL_READY_FLAG                                        \
    while (!sptr_i3d->isProposalReady()) {                                     \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define RAISE_PROPOSAL_READY_FLAG                                              \
    if (init) {                                                                \
        init = false;                                                          \
        LOG(INFO) << "-- proposing region ";                                   \
        sptr_i3d->raiseProposalReadyFlag();                                    \
    }

#define SLEEP_UNTIL_SEGMENT_READY                                              \
    while (!sptr_i3d->isSegmented()) {                                         \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define RAISE_SEGMENT_READY_FLAG                                               \
    if (init) {                                                                \
        init = false;                                                          \
        LOG(INFO) << "-- segmenting region";                                   \
        sptr_i3d->raiseSegmentedFlag();                                        \
    }

#define SLEEP_UNTIL_CLUSTERS_READY                                             \
    while (!sptr_i3d->isClustered()) {                                         \
        std::this_thread::sleep_for(std::chrono::milliseconds(3));             \
    }

#define RAISE_CLUSTERS_READY_FLAG                                              \
    if (init) {                                                                \
        init = false;                                                          \
        LOG(INFO) << "-- clustering region";                                   \
        sptr_i3d->raiseClusteredFlag();                                        \
    }

#define EXIT_CALLBACK                                                          \
    if (sptr_i3d->isStop()) {                                                  \
        sptr_i3d->stop();                                                      \
    }

#define STOP sptr_i3d->raiseStopFlag();

#define START bool init = true;

#define PRINT(pCloud) ply::write(pCloud);

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

// todo: introduce macro configuration logic

#endif /*INTACT_MACROS_HPP*/
