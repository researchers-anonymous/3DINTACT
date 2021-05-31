#ifndef INTACT_MACROS_HPP
#define INTACT_MACROS_HPP

#include "logger.h"
#include "ply.h"
#include "timer.h"

// colors handy for coloring clusters:
// see@ https://colorbrewer2.org/#type=diverging&scheme=RdYlBu&n=9
//
__attribute__((unused)) constexpr uint8_t red[3] = { 215, 48, 39 };
__attribute__((unused)) constexpr uint8_t orange[3] = { 244, 109, 67 };
__attribute__((unused)) constexpr uint8_t gold[3] = { 253, 173, 97 };
__attribute__((unused)) constexpr uint8_t brown[3] = { 254, 224, 144 };
__attribute__((unused)) constexpr uint8_t yellow[3] = { 255, 255, 191 };
__attribute__((unused)) constexpr uint8_t skyblue[3] = { 224, 243, 248 };
__attribute__((unused)) constexpr uint8_t oceanblue[3] = { 171, 217, 233 };
__attribute__((unused)) constexpr uint8_t blue[3] = { 116, 173, 209 };
__attribute__((unused)) constexpr uint8_t deepblue[3] = { 69, 117, 180 };

// colors handy for coloring clusters:
// https://colorbrewer2.org/#type=diverging&scheme=BrBG&n=9 */
//
__attribute__((unused)) constexpr uint8_t depbrown[3] = { 140, 81, 10 };
__attribute__((unused)) constexpr uint8_t darkbrown[3] = { 191, 129, 45 };
__attribute__((unused)) constexpr uint8_t goldenbrown[3] = { 223, 194, 125 };
__attribute__((unused)) constexpr uint8_t khaki[3] = { 223, 232, 195 };
__attribute__((unused)) constexpr uint8_t lightgrey[3] = { 245, 245, 245 };
__attribute__((unused)) constexpr uint8_t lightgreen[3] = { 199, 234, 229 };
__attribute__((unused)) constexpr uint8_t green[3] = { 128, 205, 193 };
__attribute__((unused)) constexpr uint8_t chromagreen[3] = { 120, 198, 121 };
__attribute__((unused)) constexpr uint8_t deepgreen[3] = { 53, 151, 143 };
__attribute__((unused)) constexpr uint8_t darkgreen[3] = { 1, 102, 94 };
__attribute__((unused)) constexpr uint8_t black[3] = { 0, 0, 0 };

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
