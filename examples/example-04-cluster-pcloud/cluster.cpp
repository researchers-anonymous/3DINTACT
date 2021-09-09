#include <thread>

#include "i3d.h"
#include "i3dutils.h"
#include "i3dpcl.h"
#include "i3dscene.h"
#include "kinect.h"
#include "usage.h"

int main(int argc, char* argv[])
{
    // init logger, kinect, and i3d
    logger(argc, argv);
    usage::prompt(ABOUT);
    std::shared_ptr<kinect> sptr_kinect(new kinect);
    std::shared_ptr<i3d> sptr_i3d(new i3d());
    std::thread work(
            i3dscene::context, std::ref(sptr_kinect), std::ref(sptr_i3d));

    WAIT_FOR_CLUSTERS
    std::vector<Point> pCloudSeg = *sptr_i3d->getPCloudSeg();
    std::vector<Point> pCloudSeg2x2Bin = *sptr_i3d->getPCloudSeg2x2Bin();

    /*
     * Do stuff with the point cloud segment
     */

    auto clusters = sptr_i3d->getPCloudClusters();
    auto clusteredPoints = clusters->first;
    auto clusteredPointIndexes = clusters->second;

    /*
     * Do stuff with clusters:
     *   n.b., a Point's cluster is indicated by
     *         the class member point.m_cluster
     */

    i3dpcl::write(clusteredPoints, "./output/pcloud.ply");

    // colorize clusters: visualize clusters
    std::vector<uint8_t*> rgba;
    i3dutils::add(rgba);
    for (auto& indexCluster : clusteredPointIndexes) {
        int colIndex = i3dutils::randNum((int)rgba.size());
        for (auto& index : indexCluster) {
            clusteredPoints[index].setRGBA(rgba[colIndex]);
        }
    }
    i3dpcl::write(clusteredPoints, "./output/colorized-clusters.ply");
    sptr_i3d->stop();
    work.join();
    return 0;
}
