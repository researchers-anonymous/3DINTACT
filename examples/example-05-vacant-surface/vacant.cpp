#include <thread>

#include "i3d.h"
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
    auto clusters = sptr_i3d->getPCloudClusters();
    auto clusteredPoints = clusters->first;
    auto clusteredPointIndexes = clusters->second;

    // initialize random color
    uint8_t rgba[4] = { 69, 117, 180, 1 };

    // clusters are sorted in descending order of size
    // in this example, to extract the tabletop surface
    // we assume that the vacant space corresponds to
    // the largest cluster
    std::vector<Point> vacant;
    for (auto& index : clusteredPointIndexes[0]) {
        clusteredPoints[index].setRGBA(rgba);
        vacant.emplace_back(clusteredPoints[index]);
    }
    i3dpcl::write(vacant, "./output/vacant-surface.ply");
    sptr_i3d->stop();
    work.join();
    return 0;
}
