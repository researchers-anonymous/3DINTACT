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
    uint8_t chromakey[4] = { 27, 120, 55, 1 };
    while (RUN) {
        int w = sptr_i3d->getDWidth();
        int h = sptr_i3d->getDHeight();

        std::vector<Point> pCloud = *sptr_i3d->getPCloud();
        std::vector<Point> pCloudSeg = *sptr_i3d->getPCloudSeg();

        auto clusters = sptr_i3d->getPCloudClusters();
        auto clusteredPoints = clusters->first;
        auto clusteredPointIndexes = clusters->second;

        // clusters are sorted in descending order of size
        // in this example to extract the tabletop surface
        // we assume that the vacant space corresponds to
        // the largest cluster
        for(auto& index: clusteredPointIndexes[0]){
            pCloud[clusteredPoints[index].m_id].setBGRA(chromakey);
        }

        // stitch image using point cloud's RGBD data
        uint8_t bgraData[w * h * 4 ];
        for(int i = 0; i < pCloud.size(); i ++){
            i3dutils::stitch(i, pCloud[i], bgraData);
        }
        i3dutils::show(h, w, bgraData, sptr_i3d);
    }
    work.join();
    return 0;
}
