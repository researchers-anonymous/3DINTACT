#include <thread>
#include <knn.h>

#include "i3d.h"
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

    WAIT_FOR_PROPOSAL
    std::vector<Point> pCloudSeg = *sptr_i3d->getPCloudSeg2x2Bin();
    std::vector<Point> queryPoints = pCloudSeg;

    // heap the distance to the 4th nearest
    // neighbour of each segment point
    std::vector<float> heapDists = knn::compute(pCloudSeg, queryPoints, 4);

    // sort heaped distances and output them for plotting
    // once plot, use the value that corresponds to the
    // 'elbow curve' as a proxy to the epsilon parameter
    std::sort(heapDists.begin(), heapDists.end(), std::greater<>());
    io::write(heapDists, "./output/epsilon-candidates.csv");
    sptr_i3d->stop();
    work.join();
    return 0;
}
