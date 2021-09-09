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

    WAIT_FOR_SEGMENT
    std::vector<Point> pCloudSeg = *sptr_i3d->getPCloudSeg2x2Bin();

    /*
     * Do some work with the point cloud segment (pCloudSeg)
     */

    // output synchronous RGB-D segment
    int w = sptr_i3d->getDWidth();
    int h = sptr_i3d->getDHeight();
    int16_t* xyz = sptr_i3d->getXYZSeg()->data();
    uint8_t* rgba = sptr_i3d->getRGBASeg()->data();
    uint8_t* bgra = sptr_i3d->getBGRA()->data();
    io::write(w, h, bgra, "./output/segment.png");
    i3dpcl::write(w, h, xyz, rgba, "./output/segment.ply");

    sptr_i3d->stop();
    work.join();
    return 0;
}
