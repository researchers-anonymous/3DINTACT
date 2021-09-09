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

    // write non-color point cloud | fast 2x2 Bin
    WAIT_FOR_FAST_POINTCLOUD
    std::vector<Point> pCloud = *sptr_i3d->getPCloud2x2Bin();
    i3dpcl::write(pCloud, "./output/gray.ply");

    // write color point cloud
    WAIT_FOR_C2D_POINTCLOUD
    int w = sptr_i3d->getDWidth();
    int h = sptr_i3d->getDHeight();
    int16_t* xyz = sptr_i3d->getXYZ()->data();
    uint8_t* rgba = sptr_i3d->getRGBA()->data();
    i3dpcl::write(w, h, xyz, rgba, "./output/color.ply");

    sptr_i3d->stop();
    work.join();
    return 0;
}
