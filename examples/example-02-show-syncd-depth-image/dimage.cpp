#include <thread>

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

    // write synchronized depth image
    WAIT_FOR_C2D_TRANSFORMATION
    int w = sptr_i3d->getDWidth();
    int h = sptr_i3d->getDHeight();
    uint8_t* bgra = sptr_i3d->getBGRA()->data();
    io::write(w, h, bgra, "./output/synchronized-depth-image.png");

    sptr_i3d->stop();
    work.join();
    return 0;
}
