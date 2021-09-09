#include <thread>

#include "i3d.h"
#include "i3dscene.h"
#include "io.h"
#include "kinect.h"
#include "surface.h"
#include "usage.h"

int main(int argc, char* argv[])
{
    // init logger, kinect, and i3d
    logger(argc, argv);
    usage::prompt(ABOUT);
    std::shared_ptr<kinect> sptr_kinect(new kinect);
    std::shared_ptr<i3d> sptr_i3d(new i3d());

    // start i3d worker
    std::thread work(
        i3dscene::context, std::ref(sptr_kinect), std::ref(sptr_i3d));

    std::vector<cv::Mat> captures;

    // setup projector window
    int proWinWidth = 1366;
    int proWinHeight = 768;
    const std::string window = "re-projection window";
    surface::contrast(sptr_i3d, window, proWinWidth, proWinHeight, captures);

    // find region of interest / projection area on surface
    cv::Rect roi;
    roi = surface::reg(captures[1], captures[0]);
    cv::Mat scene = captures[1];

    // todo scene::undistort(background);
    cv::Mat projection = scene(roi);
    // todo reproject aop
    cv::Mat R, t; // todo
    // todo scene::predistort(background);

    while (RUN) {
        int w = sptr_i3d->getDWidth();
        int h = sptr_i3d->getDHeight();
        uint8_t* c2dImage = *sptr_i3d->getC2DBGRAData();
        cv::Mat frame = cv::Mat(h, w, CV_8UC4, (void*)c2dImage, cv::Mat::AUTO_STEP).clone();

        //projection = projectionArea(sptr_i3d, roi);
        projection = frame(roi);
        surface::project(window, proWinWidth, proWinHeight, projection, R, t);
        if (cv::waitKey(1) == 27) {
            sptr_i3d->stop();
        }
    }
    work.join();
    return 0;
}
