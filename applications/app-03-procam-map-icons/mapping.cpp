#include <opencv2/core/mat.hpp>
#include <thread>
#include <torch/script.h>

#include "i3d.h"
#include "i3dscene.h"
#include "icon.h"
#include "io.h"
#include "kinect.h"
#include "object.h"
#include "od.h"
#include "surface.h"
#include "usage.h"

void loadIcons(cv::Mat& spotifyIcon, cv::Mat& discordIcon, cv::Mat& facebookIcon)
{
    spotifyIcon = icon::load("./resources/icons/discord.png");
    discordIcon = icon::load("./resources/icons/spotify.png");
    facebookIcon = icon::load("./resources/icons/facebook.png");

    // saturate icons to make them a bit clearer
    int beta = 0;       // brightness | range 1 - 100
    double alpha = 3.0; //   contrast | range 1.0 - 3.0
    icon::saturate(spotifyIcon, beta, alpha);
    icon::saturate(discordIcon, beta, alpha);
    icon::saturate(facebookIcon, beta, alpha);

    cv::rotate(spotifyIcon, spotifyIcon, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(discordIcon, discordIcon, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(facebookIcon, facebookIcon, cv::ROTATE_90_COUNTERCLOCKWISE);

    // scale icons to enlarge them
    int scaleWidth = 10;
    int scaleHeight = 10;
    icon::scale(spotifyIcon, scaleWidth, scaleHeight);
    icon::scale(discordIcon, scaleWidth, scaleHeight);
    icon::scale(facebookIcon, scaleWidth, scaleHeight);
}

void overlayBackground(cv::Mat& frame, cv::Mat& copy, const std::vector<object_t>& objects, const int& height, const int& width, cv::Mat& spotifyIcon, cv::Mat& discordIcon, cv::Mat& facebookIcon)
{
    // black background
    frame = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    // create rectangles using icon size
    int w = spotifyIcon.cols;
    int h = spotifyIcon.rows;

    // declare icon rectangle
    cv::Rect spotifyIconRegion;

    cv::Rect roi;
    int xShift = -10; // vertical
    int yShift = +10; // horizontal

    // specifiy i
    if (!objects.empty()) {
        roi = objects[0].m_bbox;
        spotifyIconRegion = cv::Rect((roi.x + xShift), (roi.y + yShift), w, h);

        // get a sense of orientation of icon placement
        std::cout << "icon placed at: x, y, w, h  = " << (roi.x + xShift) << "  " << (roi.y + yShift) << "  " << w << "  " << h << std::endl;

        // get roi from background image
        cv::Mat spotifyIconOverlayRegion = frame(spotifyIconRegion);

        // place icons on roi
        spotifyIcon.copyTo(spotifyIconOverlayRegion);
    }
}

cv::Mat grabFrame(std::shared_ptr<i3d>& sptr_i3d)
{
    int w = sptr_i3d->getDWidth();
    int h = sptr_i3d->getDHeight();
    uint8_t* c2dImage = *sptr_i3d->getC2DBGRAData();
    return cv::Mat(h, w, CV_8UC4, (void*)c2dImage, cv::Mat::AUTO_STEP).clone();
}

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

    // initialize object detection resources
    torch::jit::script::Module module;
    std::vector<std::string> classnames;
    std::string torchscript = io::pwd() + "/resources/best.pt";
    std::string classes = io::pwd() + "/resources/class.names";
    od::setup(classnames, module, torchscript, classes);

    // setup projector window
    std::vector<cv::Mat> captures;
    int w = 1366;
    int h = 768;
    const std::string window = "re-projection window";
    surface::contrast(sptr_i3d, window, w, h, captures);

    // load icons and black background
    cv::Mat spotifyIcon, discordIcon, facebookIcon;
    loadIcons(spotifyIcon, discordIcon, facebookIcon);

    // find region of interest / projection area on surface
    cv::Rect roi;
    roi = surface::reg(captures[1], captures[0]);
    cv::Mat scene = captures[1];

    // get projection area
    // todo scene::undistort(background);
    cv::Mat aop = scene(roi);
    cv::Mat R, t;
    // todo scene::predistort(background);

    while (RUN) {
        // get image frame and clone it
        cv::Mat frame = grabFrame(sptr_i3d);
        cv::Mat copy = frame.clone();

        // detect mobile devices
        std::vector<object_t> objects = od::detect(h, w, copy, classnames, module, sptr_i3d);

        // overlay icons near mobile devices using detected objects
        overlayBackground(frame, copy, objects, h, w, spotifyIcon, discordIcon, facebookIcon);

        // crop area of projection
        aop = frame(roi);

        // transform and project surface overlay
        surface::project(window, w, h, aop, R, t);

        // show frame copy (untransformed) with object detection
        for (auto& object : objects) {
            copy = object.m_image;
            cv::rectangle(copy, object.m_bbox, cv::Scalar(0, 255, 0), 1);
            cv::putText(copy, object.m_label, object.m_bboxOrigin, cv::FONT_HERSHEY_SIMPLEX, (double)(object.m_imageWidth) / 200, cv::Scalar(0, 255, 0), 1);
        }
        cv::imshow("object detection", copy);

        // press ESC to close window/app
        if (cv::waitKey(1) == 27) {
            sptr_i3d->stop();
        }
    }
    work.join();
    return 0;
}
