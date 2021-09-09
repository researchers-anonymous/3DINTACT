#include <opencv2/core/mat.hpp>
#include <thread>
#include <torch/script.h>

#include "i3d.h"
#include "i3dscene.h"
#include "i3dutils.h"
#include "io.h"
#include "kinect.h"
#include "object.h"
#include "od.h"
#include "usage.h"

void show(std::shared_ptr<i3d>& sptr_i3d, const std::vector<object_t>& objects)
{
    cv::Mat img;
    for (auto& object : objects) {
        img = object.m_image;
        cv::rectangle(img, object.m_boundingBox, cv::Scalar(0, 255, 0), 1);
        cv::putText(img, object.m_label, object.m_boundingBoxOrigin, cv::FONT_HERSHEY_SIMPLEX, (double)(object.m_imageWidth) / 200, cv::Scalar(0, 255, 0), 1);
    }

    cv::imshow("i3d", img);
    if (cv::waitKey(1) == 27) {
        sptr_i3d->stop();
    }
}

int main(int argc, char* argv[])
{
    // init logger, kinect, and i3d
    logger(argc, argv);
    usage::prompt(ABOUT);
    std::shared_ptr<kinect> sptr_kinect(new kinect);
    std::shared_ptr<i3d> sptr_i3d(new i3d());
    std::thread work(
        i3dscene::context, std::ref(sptr_kinect), std::ref(sptr_i3d));

    // initialize object detection resources
    torch::jit::script::Module module;
    std::vector<std::string> classnames;
    std::string torchscript = io::pwd() + "/resources/best.pt";
    std::string classes = io::pwd() + "/resources/class.names";
    od::setup(classnames, module, torchscript, classes);

    // segment and cluster interaction regions in fish tank
    WAIT_FOR_CLUSTERS

    // process synchronized point cloud and images
    while (RUN) {
        int w = sptr_i3d->getDWidth();
        int h = sptr_i3d->getDHeight();

        std::vector<Point> pCloud = *sptr_i3d->getPCloud();
        uint8_t* pCloudSynchImage = *sptr_i3d->getC2DBGRAData();

        uint8_t processedImage[w * h * 4];
        for (int i = 0; i < pCloud.size(); i++) {
            // do some operation of images using RGBD here
            // ++
            i3dutils::stitch(i, pCloud[i], processedImage);
        }
        std::vector<object_t> objects = od::detect(h, w, pCloudSynchImage, classnames, module, sptr_i3d);
        show(sptr_i3d, objects);
    }
    work.join();
    return 0;
}
