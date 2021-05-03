#include <chrono>
#include <string>
#include <thread>

#include "intact.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"

void sense(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    bool init = true;
    while (sptr_intact->isRun()) {
        /** get next frame: */
        sptr_kinect->getFrame(RGB_TO_DEPTH);
        sptr_intact->buildPcl(
            sptr_kinect->getPclImage(), sptr_kinect->getRgb2DepthImage());
        sptr_kinect->release();

        /** inform threads: init frame processing done */
        if (init) {
            init = false;
            sptr_intact->raiseKinectReadyFlag();
        }
        if (sptr_intact->isStop()) {
            sptr_intact->stop();
            // sptr_kinect->close(); //TODO: undefined behaviour?
        }
        // if (sptr_intact->isCalibrated()) {
        //     sptr_intact->stop();
        // }
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void calibrate(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    // todo: interface these calibration specifications
    //  const float arucoSquareEdgeLength = 0.0565f;           // in meters
    //  const float calibrationSquareEdgeLength = 0.02500f;    // in meters
    //  const std::string calibrationFile = "calibration.txt"; // external file
    //  for saving calibration
    sptr_intact->calibrate(sptr_kinect, sptr_intact);
}

void segment(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->segment(sptr_kinect, sptr_intact);
}

void render(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->render(sptr_kinect, sptr_intact);
}

void estimate(std::shared_ptr<Intact>& sptr_intact)
{
    const int K = 5; // <- kth nearest neighbour [ core + 4 nn ]
    sptr_intact->estimateEpsilon(K, sptr_intact);
}

void cluster(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    const float E = 3.310; // <- epsilon
    const int N = 4;       // <- min points in epsilon neighbourhood
    sptr_intact->cluster(E, N, sptr_intact);
}

int main(int argc, char* argv[])
{
    std::cout << "Press ESC to exit." << std::endl;
    logger(argc, argv);

    /** initialize kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    /** initialize API */
    std::shared_ptr<Intact> sptr_intact(new Intact(sptr_kinect->m_numPoints));
    sptr_intact->raiseRunFlag();

    /** start sensing */
    std::thread senseWorker(
        sense, std::ref(sptr_kinect), std::ref(sptr_intact));

    /** start sensing */ // todo: revision starts here
    std::thread calibrateWorker(
        calibrate, std::ref(sptr_kinect), std::ref(sptr_intact));

    /** segment in separate worker thread */
    std::thread segmentWorker(
        segment, std::ref(sptr_kinect), std::ref(sptr_intact));

    /** render in separate worker thread */
    std::thread renderWorker(
        render, std::ref(sptr_kinect), std::ref(sptr_intact));

    /** determine epsilon hyper-parameter */
    std::thread epsilonWorker(estimate, std::ref(sptr_intact));

    /** cluster interaction context  */
    std::thread clusterWorker(
        cluster, std::ref(sptr_kinect), std::ref(sptr_intact));

    /** join worker threads */
    senseWorker.join();
    segmentWorker.join();
    renderWorker.join();
    epsilonWorker.join();
    clusterWorker.join();
    calibrateWorker.join();

    /** grab image of scene */
    // io::write(sptr_kinect->m_rgbImage);
    return 0;
}
