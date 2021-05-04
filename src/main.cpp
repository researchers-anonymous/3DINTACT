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
    // Here is the interesting bit:
    // Your sensor of choice (which needn't be the Kinect) should
    // handshake 3DINTACT's API. No adapter is provided for
    // casting different point-cloud data structures into the
    // structure required by the API. This has to be done manually
    // per specific case. The only important thing to note is the
    // API requires and expects two std vectors:
    //   1) std::vector<float> points;
    //      where points =
    //      { x_0, y_0, z_0, x_1, y_1, z_1, ..., x_n, y_n, z_n }
    //   2) std::vector<uint8_t> color;
    //      where color =
    //      { r_0, g_0, b_0, r_1, g_1, b_1, ..., r_n, g_n, b_n }
    //
    bool init = true;
    while (sptr_intact->isRun()) {

        /** capture point cloud */
        sptr_kinect->getFrame(RGB_TO_DEPTH);

        /** hand over point cloud to API */
        auto* data
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->getPclImage());
        uint8_t* color = k4a_image_get_buffer(sptr_kinect->getRgb2DepthImage());

        const int N = sptr_intact->getNumPoints();

        /** raw point cloud */
        std::vector<float> raw(N * 3);
        std::vector<uint8_t> rawColor(N * 3);

        /** segmented interaction regions */
        std::vector<float> segment(N * 3);
        std::vector<uint8_t> segmentColor(N * 3);

        /** largest interaction region:
         *   vacant tabletop surface spaces */
        std::vector<float> region(N * 3);
        std::vector<uint8_t> regionColor(N * 3);

        /** n.b., kinect colors reversed! */
        for (int i = 0; i < N; i++) {
            if (data[3 * i + 2] == 0) {
                raw[3 * i + 0] = 0.0f;
                raw[3 * i + 1] = 0.0f;
                raw[3 * i + 2] = 0.0f;
                rawColor[3 * i + 2] = color[4 * i + 0];
                rawColor[3 * i + 1] = color[4 * i + 1];
                rawColor[3 * i + 0] = color[4 * i + 2];
                continue;
            }
            raw[3 * i + 0] = (float)data[3 * i + 0];
            raw[3 * i + 1] = (float)data[3 * i + 1];
            raw[3 * i + 2] = (float)data[3 * i + 2];
            rawColor[3 * i + 2] = color[4 * i + 0];
            rawColor[3 * i + 1] = color[4 * i + 1];
            rawColor[3 * i + 0] = color[4 * i + 2];

            /** segment interaction regions  */
            if (sptr_intact->getSegmentBoundary().second.m_xyz[2] == __FLT_MAX__
                || sptr_intact->getSegmentBoundary().first.m_xyz[2]
                    == __FLT_MIN__) {
                continue;
            }

            if ((float)data[3 * i + 0]
                    > sptr_intact->getSegmentBoundary().second.m_xyz[0]
                || (float)data[3 * i + 0]
                    < sptr_intact->getSegmentBoundary().first.m_xyz[0]
                || (float)data[3 * i + 1]
                    > sptr_intact->getSegmentBoundary().second.m_xyz[1]
                || (float)data[3 * i + 1]
                    < sptr_intact->getSegmentBoundary().first.m_xyz[1]
                || (float)data[3 * i + 2]
                    > sptr_intact->getSegmentBoundary().second.m_xyz[2]
                || (float)data[3 * i + 2]
                    < sptr_intact->getSegmentBoundary().first.m_xyz[2]) {
                continue;
            }
            segment[3 * i + 0] = (float)data[3 * i + 0];
            segment[3 * i + 1] = (float)data[3 * i + 1];
            segment[3 * i + 2] = (float)data[3 * i + 2];
            segmentColor[3 * i + 2] = color[4 * i + 0];
            segmentColor[3 * i + 1] = color[4 * i + 1];
            segmentColor[3 * i + 0] = color[4 * i + 2];
        }
        sptr_intact->setRaw(raw);
        sptr_intact->setRawColor(rawColor);

        if (sptr_intact->getSegmentBoundary().second.m_xyz[2] == __FLT_MAX__
            || sptr_intact->getSegmentBoundary().first.m_xyz[2]
                == __FLT_MIN__) {
            sptr_intact->setSegment(raw);
            sptr_intact->setSegmentColor(rawColor);
        } else {
            sptr_intact->setSegment(segment);
            sptr_intact->setSegmentColor(segmentColor);
        }

        /** release kinect resources */
        sptr_kinect->release();

        /** update initialization semaphore */
        if (init) {
            init = false;
            sptr_intact->raiseKinectReadyFlag();
        }
        if (sptr_intact->isStop()) {
            sptr_intact->stop();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void calibrate(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->calibrate(sptr_intact);
}

void segment(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->segment(sptr_intact);
}

void render(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->render(sptr_intact);
}

void estimate(std::shared_ptr<Intact>& sptr_intact)
{
    const int K = 5; // <- kth nearest neighbour [ core + 4 nn ]
    sptr_intact->estimateEpsilon(K, sptr_intact);
}

void cluster(std::shared_ptr<Intact>& sptr_intact)
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

    /** sense */
    std::thread calibrateWorker(calibrate, std::ref(sptr_intact));

    /** segment */
    std::thread segmentWorker(segment, std::ref(sptr_intact));

    /** render */
    std::thread renderWorker(render, std::ref(sptr_intact));

    /** find epsilon */
    std::thread epsilonWorker(estimate, std::ref(sptr_intact));

    /** cluster */
    std::thread clusterWorker(cluster, std::ref(sptr_intact));

    /** wait for segmenting to compete ~15ms */
    while (!sptr_intact->isSegmented()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    // ------> do stuff with segmented region of interest here <------
    sptr_intact->getSegment();      // std::make_shared<std::vector<float>>
    sptr_intact->getSegmentColor(); // std::make_shared<std::vector<uint8_t>>
    //
    // CAVEAT:
    // Make sure access to the raw point cloud is thread safe.
    // If unsure how, the API provides thread-safe access to the raw point cloud
    //
    sptr_intact->getRaw();      // std::make_shared<std::vector<float>>
    sptr_intact->getRawColor(); // std::make_shared<std::vector<uint8_t>>
    // ------> do stuff with segmented region of interest here <------

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
