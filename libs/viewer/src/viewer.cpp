#include <pangolin/gl/glvbo.h>
#include <pangolin/pangolin.h>
#define TINYPLY_IMPLEMENTATION

#include "tinyply.h"

#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <thread>

#include "intact.h"
#include "viewer.h"

int mode = 0;

void view()
{
    mode++;
    if (mode == 4) {
        mode = 0;
    }
}

void viewer::draw(std::shared_ptr<Intact>& sptr_intact)
{
    /** create window and bind its context to the main thread */
    pangolin::CreateWindowAndBind("VIGITIA", 2560, 1080);

    /** initialize glew */
    glewInit();

    /**  enable mouse handler with depth testing */
    glEnable(GL_DEPTH_TEST);

    /** create vertex and colour buffer objects and register them with CUDA */
    pangolin::GlBuffer vA(pangolin::GlArrayBuffer, sptr_intact->getNumPoints(),
        GL_FLOAT, 3, GL_STATIC_DRAW);
    pangolin::GlBuffer cA(pangolin::GlArrayBuffer, sptr_intact->getNumPoints(),
        GL_UNSIGNED_BYTE, 3, GL_STATIC_DRAW);

    /** define camera render object for scene browsing */
    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(2560, 1080, 800, 800, 1280, 540, 0.1, 10000),
        ModelViewLookAt(-0, 2, -2, 0, 0, 0, pangolin::AxisY));
    const int UI_WIDTH = 180;

    /** add named OpenGL viewport to window and provide 3D handler */
    pangolin::View& viewPort
        = pangolin::Display("cam")
              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0,
                  -640.0f / 480.0f)
              .SetHandler(new pangolin::Handler3D(camera));

    // register key press to trigger different view perspective
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'c', view);

    /** render point cloud */
    while (!sptr_intact->isStop()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (mode == 0) {
            vA.Upload((void*)sptr_intact->getRaw()->data(),
                sptr_intact->getNumPoints() * 3 * sizeof(float));
            cA.Upload((void*)sptr_intact->getRawColor()->data(),
                sptr_intact->getNumPoints() * 3 * sizeof(uint8_t));
        } else if (mode == 1) {
            vA.Upload((void*)sptr_intact->getSegment()->data(),
                sptr_intact->getNumPoints() * 3 * sizeof(float));
            cA.Upload((void*)sptr_intact->getSegmentColor()->data(),
                sptr_intact->getNumPoints() * 3 * sizeof(uint8_t));
        } else if (mode == 2) {
            vA.Upload((void*)sptr_intact->getRegion()->data(),
                sptr_intact->getNumPoints() * 3 * sizeof(float));
            cA.Upload((void*)sptr_intact->getRegionColor()->data(),
                sptr_intact->getNumPoints() * 3 * sizeof(uint8_t));
        } else if (mode == 3) {
            vA.Upload((void*)sptr_intact->getObject()->data(),
                sptr_intact->getNumPoints() * 3 * sizeof(float));
            cA.Upload((void*)sptr_intact->getObjectColor()->data(),
                sptr_intact->getNumPoints() * 3 * sizeof(uint8_t));
        }

        viewPort.Activate(camera);
        glClearColor(0.0, 0.0, 0.3, 1.0);
        pangolin::glDrawAxis(4000.f);
        pangolin::RenderVboCbo(vA, cA);
        pangolin::FinishFrame();

        /** gracious exit from rendering app */
        if (pangolin::ShouldQuit()) {
            sptr_intact->raiseStopFlag();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
