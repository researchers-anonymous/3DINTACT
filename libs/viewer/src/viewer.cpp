#include <pangolin/gl/glvbo.h>
#include <pangolin/pangolin.h>
#define TINYPLY_IMPLEMENTATION

#include "tinyply.h"

#include "i3d.h"
#include "viewer.h"

int mode = 0;
void view()
{
    mode++;
    if (mode == 2) {
        mode = 0;
    }
}

void viewer::render(std::shared_ptr<I3d>& sptr_i3d)
{
    // get dimensions
    int w = sptr_i3d->getDepthWidth();
    int h = sptr_i3d->getDepthHeight();

    /** create window and bind its context to the main thread */
    pangolin::CreateWindowAndBind("VIGITIA", 2560, 1080);

    /** initialize glew */
    glewInit();

    /**  enable mouse handler with depth testing */
    glEnable(GL_DEPTH_TEST);

    /** create vertex and colour buffer objects and register them with CUDA */
    pangolin::GlBuffer vA(
        pangolin::GlArrayBuffer, w * h, GL_SHORT, 3, GL_STATIC_DRAW);
    pangolin::GlBuffer cA(
        pangolin::GlArrayBuffer, w * h, GL_UNSIGNED_BYTE, 4, GL_STATIC_DRAW);

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

    /** pool resources, and render */
    int16_t* ptr_pCloudFrame;
    uint8_t* ptr_imgFrame;

    while (!sptr_i3d->isStop()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (mode == 0) {
            ptr_pCloudFrame = sptr_i3d->getPCloudFrame()->data();
            ptr_imgFrame = sptr_i3d->getImgFrame_GL()->data();
        } else if (mode == 1) {
            ptr_pCloudFrame = sptr_i3d->getPCloudSegFrame()->data();
            ptr_imgFrame = sptr_i3d->getImgSegFrame_GL()->data();
        }
        // else if (mode == 2) {
        //     auto clusters = sptr_i3d->getColClusters();
        //     ptr_pCloudFrame = clusters->first;
        //     ptr_imgFrame = clusters->second;
        // }

        vA.Upload((void*)ptr_pCloudFrame, w * h * 3 * sizeof(int16_t));
        cA.Upload((void*)ptr_imgFrame, w * h * 4 * sizeof(uint8_t));

        viewPort.Activate(camera);
        glClearColor(0.0, 0.0, 0.3, 1.0);
        pangolin::glDrawAxis(4000.f);
        pangolin::RenderVboCbo(vA, cA);
        pangolin::FinishFrame();

        /** gracious exit from rendering app */
        if (pangolin::ShouldQuit()) {
            sptr_i3d->raiseStopFlag();
        }
    }
}
