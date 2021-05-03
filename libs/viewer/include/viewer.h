#ifndef RENDERER_H
#define RENDERER_H

#include <memory>
#include <vector>

#include "intact.h"
#include "kinect.h"

namespace viewer {
void draw(
    std::shared_ptr<Intact>& sptr_intact, std::shared_ptr<Kinect>& sptr_kinect);
}
#endif /* RENDERER_H */
