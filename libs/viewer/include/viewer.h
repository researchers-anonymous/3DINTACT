#ifndef VIEWER_H
#define VIEWER_H

#include <memory>
#include <vector>

#include "i3d.h"
#include "kinect.h"

namespace viewer {
void render(std::shared_ptr<I3d>& sptr_i3d);
}
#endif /* VIEWER_H */
