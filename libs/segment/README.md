### :mortar_board: :mortar_board: :mortar_board: :mortar_board:
#### a tiny library for segmenting a 3D point cloud and extracting an orthogonal planar surface

*   the interface

```cpp

#include <vector>
#include "point.h"

namespace region {

/** segment
 *   Segments a 3D point cloud and extracts an
 *   orthogonal planar surface.
 *
 * @param points
 *   The given set of 3D points
 *
 * @retval
 *   Set of 3D points corresponding to the
 *   orthogonal planar surface
 */
std::vector<Point> segment(std::vector<Point>& points);

}

```

* usage example

```cpp
#include <vector>

#include "point.h"
#include "region.h" // <-- include the library

int main(int argc, char* argv[]){

    logger(argc, argv);
    std::vector<Point> points = readPoints();
    std::vector<Point> pCloudSeg = region::segment(points);
    write(pCloudSeg);

    return 0;
}

```

######   Checkout:

* a flexible `point.h` header [here](https://github.com/edisonslightbulbs/point).
* the [`outliers`](https://github.com/edisonslightbulbs/outliers)  submodule dependency
* the [`svd`](https://github.com/edisonslightbulbs/svd)  submodule dependency
