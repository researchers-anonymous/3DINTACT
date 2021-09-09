#### Usage of this tiny knn lib based on [nanoflann](https://github.com/jlblancoc/nanoflann)

Massive thanks to [Jose](https://github.com/jlblancoc) for sharing this adaptation of [flann](https://github.com/mariusmuja/flann) 👏🍻🍻

*   the interface

```cpp
#include <vector>
#include "point.h"

namespace knn {

/** compute
 *    Computes the K nearest neighbours of a queryPoint.
 *
 *  @param points
 *    Set of 3D points (x, y, z).
 *
 *  @param k
 *    K value.
 *
 *  @param queryPoint
 *    query point
 *
 *  @retval
 *    The indexes of the nearest neighbors from closest
 *    neighbor to furthest neighbour
 * */
std::vector<int> compute(
    std::vector<Point>& points, const Point& queryPoint, const int& k);

/** compute
 *    Computes the K nearest neighbours of a set of queryPoints.
 *
 *  @param points
 *    Set of 3D points (x, y, z).
 *
 *  @param k
 *    K value.
 *
 *  @param queryPoints
 *    Set of 3D query points (x, y, z).
 *
 *  @retval
 *    The squared distance to the Kth nearest neighbor
 *    for each query point.
 * */
std::vector<float> compute(std::vector<Point>& points,
    const std::vector<Point>& queryPoints, const int& k);
}
```

here's an example of how to use the interface

```cpp
#include "knn.h" // <-- include library
#include "point.h"

int main()
{
    int k = 5;                                // set k value
    std::vector<Point> points = readPoints(); // get points

    /** query single point  */
    Point queryPoint = points[1000]; // arbitrary query point
    std::vector<int> heapNN = knn::compute(points, points[1], k);

    /** query multiple points */
    int start = (int)points.size() / 2; // arbitrary slice position
    int end = (int)points.size() / 4;   // arbitrary slice position
    std::vector<Point> queryPoints
        = std::vector<Point>(points.begin() + start, points.end() - end);
    std::vector<float> heapDists = knn::compute(points, queryPoints, k);

    /* sort the squared L2 distances and output them for plotting */
    std::sort(heapDists.begin(), heapDists.end(), std::greater<>());
    const std::string file = pwd() + "/knn.csv";
    std::cout << file << std::endl;
    writePoints(heapDists, file);

    return 0;
}

```

*   You can find an example Point Class [here](https://github.com/edisonslightbulbs/point).

*   The `readPoints` and `writePoints` functions are adaptable implementations and therefore left out.
