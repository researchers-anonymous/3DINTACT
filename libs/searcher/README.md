#### Usage of this tiny searcher lib based on [nanoflann](https://github.com/jlblancoc/nanoflann)

Massive thanks to [Jose](https://github.com/jlblancoc) for sharing this adaptation of [flann](https://github.com/mariusmuja/flann) 👏🍻🍻

*   the interface

```cpp
#include <vector>
#include "point.h"

namespace searcher {

/** pointFound
 *    Computes the K nearest neighbours of a queryPoint.
 *
 *  @param points
 *    Set of 3D points (x, y, z).
 *
 *  @param queryPoint
 *    query point
 *
 *  @retval
 *    true: if point found
 *    false: if point not found
 * */
bool pointFound(std::vector<Point>& points, const Point& queryPoint);
}
```

here's an example of how to use the interface

```cpp
#include "point.h"
#include "searcher.h" // <-- include library

int main()
{
    std::vector<Point> points = readPoints(); // <-- our dense set of points

    std::vector<Point> queryPoints(2);
    queryPoints[0] = Point(4.0, 5.0, 6.0);    // point does not exist in our set
    queryPoints[1] = points[1000];            // point does not exist in our set

    for (auto& queryPoint : queryPoints) {
        if (searcher::pointFound(points, queryPoint)) {
            std::cout << "-- point: (" << queryPoint <<  ") found" << std::endl;
        } else {
            std::cout << "-- point: (" << queryPoint <<  ") not found" << std::endl;
        }
    }
    return 0;
}

```

*   You can find an example Point Class [here](https://github.com/edisonslightbulbs/point).

*   The `readPoints` function is an adaptable implementation and therefore left out.
