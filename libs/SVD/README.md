###### Helper for class singular value decomposition (SVD)

*   interface:

```cpp

#include <Eigen/Dense>
#include <vector>
#include "point.h"

class svd {

private:
    const int C0 = 0; // column 0
    const int C1 = 1; // column 1
    const int C2 = 2; // column 2
    std::vector<Point> m_points;

public:
    Point m_centroid;                        // centroid of given set of points
    Eigen::MatrixXf m_vectors;               // points translated from centroid
    Eigen::JacobiSVD<Eigen::MatrixXf> m_usv; // computed USV solution

    /** svd
     *    Computes USV on construction
     *     U - the unitary matrix,
     *     S - rectangular diagonal matrix, and
     *     V - the complex unitary matrix.
     *
     * @param points
     *   The given set of points.
     */
    explicit svd(const std::vector<Point>& points, const int& flag);

    /** getV3Normal
     *
     * @retval
     *   Normal vector (v3) from the complex unitary matrix.
     */
    Eigen::Vector3d getV3Normal();

    /** getUNormal
     *
     * @retval
     *   Normal vectors (U) from the unitary matrix.
     */
    std::vector<Eigen::Vector3d> getUNormals();
};

```

*   usage:

```cpp
#include <Eigen/Dense>
#include <vector>
#include "point.h"
#include "svd.h" //<-- include svd library

int main(int argc, char* argv[]){
    std::vector<Point> points = readPoints();

    // set svd computation flag
    int flag = Eigen::ComputeThinU | Eigen::ComputeThinV;

    // compute SVD
    svd usv(cluster, flag);

return 0;
}
```

*   Checkout a flexible `Point` struct [here](https://github.com/edisonslightbulbs/point).
