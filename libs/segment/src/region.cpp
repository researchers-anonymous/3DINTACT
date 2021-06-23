#include <ply.h>
#include <vector>

#include "outliers.h"
#include "proposal.h"
#include "region.h"
#include "svd.h"

std::vector<Point> region::segment(std::vector<Point>& points)
{
    //  arguably unnecessary, still done
    //  to keep a 'non mutated' point cloud.
    std::vector<Point> raw = points;

    // remove outliers
    std::vector<Point> denoised = outliers::filter(raw);

    // compute svd
    int flag = Eigen::ComputeThinU | Eigen::ComputeThinV;
    SVD usv(denoised, flag);

    // course segment
    std::vector<Point> coarse = proposal::grow(usv, denoised);

    // final segment
    // std::vector<Point> final = outliers::filter(coarse);

    return coarse;
}
