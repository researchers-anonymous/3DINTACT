#define DEPTH_NORMAL_THRESHOLD 0.005 // <- depth normal threshold

/** Silhouette edge adapted */
namespace edge {

bool detect(const float& depthNormalComponent)
{
    if (depthNormalComponent < DEPTH_NORMAL_THRESHOLD
        && depthNormalComponent > -DEPTH_NORMAL_THRESHOLD) {
        return true;
    } else
        return false;
}
}
