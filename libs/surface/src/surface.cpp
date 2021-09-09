#include "surface.h"
#include "file.h"
#include "i3d.h"

cv::Mat surface::getSurfaceCapture(std::shared_ptr<i3d>& sptr_i3d)
{
    int w = sptr_i3d->getDWidth();
    int h = sptr_i3d->getDHeight();
    uint8_t* c2dImage = *sptr_i3d->getC2DBGRAData();
    cv::Mat frame
        = cv::Mat(h, w, CV_8UC4, (void*)c2dImage, cv::Mat::AUTO_STEP).clone();
    return frame;
}

void surface::project(const std::string& window, const int& w, const int& h,
    cv::Mat& img, const cv::Mat& R, const cv::Mat& t)
{
    // rotate image
    cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);

    // scale
    cv::Size dSize = cv::Size(w, h);
    cv::resize(img, img, dSize, 0, 0, cv::INTER_AREA);

    // create named window
    cv::namedWindow(window, cv::WINDOW_NORMAL);

    // remove window's title bar
    cv::setWindowProperty(
        window, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    // show window @ position x=3000, y=0
    cv::imshow(window, img);
    cv::moveWindow(window, 3000, 0);
}

cv::Mat surface::color(const bool& contrast, const int& w, const int& h)
{
    cv::Mat black(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat white(h, w, CV_8UC3, cv::Scalar(255, 255, 255));

    if (contrast) {
        return white;
    } else {
        return black;
    }
}

void surface::contrast(std::shared_ptr<i3d>& sptr_i3d,
    const std::string& window, const int& w, const int& h,
    std::vector<cv::Mat>& scene)
{
    bool contrast = false;
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::setWindowProperty(
        window, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    while (true) {
        cv::Mat img = color(contrast, w, h);
        cv::imshow(window, img);
        cv::moveWindow(window, 3000, 0);
        if (cv::waitKey(2000) >= 0) {
            break;
        }

        // capture scene
        cv::Mat frame = getSurfaceCapture(sptr_i3d);
        scene.emplace_back(frame);
        contrast = !contrast;

        if (scene.size() == 2) {
            cv::waitKey(1000);
            cv::destroyWindow(window);
            break;
        }
    }
}

void surface::saturate(const cv::Mat& src, cv::Mat& dst)
{
    int beta = 100;     // brightness | range 1 - 100
    double alpha = 3.0; // contrast | range 1.0 - 3.0]

    dst = cv::Mat::zeros(src.size(), src.type());
    for (int y = 0; y < src.rows; y++) {
        for (int x = 0; x < src.cols; x++) {
            for (int c = 0; c < src.channels(); c++) {
                dst.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
                    alpha * src.at<cv::Vec3b>(y, x)[c] + beta);
            }
        }
    }
}

void surface::undistort(cv::Mat& frame)
{
    // initialize image and camera matrix
    cv::Mat undistortedFrame, K, refinedK, distortionCoefficients;
    K = cv::Mat::eye(3, 3, CV_64F);
    //usage::prompt(LOADING_CALIBRATION_PARAMETERS);

    std::string file = "./output/calibration/samples/camera.txt";
    parameters::read(file, K, distortionCoefficients);

    int alpha = 1;
    cv::Size dSize = cv::Size(frame.rows, frame.cols);
    refinedK = cv::getOptimalNewCameraMatrix(
        K, distortionCoefficients, dSize, alpha, dSize);
    cv::undistort(frame, undistortedFrame, K, distortionCoefficients, refinedK);
}

cv::Rect surface::reg(const cv::Mat& src1, const cv::Mat& src2)
{
    cv::Mat background, foreground;
    background = src1;
    foreground = src2;

    // subtract images and contrast resulting image
    cv::Mat diff, contrast;
    diff = background - foreground;
    saturate(diff, contrast);
    // todo: undistort

    // split high contrast image
    cv::Mat rgb[3];
    cv::Mat bgr;
    cv::cvtColor(contrast, bgr, cv::COLOR_BGR2RGB);

    cv::split(bgr, rgb);
    // cv::equalizeHist(src, dst); // one other good approach to contrasting

    // threshold blue channel
    cv::Mat thresh;
    cv::threshold(rgb[2], thresh, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // clean using morphological operations
    cv::Mat shape, proposal;
    shape = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(thresh, proposal, cv::MORPH_OPEN, shape);

    // de-noise
    cv::Mat blur, secThresh;
    cv::Size dBlur = cv::Size(75, 75);
    cv::GaussianBlur(proposal, blur, dBlur, 0);

    // threshold denoised frame
    cv::threshold(blur, secThresh, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

    // flood fill
    cv::Mat floodFill = secThresh.clone();
    cv::floodFill(floodFill, cv::Point(0, 0), cv::Scalar(255));

    // invert flood fill
    cv::Mat floodFillInv;
    cv::bitwise_not(floodFill, floodFillInv);

    // combine threshold and flood fill inverse
    cv::Mat roi = (secThresh | floodFillInv);

#define show 0
#if show == 1
    cv::imshow("1: Background subtraction", diff);
    cv::imshow("2: Contrast", contrast);
    cv::imshow("3.1: Red channel", rgb[0]);
    cv::imshow("3.2: Green channel", rgb[1]);
    cv::imshow("3.3: Blue channel", rgb[2]);
    cv::imshow("4: Binary inverted threshold", thresh);
    cv::imshow("5.1: Morphological cleaning", proposal);
    cv::imshow("5.2: Morphological cleaning", blur);
    cv::imshow("5.3: Morphological cleaning", secThresh);
    cv::imshow("6.1: Flood-fill", floodFill);
    cv::imshow("6.2: Flood-fill inverse", floodFillInv);
    cv::imshow("7: ROI", roi);
    cv::waitKey();
#endif

#define write 0
#if write == 1
    cv::imwrite("./output/background.png", src1);
    cv::imwrite("./output/foreground.png", src2);
    cv::imwrite("./output/01___diff.png", diff);
    cv::imwrite("./output/02___contrast.png", contrast);
    cv::imwrite("./output/03___redchannel.png", rgb[0]);
    cv::imwrite("./output/04___greenchannel.png", rgb[1]);
    cv::imwrite("./output/05___bluechannel.png", rgb[2]);
    cv::imwrite("./output/06___thresh.png", thresh);
    cv::imwrite("./output/07___proposal.png", proposal);
    cv::imwrite("./output/08___blur.png", blur);
    cv::imwrite("./output/09___secthresh.png", secThresh);
    cv::imwrite("./output/10___floodfill.png", floodFill);
    cv::imwrite("./output/11___floodfillinverse.png", floodFillInv);
    cv::imwrite("./output/12___segment.png", roi);
#endif
    return cv::boundingRect(roi);
}
