// #include <opencv2/aruco.hpp>
// #include <opencv2/calib3d.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/opencv.hpp>
//
// #include <chrono>
// #include <fstream>
// #include <iostream>
// #include <thread>
//
// #include "calibration.h"
//
// void calibration::createArucoMarkers()
// {
//     cv::Mat outputMarker;
//     cv::Ptr<cv::aruco::Dictionary> markerDictionary
//         = cv::aruco::getPredefinedDictionary(
//             cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
//
//     for (int i = 0; i < 50; i++) {
//         cv::aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
//         std::ostringstream convert;
//         std::string imageName = "4x4Marker_";
//         convert << imageName << i << ".jpg";
//         cv::imwrite(convert.str(), outputMarker);
//     }
// }
//
// void calibration::createChessboardBoarder(const cv::Size& boardSize,
//     float squareEdgeLength, std::vector<cv::Point3f>& corners)
// {
//     for (int i = 0; i < boardSize.height; i++) {
//         for (int j = 0; j < boardSize.width; j++) {
//             corners.emplace_back(cv::Point3f((float)j * squareEdgeLength,
//                 (float)i * squareEdgeLength, 0.0f));
//         }
//     }
// }
//
// void calibration::findChessboardCorners(std::vector<cv::Mat>& images,
//     std::vector<std::vector<cv::Point2f>>& allFoundCorners, bool showResults)
// {
//     for (auto& image : images) {
//         std::vector<cv::Point2f> pointBuf;
//         bool found = cv::findChessboardCorners(image, cv::Size(9, 6),
//         pointBuf,
//             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
//
//         if (found) {
//             allFoundCorners.emplace_back(pointBuf);
//         }
//
//         if (showResults) {
//             cv::drawChessboardCorners(image, cv::Size(9, 6), pointBuf,
//             found); cv::imshow("Looking for Corners", image); cv::waitKey(0);
//         }
//     }
// }
//
// void calibration::calibrate(std::vector<cv::Mat> calibrationImages,
//     const cv::Size& boardSize, float squareEdgeLength, cv::Mat& cameraMatrix,
//     cv::Mat& distanceCoefficients)
// {
//     std::vector<std::vector<cv::Point2f>> checkerboardImageSpacePoints;
//     findChessboardCorners(
//         calibrationImages, checkerboardImageSpacePoints, false);
//
//     std::vector<std::vector<cv::Point3f>> worldSpaceCornersPoints(1);
//
//     createChessboardBoarder(
//         boardSize, squareEdgeLength, worldSpaceCornersPoints[0]);
//
//     worldSpaceCornersPoints.resize(
//         checkerboardImageSpacePoints.size(), worldSpaceCornersPoints[0]);
//
//     std::vector<cv::Mat> rVectors, tVectors;
//     distanceCoefficients = cv::Mat::zeros(8, 1, CV_64F);
//
//     cv::calibrateCamera(worldSpaceCornersPoints,
//     checkerboardImageSpacePoints,
//         boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
// }
//
// bool calibration::exportCalibration(
//     const std::string& name, cv::Mat cameraMatrix, cv::Mat
//     distanceCoefficients)
// {
//     std::ofstream outStream(name);
//     if (outStream) {
//
//         uint16_t rows = cameraMatrix.rows;
//         uint16_t columns = cameraMatrix.cols;
//
//         outStream << rows << std::endl;
//         outStream << columns << std::endl;
//
//         for (int r = 0; r < rows; r++) {
//             for (int c = 0; c < columns; c++) {
//                 double value = cameraMatrix.at<double>(r, c);
//                 outStream << value << std::endl;
//             }
//         }
//
//         rows = distanceCoefficients.rows;
//         columns = distanceCoefficients.cols;
//
//         outStream << rows << std::endl;
//         outStream << columns << std::endl;
//
//         for (int r = 0; r < rows; r++) {
//             for (int c = 0; c < columns; c++) {
//                 double value = distanceCoefficients.at<double>(r, c);
//                 outStream << value << std::endl;
//             }
//         }
//         outStream.close();
//         return true;
//     }
//     return false;
// }
//
// bool calibration::importCalibration(const std::string& name,
//     cv::Mat& cameraMatrix, cv::Mat& distanceCoefficients)
// {
//     std::cout << "-- loading calibration parameters" << std::endl;
//     std::ifstream inStream(name);
//     if (inStream) {
//
//         uint16_t rows;
//         uint16_t columns;
//
//         inStream >> rows;
//         inStream >> columns;
//         cameraMatrix = cv::Mat(cv::Size(columns, rows), CV_64F);
//
//         for (int r = 0; r < rows; r++) {
//             for (int c = 0; c < columns; c++) {
//                 double read = 0.0f;
//                 inStream >> read;
//                 cameraMatrix.at<double>(r, c) = read;
//                 std::cout << cameraMatrix.at<double>(r, c) << "\n";
//             }
//         }
//         inStream >> rows;
//         inStream >> columns;
//         distanceCoefficients = cv::Mat(cv::Size(columns, rows), CV_64F);
//
//         for (int r = 0; r < rows; r++) {
//             for (int c = 0; c < columns; c++) {
//                 double read = 0.0f;
//                 inStream >> read;
//                 distanceCoefficients.at<double>(r, c) = read;
//                 std::cout << cameraMatrix.at<double>(r, c) << "\n";
//             }
//         }
//         inStream.close();
//         return true;
//     }
//     return false;
// }
//
// int calibration::findArucoMarkers(
//     const cv::Mat& cameraMatrix, const cv::Mat& distanceCoefficients)
// {
//     cv::Mat frame;
//     std::vector<int> markerIds;
//     std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCorners;
//
//     cv::Ptr<cv::aruco::Dictionary> markerDictionary
//         = cv::aruco::getPredefinedDictionary(
//             cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
//
//     /** create named window */
//     cv::namedWindow("kinect", cv::WINDOW_AUTOSIZE);
//
//     /** calibration R and t */
//     std::vector<cv::Vec3d> rotationVectors, translationVectors;
//
//     /** initialize kinect */
//     std::shared_ptr<Kinect> sptr_kinect(new Kinect);
//     int rgbWidth = k4a_image_get_width_pixels(sptr_kinect->m_img);
//     int rgbHeight = k4a_image_get_height_pixels(sptr_kinect->m_img);
//
//     bool init = true;
//     while (true) {
//         if (init) {
//             std::cout
//                 << "-- place aruco marker/s in field of view of the camera"
//                 << std::endl;
//             init = false;
//         }
//         /** get next frame from kinect */
//         sptr_kinect->capture();
//         sptr_kinect->imgCapture();
//
//         /** get image from kinect */
//         uint8_t* ptr_img = k4a_image_get_buffer(sptr_kinect->m_img);
//
//         /** release resources */
//         sptr_kinect->releaseK4aImages();
//         sptr_kinect->releaseK4aCapture();
//
//         /** cast to cv::Mat */
//         frame = cv::Mat(
//             rgbHeight, rgbWidth, CV_8UC4, (void*)ptr_img,
//             cv::Mat::AUTO_STEP);
//
//         cv::cvtColor(frame, frame, cv::COLOR_BGRA2RGB);
//
//         cv::aruco::detectMarkers(
//             frame, markerDictionary, markerCorners, markerIds);
//         cv::aruco::estimatePoseSingleMarkers(markerCorners,
//             arucoSquareDimension, cameraMatrix, distanceCoefficients,
//             rotationVectors, translationVectors);
//
//         /** draw axis on the detected aruco markers */
//         for (int i = 0; i < markerIds.size(); i++) {
//             cv::aruco::drawAxis(frame, cameraMatrix, distanceCoefficients,
//                 rotationVectors[i], translationVectors[i], 0.1f);
//         }
//         cv::imshow("kinect", frame);
//         if (cv::waitKey(30) >= 0)
//             break;
//     }
//     return 1;
// }
//
// void calibration::startChessBoardCalibration(
//     cv::Mat& cameraMatrix, cv::Mat distanceCoefficients)
// {
//     const cv::Size chessboardDimensions = cv::Size(9, 6);
//
//     cv::Mat frame;
//     cv::Mat drawToFrame;
//
//     std::vector<cv::Mat> savedImages;
//     std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
//
//     /** initialize kinect */
//     std::shared_ptr<Kinect> sptr_kinect(new Kinect);
//     int rgbWidth = k4a_image_get_width_pixels(sptr_kinect->m_img);
//     int rgbHeight = k4a_image_get_height_pixels(sptr_kinect->m_img);
//
//     /** defined frames per second */
//     const int fps = 20;
//
//     /** create named window */
//     cv::namedWindow("kinect", cv::WINDOW_AUTOSIZE);
//
//     bool done = false;
//     bool init = true;
//     while (!done) {
//         /** helper prompt */
//         if (init) {
//             init = false;
//             std::cout << "-- press ENTER key to take image of chessboard"
//                       << std::endl;
//             std::cout << "-- press ESC key to exit calibration" << std::endl;
//         }
//
//         /** get next frame from kinect */
//         sptr_kinect->capture();
//         sptr_kinect->imgCapture();
//
//         /** get image from kinect */
//         uint8_t* ptr_img = k4a_image_get_buffer(sptr_kinect->m_img);
//
//         /** release resources */
//         sptr_kinect->releaseK4aImages();
//         sptr_kinect->releaseK4aCapture();
//
//         /** cast to cv::Mat */
//         frame = cv::Mat(
//             rgbHeight, rgbWidth, CV_8UC4, (void*)ptr_img,
//             cv::Mat::AUTO_STEP);
//
//         /** find corners */
//         std::vector<cv::Point2f> foundPoints;
//         bool found;
//         found = cv::findChessboardCorners(frame, chessboardDimensions,
//             foundPoints,
//             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
//         frame.copyTo(drawToFrame);
//
//         /** if found,  draw them */
//         cv::drawChessboardCorners(
//             drawToFrame, chessboardDimensions, foundPoints, found);
//
//         if (found) {
//             cv::imshow("kinect", drawToFrame);
//         } else {
//             cv::imshow("kinect", frame);
//         }
//         int key = cv::waitKey(1000 / fps);
//
//         switch (key) {
//         case 13:
//             /** save image */
//             if (found) {
//                 cv::Mat temp;
//                 frame.copyTo(temp);
//                 savedImages.emplace_back(temp);
//                 std::cout << "current number of images: " <<
//                 savedImages.size()
//                           << std::endl;
//             }
//             break;
//         case 27:
//             /** calibrate */
//             if (savedImages.size() > 15) {
//                 calibrate(savedImages, chessboardDimensions,
//                     calibrationSquareDimension, cameraMatrix,
//                     distanceCoefficients);
//                 exportCalibration(
//                     "calibration.txt", cameraMatrix, distanceCoefficients);
//                 done = true;
//             }
//         default:
//             break;
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(5));
//     }
// }
//
// void calibration::kinect2CV(std::shared_ptr<Kinect>& sptr_kinect)
// {
//     std::vector<k4a_float3_t> points_3d
//         = { { { 0.f, 0.f, 1000.f } },            // color camera center
//               { { -1000.f, -1000.f, 1000.f } },  // color camera top left
//               { { 1000.f, -1000.f, 1000.f } },   // color camera top right
//               { { 1000.f, 1000.f, 1000.f } },    // color camera bottom right
//               { { -1000.f, 1000.f, 1000.f } } }; // color camera bottom left
//
//     // k4a project function
//     std::vector<k4a_float2_t> k4a_points_2d(points_3d.size());
//     for (size_t i = 0; i < points_3d.size(); i++) {
//         int valid = 0;
//         k4a_calibration_3d_to_2d(&sptr_kinect->m_calibration, &points_3d[i],
//             K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_DEPTH,
//             &k4a_points_2d[i], &valid);
//     }
//
//     // converting the calibration data to OpenCV format
//     // extrinsic transformation from color to depth camera
//     cv::Mat se3 = cv::Mat(3, 3, CV_32FC1,
//         sptr_kinect->m_calibration
//             .extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH]
//             .rotation);
//     cv::Mat r_vec = cv::Mat(3, 1, CV_32FC1);
//     Rodrigues(se3, r_vec);
//     cv::Mat t_vec = cv::Mat(3, 1, CV_32F,
//         sptr_kinect->m_calibration
//             .extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_DEPTH]
//             .translation);
//
//     // intrinsic parameters of the depth camera
//     k4a_calibration_intrinsic_parameters_t* intrinsics
//         = &sptr_kinect->m_calibration.depth_camera_calibration.intrinsics
//                .parameters;
//     std::vector<float> _camera_matrix
//         = { intrinsics->param.fx, 0.f, intrinsics->param.cx, 0.f,
//               intrinsics->param.fy, intrinsics->param.cy, 0.f, 0.f, 1.f };
//     cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, &_camera_matrix[0]);
//     std::vector<float> _dist_coeffs
//         = { intrinsics->param.k1, intrinsics->param.k2, intrinsics->param.p1,
//               intrinsics->param.p2, intrinsics->param.k3,
//               intrinsics->param.k4, intrinsics->param.k5,
//               intrinsics->param.k6 };
//     cv::Mat dist_coeffs = cv::Mat(8, 1, CV_32F, &_dist_coeffs[0]);
//
//     // OpenCV project function
//     std::vector<cv::Point2f> cv_points_2d(points_3d.size());
//     projectPoints(*reinterpret_cast<std::vector<cv::Point3f>*>(&points_3d),
//         r_vec, t_vec, camera_matrix, dist_coeffs, cv_points_2d);
//
//     for (size_t i = 0; i < points_3d.size(); i++) {
//         printf("3d point:\t\t\t(%.5f, %.5f, %.5f)\n", points_3d[i].v[0],
//             points_3d[i].v[1], points_3d[i].v[2]);
//         printf("OpenCV projectPoints:\t\t(%.5f, %.5f)\n", cv_points_2d[i].x,
//             cv_points_2d[i].y);
//         printf("k4a_calibration_3d_to_2d:\t(%.5f, %.5f)\n\n",
//             k4a_points_2d[i].v[0], k4a_points_2d[i].v[1]);
//     }
// }
//
