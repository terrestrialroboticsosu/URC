#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

// Helper function to get Euler angles
// Helper function to get Euler angles
cv::Vec3d getEulerAngles(const cv::Mat &rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R); // Convert Rodrigues vector to rotation matrix
    
    cv::Mat mtxR, mtxQ;
    cv::Mat eulerAnglesMat; 
    
    // Pass the real Mat objects to the function
    cv::RQDecomp3x3(R, mtxR, mtxQ, eulerAnglesMat);
    
    // --- THIS IS THE FIX ---
    // Don't use the implicit converter. Extract the values manually.
    // eulerAnglesMat is a 3x1 matrix of type CV_64F (double)
    cv::Vec3d eulerAngles;
    eulerAngles[0] = eulerAnglesMat.at<double>(0, 0); // roll
    eulerAngles[1] = eulerAnglesMat.at<double>(1, 0); // pitch
    eulerAngles[2] = eulerAnglesMat.at<double>(2, 0); // yaw
    // --- END OF FIX ---

    return eulerAngles;
}

int main() {
    // --- 1. Configuration ---
    std::string CALIBRATION_FILE = "webcam_calibration.yml";
    double current_marker_size = 10.0; // Default size in mm

    cv::Mat camera_matrix, dist_coeffs;

    cv::aruco::Dictionary aruco_dict = 
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    cv::aruco::DetectorParameters parameters;

    // --- 2. Load Calibration Data ---
    cv::FileStorage fs(CALIBRATION_FILE, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open calibration file: " << CALIBRATION_FILE << std::endl;
        return -1;
    }
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs.release();

    if (camera_matrix.empty() || dist_coeffs.empty()) {
        std::cerr << "Error: Calibration data is invalid." << std::endl;
        return -1;
    }
    std::cout << "Calibration data loaded successfully." << std::endl;
    std::cout << "Camera Matrix:\n" << camera_matrix << std::endl;
    std::cout << "Distortion Coeffs:\n" << dist_coeffs << std::endl;

    // --- 3. Start Webcam ---
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_AUTOFOCUS, 0); // Disable autofocus
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open webcam." << std::endl;
        return -1;
    }

    std::cout << "Webcam opened. Press 'q' to quit." << std::endl;
    std::cout << "Press '1' for 10mm markers" << std::endl;
    std::cout << "Press '2' for 20mm markers" << std::endl;
    std::cout << "Press '3' for 40mm markers" << std::endl;

    cv::Mat frame, gray;
    while (true) {
        cap >> frame; 
        if (frame.empty()) {
            break; 
        }

        // --- 4. Key Press Logic ---
        char key = (char)cv::waitKey(1); 
        if (key == 'q') {
            break;
        } else if (key == '1') {
            current_marker_size = 10.0;
            std::cout << "Switched to " << current_marker_size << "mm" << std::endl;
        } else if (key == '2') {
            current_marker_size = 20.0;
            std::cout << "Switched to " << current_marker_size << "mm" << std::endl;
        } else if (key == '3') {
            current_marker_size = 40.0;
            std::cout << "Switched to " << current_marker_size << "mm" << std::endl;
        }

        // --- 5. Detect Markers ---
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        // --- FIX 1: Create Ptrs for the old API header ---
        cv::Ptr<cv::aruco::Dictionary> dict_ptr = cv::makePtr<cv::aruco::Dictionary>(aruco_dict);
        cv::Ptr<cv::aruco::DetectorParameters> params_ptr = cv::makePtr<cv::aruco::DetectorParameters>(parameters);

        cv::aruco::detectMarkers(gray, dict_ptr, corners, ids, params_ptr); // Pass Ptrs

        // --- 6. Estimate Pose ---
        if (ids.size() > 0) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                corners, 
                current_marker_size, 
                camera_matrix, 
                dist_coeffs, 
                rvecs, 
                tvecs
            );

            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            for (size_t i = 0; i < ids.size(); ++i) {
                // Draw 3D axes
                cv::drawFrameAxes(
                    frame, 
                    camera_matrix, 
                    dist_coeffs, 
                    rvecs[i], 
                    tvecs[i], 
                    current_marker_size / 2
                );

                // --- 7. Get and Display Pose Info ---
                cv::Vec3d tvec = tvecs[i];
                
                // --- FIX 2: Use explicit Mat constructor ---
                cv::Mat rvec_mat(rvecs[i]); 
                cv::Vec3d eulerAngles = getEulerAngles(rvec_mat);
                
                // Format text strings
                char text_buf[200];
                snprintf(text_buf, sizeof(text_buf), "ID: %d", ids[i]);
                std::string id_str(text_buf);
                snprintf(text_buf, sizeof(text_buf), "x: %.0f y: %.0f z: %.0f mm", tvec[0], tvec[1], tvec[2]);
                std::string pos_str(text_buf);
                snprintf(text_buf, sizeof(text_buf), "R: %.0f P: %.0f Y: %.0f deg", eulerAngles[0], eulerAngles[1], eulerAngles[2]);
                std::string rot_str(text_buf);

                cv::Point text_corner = corners[i][0]; 

                // Draw the text
                cv::putText(frame, rot_str, cv::Point(text_corner.x, text_corner.y - 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, pos_str, cv::Point(text_corner.x, text_corner.y - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, id_str, text_corner, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            }
        }

        // Display current marker size
        cv::putText(frame, "Active Size: " + std::to_string((int)current_marker_size) + "mm (Keys: 1, 2, 3)", 
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 100), 2);

        // --- 8. Show Frame ---
        cv::imshow("ArUco 6D Pose Detection (C++)", frame);
    }

    // --- 9. Cleanup ---
    cap.release();
    cv::destroyAllWindows();
    return 0;
}