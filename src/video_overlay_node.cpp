#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <limits>
#include <numeric>
using namespace cv;
using namespace std;

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

vector<Point2f> camera_starting_pts;
vector<Point2f> vicon_starting_pts;
vector<Point2f> vicon_starting_img_pts;
Mat imageCamera;

double x_min = -8; double x_max = 2; double y_min = -4; double y_max = 4; double resolution = 0.01;
Mat traj_img(int((y_max - y_min)/resolution), int((x_max - x_min)/resolution), CV_8UC3, Scalar(0,0,0));

// // Function to add main image and transformed logo image and show final output.
// // Icon image replaces the pixels of main image in this implementation.
// void showFinal(Mat src1,Mat src2)
// {
//     Mat gray,gray_inv,src1final,src2final;
//     cvtColor(src2,gray,CV_BGR2GRAY);
//     threshold(gray,gray,0,255,CV_THRESH_BINARY);
//     //adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,5,4);
//     bitwise_not ( gray, gray_inv );
//     src1.copyTo(src1final,gray_inv);
//     src2.copyTo(src2final,gray);
//     Mat finalImage = src1final+src2final;
//     namedWindow( "output", WINDOW_AUTOSIZE );
//     imshow("output",finalImage);
//     cvWaitKey(0);

// }

// void overlayFinal(Mat src1,Mat src2){
//     cv::Mat src1_bgra;
//     cv::Mat src2_bgra;
//     cv::cvtColor(src1, src1_bgra, CV_BGR2BGRA);
//     cv::cvtColor(src2, src2_bgra, CV_BGR2BGRA);

//     // find all white pixel and set alpha value to zero:
//     for (int y = 0; y < src2_bgra.rows; ++y)
//     for (int x = 0; x < src2_bgra.cols; ++x)
//     {
//         cv::Vec4b & pixel = src2_bgra.at<cv::Vec4b>(y, x);
//         // if pixel is black
//         if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0)
//         {
//             // set alpha to zero:
//             pixel[3] = 0;
//         }
//     }

//     Mat finalImage = src1_bgra+src2_bgra;
//     namedWindow( "output", WINDOW_AUTOSIZE );
//     imshow("output",finalImage);
//     cvWaitKey(0);
// }

// void overlayImage(Mat* src, Mat* overlay, const Point& location)
// {
//     for (int y = max(location.y, 0); y < src->rows; ++y)
//     {
//         int fY = y - location.y;

//         if (fY >= overlay->rows)
//             break;

//         for (int x = max(location.x, 0); x < src->cols; ++x)
//         {
//             int fX = x - location.x;

//             if (fX >= overlay->cols)
//                 break;

//             double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3]) / 255;

//             for (int c = 0; opacity > 0 && c < src->channels(); ++c)
//             {
//                 unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
//                 unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
//                 src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
//             }
//         }
//     }
    
// }

// // Here we get four points from the user with left mouse clicks.
// // On 5th click we output the overlayed image.
// void on_mouse( int e, int x, int y, int d, void *ptr )
// {
//     if (e == EVENT_LBUTTONDOWN )
//     {
//         if(camera_starting_pts.size() < 4 )
//         {
//             camera_starting_pts.push_back(Point2f(float(x),float(y)));
//             cout << x << " "<< y <<endl;
//         }
//         else
//         {
//             cout << " Calculating Homography " <<endl;
//             // Deactivate callback
//             cv::setMouseCallback("Display window", NULL, NULL);
//             // once we get 4 corresponding points in both images calculate homography matrix

//             for (int i=0; i<vicon_starting_img_pts.size(); i++){
//                 std::cout << vicon_starting_img_pts[i] << std::endl;
//             }
//             for (int i=0; i<camera_starting_pts.size(); i++){
//                 std::cout << camera_starting_pts[i] << std::endl;
//             }

            

//             Mat H = findHomography(  vicon_starting_img_pts, camera_starting_pts, 0 );
//             // cout << "vicon_starting_img_pts = "<< endl << " "  << vicon_starting_img_pts << endl << endl;
//             // cout << "camera_starting_pts = "<< endl << " "  << camera_starting_pts << endl << endl;
//             // cout << "H = "<< endl << " "  << H << endl << endl;

//             // Warp the logo image to change its perspective
//             Mat hx04_traj_warped;
//             warpPerspective(hx04_traj_img, hx04_traj_warped, H, imageCamera.size() );

//             // overlayImage( &imageCamera, &hx04_traj_warped, Point() );
//             // namedWindow( "output", WINDOW_AUTOSIZE );
//             // imshow("output",imageCamera);
//             // cvWaitKey(0);
//             // overlayFinal(imageCamera, hx04_traj_warped);
//             showFinal(imageCamera, hx04_traj_warped);

//         }

//     }
// }

void unlimited_mouse_click( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN )
    {
        cout << x << " "<< y <<endl;
    }
}

Point2f vicon_coord_to_top_down_img_plane_coord(Point2f vicon_pt){
    return Point((vicon_pt.x - x_min)/resolution, (y_max - vicon_pt.y)/resolution);
}

int closest_ind(vector<double> v, double refElem){
    vector<double>::iterator lower = std::lower_bound(v.begin(), v.end(), refElem);
    return lower - v.begin();
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "video_overlay_node");
    ros::NodeHandle nh("~");

    bool write_video;
    bool have_homography_matrix;
    vector<string> veh_names;
    string output_video_filename;
    string bag_filename;
    string input_video_filename;
    double video_start_time;
    double video_end_time;
    double traj_time_length;
    double video_ahead_of_rosbag;
    double time_btwn_veh_and_latest_traj;
    double time_btwn_traj_pts;
    double max_time_of_traj;
    bool click_pts_for_homography;
    double circle_radius, circle_thickness;

    nh.getParam("veh_names", veh_names);
    nh.getParam("output_video_filename", output_video_filename);
    nh.getParam("bag_filename", bag_filename);
    nh.getParam("input_video_filename", input_video_filename);
    nh.getParam("video_start_time", video_start_time);
    nh.getParam("video_end_time", video_end_time);
    nh.getParam("traj_time_length", traj_time_length);
    nh.getParam("video_ahead_of_rosbag", video_ahead_of_rosbag);
    nh.getParam("time_btwn_veh_and_latest_traj", time_btwn_veh_and_latest_traj);
    nh.getParam("time_btwn_traj_pts", time_btwn_traj_pts);
    nh.getParam("max_time_of_traj", max_time_of_traj);
    nh.getParam("write_video", write_video);
    nh.getParam("have_homography_matrix", have_homography_matrix);
    nh.getParam("click_pts_for_homography", click_pts_for_homography);
    nh.getParam("circle_radius", circle_radius);
    nh.getParam("circle_thickness", circle_thickness);

    if (click_pts_for_homography){
        if( argc != 2)
        {
            cout << "Usage Error: Need to supply static image for clicking pts." << endl;
            return -1;
        }
        imageCamera = imread(argv[1], CV_LOAD_IMAGE_COLOR);
        namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
        imshow( "Display window", imageCamera );
        setMouseCallback("Display window", unlimited_mouse_click, NULL );
        //  Press "Escape button" to exit
        while(1)
        {
            int key=cvWaitKey(10);
            if(key==27) break;
        }
        return 0;
    }

    Mat H;
    if (not have_homography_matrix){
        // Get homography matrix from yaml file

        XmlRpc::XmlRpcValue vicon_camera_pairs;
        XmlRpc::XmlRpcValue vicon_camera_pair;
        nh.getParam("vicon_camera_pairs", vicon_camera_pairs);

        for (int32_t i = 0; i < vicon_camera_pairs.size(); ++i) 
        {
          double vicon_x = vicon_camera_pairs[i][0][0];
          double vicon_y = vicon_camera_pairs[i][0][1];
          int camera_x = vicon_camera_pairs[i][1][0];
          int camera_y = vicon_camera_pairs[i][1][1];
          vicon_starting_pts.push_back(Point2f(vicon_x, vicon_y));
          camera_starting_pts.push_back(Point2f(camera_x, camera_y));
        }

        for (int i=0; i<vicon_starting_pts.size(); i++){
            vicon_starting_img_pts.push_back(vicon_coord_to_top_down_img_plane_coord(vicon_starting_pts[i]));
        }
        H = findHomography(  vicon_starting_img_pts, camera_starting_pts, 0 );

    }
    
    vector<Scalar> colors;
    colors.push_back(Scalar(255*0.0980, 255*0.3250, 255*0.8500));
    colors.push_back(Scalar(255*0.7410, 255*0.4470, 255*0.0));
    colors.push_back(Scalar(255*0.1880, 255*0.6740, 255*0.4660));
    colors.push_back(Scalar(255*0.5560, 255*0.1840, 255*0.4940));
    colors.push_back(Scalar(255*0.1250, 255*0.6940, 255*0.9290));
    colors.push_back(Scalar(255*0.9330, 255*0.7450, 255*0.3010));
    colors.push_back(Scalar(255*0.1840, 255*0.0780, 255*0.6350));
    
    map<string, vector<double>> timestamps;
    map<string, vector<Point2f>> trajs;
    for (int i=0; i<veh_names.size(); i++){
        string veh_name = veh_names[i];
        
        // load one of the veh's trajs
        rosbag::Bag bag;
        bag.open(bag_filename, rosbag::bagmode::Read);
        
        std::vector<std::string> topics;
        topics.push_back(std::string("/"+veh_name+"/pose"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));
        bool first = true;
        double first_timestamp, timestamp, last_used_timestep;
        foreach(rosbag::MessageInstance const m, view)
        {
            geometry_msgs::PoseStamped::ConstPtr s = m.instantiate<geometry_msgs::PoseStamped>();
            if (s != NULL){
                timestamp = s->header.stamp.toSec();
                if (first){
                    first_timestamp = timestamp;
                    last_used_timestep = timestamp;
                    first = false;
                }
                if ((timestamp - last_used_timestep < time_btwn_traj_pts) && (first_timestamp != timestamp)){
                    continue;
                }
                if (timestamp - first_timestamp - video_ahead_of_rosbag < video_start_time){
                    continue;
                }
                if (timestamp - first_timestamp - video_ahead_of_rosbag > video_end_time){
                    break;
                }

                last_used_timestep = timestamp;
                timestamps[veh_name].push_back(timestamp-first_timestamp);
                trajs[veh_name].push_back(Point2f(s->pose.position.x, s->pose.position.y));
                if (timestamp - first_timestamp > max_time_of_traj)
                    break;
            }
        }

        bag.close();


    }

    // // Coordinate axes (vicon origin)
    // arrowedLine(hx04_traj_img, vicon_coord_to_top_down_img_plane_coord(Point(0,0)), vicon_coord_to_top_down_img_plane_coord(Point(1,0)), Scalar(0,0,255), 0.05/resolution);
    // arrowedLine(hx04_traj_img, vicon_coord_to_top_down_img_plane_coord(Point(0,0)), vicon_coord_to_top_down_img_plane_coord(Point(0,1)), Scalar(0,255,0), 0.05/resolution);
    // // Add discs along veh trajectory
    // for (int i=0; i<hx04_traj.size(); i++){
    //     circle(hx04_traj_img, vicon_coord_to_top_down_img_plane_coord(hx04_traj[i]), 0.03/resolution, Scalar(255,0,0), 0.05/resolution);
    // }
    // // Add disc around 4 vehicle starting pts
    // for (int i=0; i<vicon_starting_img_pts.size(); i++){
    //     circle(hx04_traj_img, vicon_starting_img_pts[i], 0.1/resolution, Scalar(255,255,0), 0.05/resolution);
    // }


    VideoCapture cap(input_video_filename);

    // Check if camera opened successfully
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }


    VideoWriter video;
    if (write_video){
        // Default resolution of the frame is obtained.The default resolution is system dependent. 
        int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
        int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
        double input_fps = cap.get(CV_CAP_PROP_FPS);
        video = VideoWriter(output_video_filename, CV_FOURCC('M','J','P','G'), input_fps, Size(frame_width,frame_height)); 
    }
    
    while(1){
        Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
          break;

        if (cap.get(CAP_PROP_POS_MSEC)/1000. < video_start_time){
            continue;
        }
        if (cap.get(CAP_PROP_POS_MSEC)/1000. > video_end_time){
            break;
        }

        Mat traj_img(int((y_max - y_min)/resolution), int((x_max - x_min)/resolution), CV_8UC3, Scalar(0,0,0));
        
        double end_timestamp = cap.get(CAP_PROP_POS_MSEC)/1000. + video_ahead_of_rosbag - time_btwn_veh_and_latest_traj;

        for (int i=0; i<veh_names.size(); i++){
            string veh_name = veh_names[i];
            int start_ind = closest_ind(timestamps[veh_name], end_timestamp-traj_time_length);
            int end_ind = closest_ind(timestamps[veh_name], end_timestamp);
            for (int j=start_ind; j<end_ind; j++){
                circle(traj_img, vicon_coord_to_top_down_img_plane_coord(trajs[veh_name][j]), circle_radius/resolution, colors[i], circle_thickness/resolution);
            }
        }

        Mat traj_warped;
        warpPerspective(traj_img, traj_warped, H, frame.size() );

        Mat gray,gray_inv,src1final,src2final;
        cvtColor(traj_warped,gray,CV_BGR2GRAY);
        threshold(gray,gray,0,255,CV_THRESH_BINARY);
        //adaptiveThreshold(gray,gray,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,5,4);
        bitwise_not ( gray, gray_inv );
        frame.copyTo(src1final,gray_inv);
        traj_warped.copyTo(src2final,gray);
        Mat finalImage = src1final+src2final;

        // Display the resulting frame
        imshow( "Frame", finalImage );

        if (write_video){
            video.write(finalImage);
        }

        // Press  ESC on keyboard to exit
        char c=(char)waitKey(25);
        if(c==27)
          break;
    }

    // When everything done, release the video capture object
    cap.release();
    video.release();

    // Closes all the frames
    destroyAllWindows();

    return 0;
}
