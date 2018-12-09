/**
 * MIT License
 *
 * Copyright (c) 2018 Venkatraman Narayanan, Amrish Baskaran
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 *
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 *
 *  @file    ObjectDetector.cpp
 *  @Author  Venkatraman Narayanan (vijay4313)
 *  @Author  Amrish Baskaran (amrish1222)
 *  @copyright  MIT
 *  @brief  ObjectDetector routine file
 */

#include "../include/ObjectDetector.h"

static const std::string OPENCV_WINDOW = "Image window";
/**
 * @brief Constructor
 * @param none
 * @return none
 */
ObjectDetector::ObjectDetector()
    : it_(nh_) {

  hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

  // subscribe to required topic image_raw
  im_sub_ = it_.subscribe("/ardrone/front/image_raw", 1,
                          &ObjectDetector::personDetector, this);
  // advertise to topic
  im_pub_ = it_.advertise("/camera_person_tracker/output_video", 1);

  // Publish detected pedestrians.
  pedestrians_pub_ = nh_.advertise < intelli_bot::Pedestrians
      > ("/person_detection/pedestrians", 1000);

  //cv::namedWindow(OPENCV_WINDOW);

}

/**
 * @brief Destructor
 * @param none
 * @return none
 */
ObjectDetector::~ObjectDetector() {
 // cv::destroyWindow(OPENCV_WINDOW);
}

/**
 * @brief routine to train
 * the ObjectDetector object
 * @param  none
 * @return none
 */
void ObjectDetector::personDetector(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat im_bgr = cv_ptr->image;

  cv::Mat im_gray;

  cv::cvtColor(im_bgr, im_gray, CV_BGR2GRAY);
  cv::equalizeHist(im_gray, im_gray);

  // HOG pedestrian detector
  std::vector < cv::Rect > detected_pedestrian;
  hog_.detectMultiScale(im_gray, detected_pedestrian, 0.0, cv::Size(4, 4),
                        cv::Size(0, 0), 1.05, 4);

  // Publish message of location and confident of detected pedestrians.
  // Draw detections from HOG to the screen.

  for (unsigned i = 0; i < detected_pedestrian.size(); i++) {
    // Draw on screen.
    cv::rectangle(im_bgr, detected_pedestrian[i], cv::Scalar(255));

    // Add to published message.
    intelli_bot::bbox pedestrian;
    pedestrian.center.x = detected_pedestrian[i].x
        - detected_pedestrian[i].width / 2;
    pedestrian.center.y = detected_pedestrian[i].y
        - detected_pedestrian[i].height / 2;
    pedestrian.width = detected_pedestrian[i].width;
    pedestrian.height = detected_pedestrian[i].height;
    pedestrians_msg.pedestrians.push_back(pedestrian);
  }
  pedestrians_pub_.publish(pedestrians_msg);
  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(cv_ptr->header, "bgr8", im_bgr).toImageMsg();
  //cv::imshow(OPENCV_WINDOW, im_bgr);
  //cv::waitKey(3);
  im_pub_.publish(out_msg);

}

