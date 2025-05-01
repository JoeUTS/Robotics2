#include "image_controller.h"

imageController::imageController(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg, std::shared_ptr<rclcpp::Node> parentNode) 
  : lastCameraMsg_(*incomingMsg), parentNode_(parentNode) {
  
  // Load YOLO model.
  std::string packageShareDir = ament_index_cpp::get_package_share_directory("picasso_bot");

  std::string pathImageClasses = packageShareDir + "/config/coco-classes.txt";
  load_class_list(pathImageClasses);

  std::string pathYOLO = packageShareDir + "/local/yolov5s-seg.onnx";
  load_net(pathYOLO);
}

sensor_msgs::msg::Image imageController::updateCameraImage(const realsense2_camera_msgs::msg::RGBD::SharedPtr incomingMsg) {
  std::unique_lock<std::mutex> lck(mutex_);
  lastCameraMsg_ = *incomingMsg;
  sensor_msgs::msg::Image cameraImage = lastCameraMsg_.rgb;
  lck.unlock();

  return cameraImage;
}

realsense2_camera_msgs::msg::RGBD imageController::getStoredImage(void) {
  std::unique_lock<std::mutex> lck(mutex_);
  realsense2_camera_msgs::msg::RGBD msg = lastCameraMsg_;
  lck.unlock();

  return msg;
}

void imageController::editImageQuantize(cv::Mat &image, const int colourSteps) {
  // Prepare image.
  cv::Mat data;
  image.convertTo(data, CV_32F);
  data = data.reshape(1, data.total());  // Place each pixel in its own row.

  // Perform K-means clustering
  cv::Mat labels, centers;
  cv::kmeans(data, colourSteps, labels, cv::TermCriteria(CV_TERMCRIT_ITER, 10, 1.0), 3, 
                                                        cv::KMEANS_PP_CENTERS, centers);

  // Reshape each to row of RGB pixels
  centers = centers.reshape(3, centers.rows);
  data = data.reshape(3, data.rows);

  // Replace each pixel with cluster value.
  cv::Vec3f *pixels = data.ptr<cv::Vec3f>();
  for (size_t i = 0; i < data.rows; i++) {
    int centerId = labels.at<int>(i);
    pixels[i] = centers.at<cv::Vec3f>(centerId);
  }

  // Allocate back to input image.
  int imageType = image.type();
  image = data.reshape(image.channels(), image.rows);
  image.convertTo(image, imageType);
}

cv::Scalar imageController::getNextColour(void) {
  if (colourIndex_ >= COLOURS_LIST.size() - 1) {
    colourIndex_ = 0;
  }

  cv::Scalar colour = COLOURS_LIST.at(colourIndex_);
  colourIndex_++;

  return colour;
}

cv::Mat imageController::msg2Mat(const sensor_msgs::msg::Image &imageMsg) {
  cv_bridge::CvImagePtr cvPtr;
  cv::Mat image;

  if (imageMsg.encoding == "16UC1" || imageMsg.encoding == "32FC1") {
    // Depth image.
    cvPtr = cv_bridge::toCvCopy(imageMsg);
    image = cvPtr->image;
    double min, max;
    cv::minMaxLoc(image, &min, &max);
    image.convertTo(image, CV_8UC1, 255.0 / max);

  } else if (imageMsg.encoding == "mono8") {
    // Grayscale image
    cvPtr = cv_bridge::toCvCopy(imageMsg, "mono8");
    image = cvPtr->image;

  } else {
    // Colour image.
    // NOTE: Camera encoding says RGB but is actually BGR.
    cvPtr = cv_bridge::toCvCopy(imageMsg, "bgr8");
    image = cvPtr->image;
  }

  if (imageMsg.encoding == "16UC1" || imageMsg.encoding == "32FC1") {
    // Depth image.
    cvPtr = cv_bridge::toCvCopy(imageMsg);
    image = cvPtr->image;
    double min, max;
    cv::minMaxLoc(image, &min, &max);
    image.convertTo(image, CV_8UC1, 255.0 / max);

  } else if (imageMsg.encoding == "mono8") {
    // Grayscale image
    cvPtr = cv_bridge::toCvCopy(imageMsg, "mono8");
    image = cvPtr->image;

  } else {
    // Colour image.
    // NOTE: Camera encoding says RGB but is actually BGR.
    cvPtr = cv_bridge::toCvCopy(imageMsg, "bgr8");
    image = cvPtr->image;
  }

  return image;
}

void imageController::detectEdges(cv::Mat &image, float thresh) {
  int ave = calculateAverageIntensity(image);

  // Get canny values.
  int lower = round(std::max(double(0), (1.0 - thresh) * ave));
  int upper = round(std::min(double(255), (1.0 + thresh) * ave));

  // Find edges
  cv::Canny(image, image, lower, upper);
}

std::map<int, std::shared_ptr<Contour>> imageController::getToolpaths(cv::Mat &image, const bool normalise, const bool center) {
  // Find contours.
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  // Find max and min
  bool xIsLarger = image.cols > image.rows;

  // convert to geometry msg
  std::vector<std::vector<std::shared_ptr<geometry_msgs::msg::Point>>> convertedContours;
  convertedContours.reserve(contours.size());

  for (int i = 0; i < contours.size() - 1; i++) {
    // Skip invalid
    if (contours.at(i).size() < 2){
      continue;
    }

    std::vector<std::shared_ptr<geometry_msgs::msg::Point>> convertedPoints;
    convertedPoints.reserve(contours.at(i).size());

    std::for_each(contours.at(i).begin(), contours.at(i).end(), [&](cv::Point &pointCV) {
      // convert to geometry msg.
      std::shared_ptr<geometry_msgs::msg::Point> pointMsg = std::make_shared<geometry_msgs::msg::Point>();
      pointMsg->x = pointCV.x;
      pointMsg->y = pointCV.y;
      pointMsg->z = 0.0;

      // Normalise largest dimention to [0-1].
      int maxDim = 1;
      if (normalise && !image.empty()) {
        maxDim = xIsLarger ? image.cols : image.rows;
        pointMsg->x /= maxDim;
        pointMsg->y /= maxDim;
      }

      // Center
      if (center && !image.empty()) {
        pointMsg->x -= image.cols / maxDim / 2; // maxDim = 1 if not normalised.
        pointMsg->y -= image.cols / maxDim / 2;
      }
      
      convertedPoints.push_back(pointMsg);
    });

    convertedContours.push_back(convertedPoints);
  }

  // Build contour list
  if (convertedContours.empty()) {
    return std::map<int, std::shared_ptr<Contour>>();
  }
  
  std::map<int, std::shared_ptr<Contour>> contoursList;

  for (int i = 0; i < convertedContours.size() - 1; i++) {
    std::shared_ptr<Contour> contourPtr = std::make_shared<Contour>(i, convertedContours.at(i));
    contoursList.insert(std::pair<int, std::shared_ptr<Contour>>(i,  contourPtr));
  }

  return contoursList;
}

int imageController::calculateAverageIntensity(cv::Mat &image) {
  cv::Scalar mean = cv::mean(image);
  return round(mean[0]);
}

void imageController::generateArt(void) {
  const int kernalSize = 3;
  const int blurPasses = 1;
  const int colourSteps = 3;

  std::unique_lock<std::mutex> lck(mutex_);
  cv::Mat imageRGB = msg2Mat(lastCameraMsg_.rgb);
  lck.unlock();

  // Return on empty msg.
  if (imageRGB.empty()) {
    //detectionRunning_ = false;
    return;
  }
  
  for (int i = 0; i < blurPasses; i++) {
    editImageBlurMedian(imageRGB, kernalSize);
    editImageBlurGaussian(imageRGB, kernalSize);
  }
  
  editImageQuantize(imageRGB, colourSteps);

  cv::Mat edges = imageRGB.clone();
  editImageGreyscale(edges);
  detectEdges(edges, 0.1);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  cv::drawContours(imageRGB, contours, -1, cv::Scalar(0, 0, 255));
  
  //detectionRunning_ = false;
}



void imageController::load_class_list(std::string &path) {
  std::ifstream ifs(path);
  std::string line;

  while (getline(ifs, line)) {
    classList_.push_back(line);
  }
}

void imageController::load_net(std::string &path) {   
    auto result = cv::dnn::readNet(path);
    result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
 
    net_ = result;
}

cv::Mat imageController::format_yolov5(const cv::Mat &source) {
  int col = source.cols;
  int row = source.rows;
  int _max = std::max(col, row);
  cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
  source.copyTo(result(cv::Rect(0, 0, col, row)));
  cv::resize(result, result, cv::Size(INPUT_WIDTH, INPUT_HEIGHT));
  return result;
}

std::vector<DetectedObject> imageController::detectSegment(cv::Mat &image) {

  cv::Mat blob;

        // Format the input image to fit the model input requirements
        auto input_image = format_yolov5(image);

        // Convert the image into a blob and set it as input to the network
        cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
        net_.setInput(blob);

        // Get the names of the output layers
        std::vector<std::string> output_layer_names = net_.getUnconnectedOutLayersNames();
        if (output_layer_names.size() != 2) {
            std::cerr << "Error: Expected 2 output layers for segmentation model, but got " << output_layer_names.size() << std::endl;
            // Handle error appropriately, maybe return empty vector
            return {};
        }

        std::vector<cv::Mat> outputs;
        net_.forward(outputs, output_layer_names);

        if (outputs.size() != 2) {
             std::cerr << "Error: Expected 2 output tensors, but got " << outputs.size() << std::endl;
             return {};
        }

        // Based on debug output:
        // outputs[0] is the mask output (prototypes) - shape 1x32x160x160
        // outputs[1] is the detection output (boxes, scores, coeffs) - shape 1x25200x117
        cv::Mat mask_prototypes = outputs[0]; // Prototypes are in outputs[0]
        cv::Mat detection_output = outputs[1]; // Detections+coeffs are in outputs[1]


        // Scaling factors to map the bounding boxes back to original image size
        // These factors are for the bounding boxes, not necessarily the mask prototypes directly
        float x_factor = image.cols / (float)INPUT_WIDTH;
        float y_factor = image.rows / (float)INPUT_HEIGHT;

        // Data from the detection output layer (now outputs[1])
        float *detection_data = (float *)detection_output.data;
        // CORRECTED: Use size[1] for rows and size[2] for dimensions based on shape 1x25200x117
        const int rows = detection_output.size[1]; // Number of detection proposals (25200)
        const int dimensions = detection_output.size[2]; // Dimensions per proposal (117)

        // Calculate offset for mask coefficients
        // Common YOLOv5-Seg structure: 5 (box+conf) + num_classes + num_mask_coefficients
        const int bbox_conf_scores_size = 5 + classList_.size(); // 5 + 80 = 85
        const int mask_coeffs_offset = bbox_conf_scores_size; // 85
        const int mask_coeff_count = dimensions - mask_coeffs_offset; // 117 - 85 = 32

        if (mask_coeff_count <= 0) {
             std::cerr << "Error: Could not determine mask coefficient count from output dimensions." << std::endl;
             return {};
        }

        // Data from the mask output layer (prototypes - now outputs[0])
        int num_masks_proto = mask_prototypes.size[1]; // 32
        int mask_proto_height = mask_prototypes.size[2]; // 160
        int mask_proto_width = mask_prototypes.size[3]; // 160

        // Check if mask coefficient count matches number of prototypes
        if (num_masks_proto != mask_coeff_count) {
            std::cerr << "Error: Number of mask coefficients in detection output (" << mask_coeff_count
                      << ") does not match number of masks in prototype output (" << num_masks_proto << ")."
                      << " Segmentation will not work correctly." << std::endl;
            return {}; // Return empty vector as segmentation is not possible
        }

        std::vector<int> class_ids;        // Stores class IDs of detections
        std::vector<float> confidences;      // Stores confidence scores of detections
        std::vector<cv::Rect> boxes;          // Stores bounding boxes
        std::vector<std::vector<float>> mask_coeffs; // Stores mask coefficients for each detection

        // Loop through all the rows to process predictions
        for (int i = 0; i < rows; ++i) {

            float confidence = detection_data[4];

            // Process only detections with confidence above the threshold
            if (confidence >= CONFIDENCE_THRESHOLD) {

                // Get class scores and find the class with the highest score
                float * classes_scores = detection_data + 5;
                cv::Mat scores_mat(1, classList_.size(), CV_32FC1, classes_scores);
                cv::Point class_id;
                double max_class_score;
                minMaxLoc(scores_mat, 0, &max_class_score, 0, &class_id);

                // If the class score is above the threshold, store the detection
                if (max_class_score > SCORE_THRESHOLD) {

                    confidences.push_back(confidence);
                    class_ids.push_back(class_id.x);

                    // Calculate the bounding box coordinates
                    float x = detection_data[0];
                    float y = detection_data[1];
                    float w = detection_data[2];
                    float h = detection_data[3];
                    int left = int((x - 0.5 * w) * x_factor);
                    int top = int((y - 0.5 * h) * y_factor);
                    int width = int(w * x_factor);
                    int height = int(h * y_factor);

                    // Ensure bounding box is within image bounds
                    left = std::max(0, left);
                    top = std::max(0, top);
                    width = std::min(image.cols - left, width);
                    height = std::min(image.rows - top, height);

                    boxes.push_back(cv::Rect(left, top, width, height));

                    // Store mask coefficients
                    std::vector<float> current_coeffs(mask_coeff_count);
                    std::copy(detection_data + mask_coeffs_offset,
                              detection_data + dimensions,
                              current_coeffs.begin());
                    mask_coeffs.push_back(current_coeffs);
                }
            }
            // Move to the next detection row
            detection_data += dimensions;
        }

        // Apply Non-Maximum Suppression
        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);

        // Prepare output vector
        std::vector<DetectedObject> output;
        output.reserve(nms_result.size()); // Reserve space for efficiency

        // Reshape the mask prototypes for matrix multiplication
        // From [1, num_masks, mask_h, mask_w] to [num_masks, mask_h * mask_w]
        cv::Mat mask_prototypes_reshaped = mask_prototypes.reshape(1, num_masks_proto);

        // Calculate the scaling factor used in format_yolov5 to get to the INPUT_WIDTH x INPUT_HEIGHT size
        float scale_factor_input = (float)INPUT_WIDTH / std::max(image.cols, image.rows);
        // Calculate the dimensions of the original image content within the INPUT_WIDTH x INPUT_HEIGHT space
        cv::Size original_size_in_input(static_cast<int>(image.cols * scale_factor_input), static_cast<int>(image.rows * scale_factor_input));


        for (size_t i = 0; i < nms_result.size(); ++i) {
            int idx = nms_result[i]; // Index in the original lists (before NMS)

            // Create a DetectedObject for this result
            DetectedObject result;
            result.class_id = class_ids[idx];
            result.confidence = confidences[idx];
            result.box = boxes[idx]; // Bounding box on original image scale

            // --- Segmentation Part ---

            // 1. Get mask coefficients for this detection
            cv::Mat coeffs_mat(1, mask_coeff_count, CV_32FC1, mask_coeffs[idx].data());

            // 2. Combine coefficients with prototypes
            // Result is a raw mask [1, mask_h * mask_w]
            cv::Mat object_mask_raw;
            cv::gemm(coeffs_mat, mask_prototypes_reshaped, 1.0, cv::Mat(), 0.0, object_mask_raw);

            // 3. Reshape the raw mask back to 2D [mask_h, mask_w] (160x160)
            object_mask_raw = object_mask_raw.reshape(1, mask_proto_height);

            // --- Corrected Mask Resizing and Thresholding ---
            // Resize the raw mask to the INPUT_WIDTH x INPUT_HEIGHT size
            cv::Mat resized_mask_to_input;
            cv::resize(object_mask_raw, resized_mask_to_input, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), 0, 0, cv::INTER_LINEAR); // Use linear interpolation

            // Define the region corresponding to the original image content within the INPUT_WIDTH x INPUT_HEIGHT mask
            // The original image was placed at (0,0) in the padded _max x _max canvas, then resized to INPUT_WIDTH x INPUT_HEIGHT
            cv::Rect original_roi_in_input(0, 0, original_size_in_input.width, original_size_in_input.height);

            // Crop the resized mask to the original image dimensions
            cv::Mat mask_on_original_scale = resized_mask_to_input(original_roi_in_input);

            // Crop the mask (now at original image scale) using the bounding box
            cv::Mat cropped_mask_at_original_scale = mask_on_original_scale(boxes[idx]);

            // Convert the cropped mask to 8-bit unsigned
            cv::Mat cropped_mask_at_original_scale_8u;
            cropped_mask_at_original_scale.convertTo(cropped_mask_at_original_scale_8u, CV_8U, 255.0); // Scale to 0-255 range

            // Apply thresholding to get the binary mask
            cv::Mat binary_mask;
            // Threshold on the 8-bit image (threshold value 128 corresponds to 0.5 scaled by 255)
            cv::threshold(cropped_mask_at_original_scale_8u, binary_mask, 128, 255, cv::THRESH_BINARY);

            // Store the binary mask in the DetectedObject struct (it's already the size of the box)
            result.mask = binary_mask;

            // --- Drawing Part (Optional, but kept for visualization) ---
            // Find contours of the binary mask (which is already the size of the box)
            std::vector<std::vector<cv::Point>> contours;
            // cv::findContours now receives a CV_8UC1 image
            cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Get the color for the border, cycling through the list
            cv::Scalar color = COLOURS_LIST[i % COLOURS_LIST.size()];

            // Draw the contours on the original image
            // Use the offset parameter in drawContours to shift points by the box top-left corner
            cv::drawContours(image, contours, -1, color, 2, cv::LINE_8, cv::noArray(), 0, boxes[idx].tl()); // Thickness 2 for border

            // Draw the label
            std::string label = classList_[class_ids[idx]] + ":" + cv::format("%.2f", confidences[idx]);
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseLine);
            int label_y = boxes[idx].y - 5;
            if (label_y < labelSize.height && boxes[idx].y + boxes[idx].height + labelSize.height < image.rows) {
                 label_y = boxes[idx].y + boxes[idx].height + labelSize.height; // Draw below if too high and space available
            } else if (label_y < 0) {
                 label_y = boxes[idx].y + labelSize.height; // Draw below if too high and no space above
            }


            cv::putText(image, label, cv::Point(boxes[idx].x, label_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
            // --- End Drawing Part ---

            // Add the populated DetectedObject to the output vector
            output.push_back(result);
        }

        return output;
}



std::vector<DetectedObject> imageController::detect(cv::Mat &image) {
  cv::Mat blob;

  // Format the input image to fit the model input requirements
  auto input_image = format_yolov5(image);
  
  // Convert the image into a blob and set it as input to the network
  cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
  net_.setInput(blob);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());

  // Scaling factors to map the bounding boxes back to original image size
  float x_factor = input_image.cols / INPUT_WIDTH;
  float y_factor = input_image.rows / INPUT_HEIGHT;
  
  float *data = (float *)outputs[0].data;

  const int dimensions = 85;
  const int rows = 25200;
  
  std::vector<int> class_ids; // Stores class IDs of detections
  std::vector<float> confidences; // Stores confidence scores of detections
  std::vector<cv::Rect> boxes;   // Stores bounding boxes

 // Loop through all the rows to process predictions
  for (int i = 0; i < rows; ++i) {

      // Get the confidence of the current detection
      float confidence = data[4];

      // Process only detections with confidence above the threshold
      if (confidence >= CONFIDENCE_THRESHOLD) {
          
          // Get class scores and find the class with the highest score
          float * classes_scores = data + 5;
          cv::Mat scores(1, classList_.size(), CV_32FC1, classes_scores);
          cv::Point class_id;
          double max_class_score;
          minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

          // If the class score is above the threshold, store the detection
          if (max_class_score > SCORE_THRESHOLD) {

              confidences.push_back(confidence);
              class_ids.push_back(class_id.x);

              // Calculate the bounding box coordinates
              float x = data[0];
              float y = data[1];
              float w = data[2];
              float h = data[3];
              int left = int((x - 0.5 * w) * x_factor);
              int top = int((y - 0.5 * h) * y_factor);
              int width = int(w * x_factor);
              int height = int(h * y_factor);
              boxes.push_back(cv::Rect(left, top, width, height));
          }
      }

      data += 85;
  }

  // Apply Non-Maximum Suppression
  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);

  // Draw the NMS filtered boxes and push results to output
  std::vector<DetectedObject> output;

  for (int i = 0; i < nms_result.size(); i++) {
      int idx = nms_result[i];

      // Only push the filtered detections
      DetectedObject result;
      result.class_id = class_ids[idx];
      result.confidence = confidences[idx];
      result.box = boxes[idx];
      output.push_back(result);

      // Draw the final NMS bounding box and label
      cv::rectangle(image, boxes[idx], cv::Scalar(0, 255, 0), 3);
      std::string label = classList_[class_ids[idx]];
      cv::putText(image, label, cv::Point(boxes[idx].x, boxes[idx].y - 5), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 2);
  }

  return output;
}




void imageController::draw_label(cv::Mat& input_image, std::string label, int left, int top) {
    // Display the label at the top of the bounding box.
    int baseLine;
    cv::Size label_size = cv::getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
    top = std::max(top, label_size.height);
    cv::Point tlc = cv::Point(left, top);
    cv::Point brc = cv::Point(left + label_size.width, top + label_size.height + baseLine);

    // Draw rectangle.
    cv::rectangle(input_image, tlc, brc, COLOURS_LIST[0], cv::FILLED);
    cv::putText(input_image, label, cv::Point(left, top + label_size.height), FONT_FACE, FONT_SCALE, COLOURS_LIST[5], THICKNESS);
}

std::vector<cv::Mat> imageController::detectPreProcess(cv::Mat &inputImage) {
  cv::Mat blob;
  cv::dnn::blobFromImage(inputImage, 
                        blob, 
                        1./255., 
                        cv::Size(INPUT_WIDTH, INPUT_HEIGHT), 
                        cv::Scalar(), 
                        true, 
                        false);

  net_.setInput(blob);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());

  return outputs;
}

cv::Mat imageController::detectPostProcess(cv::Mat input_image, std::vector<cv::Mat> &outputs) {
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  
  // Resizing factor.
  float x_factor = input_image.cols / INPUT_WIDTH;
  float y_factor = input_image.rows / INPUT_HEIGHT;
  float *data = (float *)outputs[0].data;

  const int dimensions = 85;
  const int rows = 25200; // 25200 for default size 640.

  for (int i = 0; i < rows; ++i) {
    float confidence = data[4];
    if (confidence >= CONFIDENCE_THRESHOLD) {
      float * classes_scores = data + 5;
      cv::Mat scores(1, classList_.size(), CV_32FC1, classes_scores);
      cv::Point class_id;
      double max_class_score;
      cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

      if (max_class_score > SCORE_THRESHOLD) {
        confidences.push_back(confidence);
        class_ids.push_back(class_id.x);
        // Center.
        float cx = data[0];
        float cy = data[1];

        // Box dimension.
        float w = data[2];
        float h = data[3];

        // Bounding box coordinates.
        int left = int((cx - 0.5 * w) * x_factor);
        int top = int((cy - 0.5 * h) * y_factor);
        int width = int(w * x_factor);
        int height = int(h * y_factor);

        // Store good detections in the boxes vector.
        boxes.push_back(cv::Rect(left, top, width, height));
      }
    }

    // Jump to the next row.
    data += 85;
  }

  // Perform Non-Maximum Suppression and draw predictions.
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
  for (int i = 0; i < indices.size(); i++) {
    int idx = indices[i];
    cv::Rect box = boxes[idx];
    int left = box.x;
    int top = box.y;
    int width = box.width;
    int height = box.height;

    // Draw bounding box.
    cv::rectangle(input_image, cv::Point(left, top), cv::Point(left + width, top + height), COLOURS_LIST[3], 3*THICKNESS);
    
    // Draw class labels.
    std::string label = cv::format("%.2f", confidences[idx]);
    label = classList_[class_ids[idx]] + ":" + label;
    draw_label(input_image, label, left, top);
  }

  return input_image;
}

cv::Mat imageController::generateMask(cv::Mat &image) {
  std::vector<DetectedObject> detectedObjects = detect(image);
  /*
  std::vector<DetectedObject> detections = imageController_->detectSegment(detectionImage);
  for (const auto& obj : detections) {
    // You can now use obj.mask (a binary cv::Mat) for further processing
    // For example, to apply the mask to the original image region:
    cv::Mat object_region = detectionImage(obj.box);
    cv::Mat masked_object;
    object_region.copyTo(masked_object, obj.mask); // Apply the binary mask
    cv::imshow("Masked Object", masked_object);
  }
  */
  //std::vector<cv::Mat> detections = imageController_->detectPreProcess(detectionImage);
  //cv::Mat img = imageController_->detectPostProcess(detectionImage, detections);
  //cv::imshow("image", detectionImage);
  //cv::waitKey(1);

  // - Return an array of detected people with location, size, bounding box ect.
  std::list<geometry_msgs::msg::Vector3> personImageLocations;


  
}
