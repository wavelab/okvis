/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Aug 17, 2017
 *      Author: Leo Koppel (l.koppel@uwaterloo.ca)
 *********************************************************************************/

/**
 * @file ImageViewer.hpp
 * @brief Header file for the ImageViewer class.
 * @author Leo Koppel
 */

#ifndef INCLUDE_OKVIS_IMAGEVIEWER_HPP_
#define INCLUDE_OKVIS_IMAGEVIEWER_HPP_

#include <opencv2/opencv.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/threadsafe/ThreadsafeQueue.hpp>

namespace okvis {

/**
 * Image viewer: displays images in a popup window
 */
class ImageViewer
{
 public:
  ImageViewer(const okvis::VioParameters& parameters) {
    if(parameters.visualization.displayImages && !parameters.publishing.publishImages){
      // Set up windows so things don't crash on Mac OS
      for (size_t im = 0; im < parameters.nCameraSystem.numCameras(); im++) {
        std::stringstream windowname;
        windowname << "OKVIS camera " << im;
        cv::namedWindow(windowname.str());
      }
    }
  }

  /// @brief Save images for later display
  /// (needed because OSX won't allow threaded display)
  void saveImagesAsCallback(const std::vector<cv::Mat>& images) {
    displayImages_.PushNonBlockingDroppingIfFull(images, 1);
  }

  /// @brief Draw any saved images
  void display() {
    std::vector<cv::Mat> out_images;
    if (displayImages_.Empty())
      return;
    if (!displayImages_.PopBlocking(&out_images) || out_images.empty())
      return;

    for (size_t im = 0; im < out_images.size(); im++) {
      std::stringstream windowname;
      windowname << "OKVIS camera " << im;
      cv::imshow(windowname.str(), out_images[im]);
    }
    cv::waitKey(1);
  }

  /// The queue containing the actual display images
  okvis::threadsafe::ThreadSafeQueue<std::vector<cv::Mat>> displayImages_;
};

} /* namespace okvis */

#endif /* INCLUDE_OKVIS_IMAGEVIEWER_HPP_ */
