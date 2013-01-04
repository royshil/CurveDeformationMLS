/*
 *  std.h
 *  CurveMatching
 *
 *  Created by Roy Shilkrot on 11/28/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 */

#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <set>
#include <fstream>
#include <iostream>
#include <limits>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

#include <dirent.h>

bool hasEnding (std::string const &fullString, std::string const &ending);
bool hasEndingLower (string const &fullString_, string const &_ending);
void open_imgs_dir(const char* dir_name, std::vector<std::string>& images_names);