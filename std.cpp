/*
 *  std.cpp
 *  CurveMatching
 *
 *  Created by Roy Shilkrot on 1/1/13.
 *  Copyright 2013 MIT. All rights reserved.
 *
 */
#include "std.h"

bool hasEnding (const std::string &fullString, const std::string &ending)
{
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

bool hasEndingLower (const string  &fullString_, const string  &_ending)
{
	string fullstring = fullString_, ending = _ending;
	transform(fullString_.begin(),fullString_.end(),fullstring.begin(),::tolower); // to lower
	return hasEnding(fullstring,ending);
}


void open_imgs_dir(const char* dir_name, std::vector<std::string>& images_names) {
	if (dir_name == NULL) {
		return;
	}
	
	string dir_name_ = string(dir_name);
	vector<string> files_;
	
	DIR *dp;
	struct dirent *ep;     
	dp = opendir (dir_name);
	
	if (dp != NULL)
	{
		while (ep = readdir (dp)) {
			if (ep->d_name[0] != '.')
				files_.push_back(ep->d_name);
		}
		
		(void) closedir (dp);
	}
	else {
		cerr << ("Couldn't open the directory");
		return;
	}
	for (unsigned int i=0; i<files_.size(); i++) {
		if (files_[i][0] == '.' || !(hasEndingLower(files_[i],"jpg")||hasEndingLower(files_[i],"png"))) {
			continue;
		}
		images_names.push_back(string(dir_name) + files_[i]);
	}
}	
