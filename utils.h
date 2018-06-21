
/**

Utils for basic and general operation

*/

#ifndef __UTILS__
#define __UTILS__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>
#include <string>

#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "aw/vencoder.h"
#include "librtmp_send264.h"

using namespace std;
using namespace cv;

extern RTMPMetadata metaData;

/**
* creating directories if none existing
*/
int mkdirs(char *muldir);

void decode_sps_pps(VencHeaderData sps_pps_data);

void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator);


#endif
