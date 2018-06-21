
#include "utils.h"

/**
* creating directories if none existing
*/
int mkdirs(char *muldir) {
    int i, len;
    char str[512];
    strncpy(str, muldir, 512);
    len = strlen(str);
    for(i=0; i<len; i++) {
        if(str[i]=='/') {
            str[i] = '\0';
            if(access(str, 0)!=0) {
                if(mkdir(str, 0777)) {
                    return -1;
                }
            }
            str[i]='/';
        }
    }
    if(len > 0 && access(str, 0) != 0) {
        if(mkdir(str, 0777)) {
          return -1;
        }
    }
    return 0;
}


void decode_sps_pps(VencHeaderData sps_pps_data) {
    unsigned char *sps_pps_buf = sps_pps_data.pBuffer;
    int tail =0;
    int sps_head = 0, sps_length = 0;
    int pps_head = 0, pps_length = 0;
    while(tail < sps_pps_data.nLength) {
        if(sps_pps_buf[tail] == 0x67) {
            sps_head = tail;
        } else if(sps_pps_buf[tail] == 0x68) {
            pps_head = tail;
        }
        tail++;
    }
    sps_length = pps_head - sps_head - 4;
    pps_length = sps_pps_data.nLength - pps_head;
    metaData.nSpsLen = sps_length;
    metaData.nPpsLen = pps_length;
    metaData.Sps = (unsigned char*) malloc(sizeof(unsigned char) * sps_length);
    metaData.Pps = (unsigned char*) malloc(sizeof(unsigned char) * pps_length);
    memcpy(metaData.Sps, sps_pps_buf + sps_head, sps_length);
    memcpy(metaData.Pps, sps_pps_buf + pps_head, pps_length);
}



void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator) {
   std::ifstream file(filename.c_str(), ifstream::in);
   if (!file) {
       string error_message = "No valid input file was given, please check the given filename.";
       CV_Error(CV_StsBadArg, error_message);
   }
   string line, path, classlabel;
   while (getline(file, line)) {
       stringstream liness(line);
       getline(liness, path, separator);
       getline(liness, classlabel);
       if(!path.empty() && !classlabel.empty()) {
           images.push_back(imread(path, 0));
           labels.push_back(atoi(classlabel.c_str()));
       }
   }
}
