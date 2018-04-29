/*
 * Copyright (c) 2016 Rosimildo DaSilva <rosimildo@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */

#ifndef __cliOptions_h__
#define __cliOptions_h__

#include <vector>
#include <string>

/*
Inputs:
-i <v4l2 Device Name>
-b <bitrate> in Kb/s
-r <FPS>  - frames per second
-k <KeyInternal> - number of key I-Frames...
-s <WxH> - dimension
-z <WxH> - output dim.
-q <Min,Max>
-o <x.h264>,<z.nv12>
-t <secs> - duration of capture
*/

struct CmdLineOptions {
    std::string input;
    std::string rtmpUrl;
    size_t bitrate;
    size_t fps;
    size_t width;
    size_t height;
    size_t width_out;
    size_t height_out;
    size_t qMin;
    size_t qMax;
    size_t keyInterval;
    unsigned int duration;
};

/**
 * This method takes the command line options, parse them and set the proper options
 * in a structured format.
 */
extern int processCmdLineOptions(CmdLineOptions &options, int argc, char **argv);

#endif // __cliOptions_h__
