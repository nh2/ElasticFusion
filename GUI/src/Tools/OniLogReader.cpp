/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include <iostream>

#include <pangolin/utils/file_utils.h>
#include <OpenNI.h>

#include "OniLogReader.h"

static void oni_check(const std::string& operation, openni::Status status)
{
    if (status != openni::STATUS_OK)
    {
        std::string what = operation + " " + openni::OpenNI::getExtendedError();
        throw std::runtime_error(what);
    }
}

OniLogReader::OniLogReader(std::string file, bool flipColors)
 : LogReader(file, flipColors)
{
    // Variables of the LogReader superclass that we don't use.
    fp = NULL;
    depthReadBuffer = NULL;
    imageReadBuffer = NULL;
    decompressionBufferDepth = NULL;
    decompressionBufferImage = NULL;

    depthSize = Resolution::getInstance().numPixels() * 2;
    imageSize = Resolution::getInstance().numPixels() * 3;

    depth = (unsigned short *) new unsigned char[depthSize];
    rgb = (unsigned char *) new unsigned char[imageSize];

    assert(pangolin::FileExists(file.c_str()));

    const char * oniUrl = file.c_str();

    oni_check("initialize", openni::OpenNI::initialize());
    oni_check("open", oni_device.open(oniUrl));
    oni_check("get depth", depth_stream.create(oni_device, openni::SENSOR_DEPTH));
    oni_check("get rgb", rgb_stream.create(oni_device, openni::SENSOR_COLOR));

    assert(oni_device.isFile());

    // These work only if `oni_device.isFile()`.
    oni_device.getPlaybackControl()->setRepeatEnabled(false);
    oni_device.getPlaybackControl()->setSpeed(-1); // Set the playback in a manual mode i.e. read a frame whenever the application requests it
    int numFramesDepth = oni_device.getPlaybackControl()->getNumberOfFrames(depth_stream);
    int numFramesRGB = oni_device.getPlaybackControl()->getNumberOfFrames(rgb_stream);
    numFrames = std::min(numFramesDepth, numFramesRGB);

    depth_stream.start();
    rgb_stream.start();
}

OniLogReader::~OniLogReader()
{
    depth_stream.stop();
    rgb_stream.stop();
    oni_device.close();
    openni::OpenNI::shutdown();

    delete [] depth;
    delete [] rgb;
    depth = NULL;
    rgb = NULL;
}

void OniLogReader::getBack()
{
}

void OniLogReader::getNext()
{
    openni::VideoFrameRef depth_frame;
    openni::VideoFrameRef rgb_frame;

    oni_check("read depth", depth_stream.readFrame(&depth_frame));
    oni_check("read rgb", rgb_stream.readFrame(&rgb_frame));

    memcpy(depth, depth_frame.getData(), depthSize);
    memcpy(rgb, rgb_frame.getData(), imageSize);

    if(flipColors)
    {
        for(int i = 0; i < imageSize; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }
}

void OniLogReader::fastForward(int frame)
{
}

int OniLogReader::getNumFrames()
{
    return numFrames;
}

bool OniLogReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}

bool OniLogReader::rewound()
{
    return false;
}

const std::string OniLogReader::getFile()
{
    return file;
}

void OniLogReader::setAuto(bool value)
{

}
