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

// Read a combination of depth + RGB frame from the given depth and RGB streams.
//
// Takes in the last frame indices that were read to determine whether there is
// another frame to read; this is important because OpenNI2's `readFrame()`
// hangs on 100% CPU when `setSpeed(-1)` and there is no more frame to read.
//
// This function can deal with skipped frames in the streams.
// It will also skip depth/RGB frames that cannot form a valid frame combination
// because their timestamps are too far away (1/fps/2 seconds) from a corresponding
// RGB/depth frame.
// If `printSkips` is true, a message will be written to std::cerr in that case.
//
// Returns true if a combination of depth + RGB frame was read; false if no
// further frame combination can be read from the streams.
static bool getNextDepthAndRGBFrame(
    openni::VideoStream& depth_stream,
    openni::VideoStream& rgb_stream,
    const int numFramesDepth,
    const int numFramesRGB,
    const int lastFrameIndexDepth,
    const int lastFrameIndexRGB,
    openni::VideoFrameRef& out_depth_frame,
    openni::VideoFrameRef& out_rgb_frame,
    bool printSkips)
{

    // `getFrameIndex()` is is like a frame_number, but a property of the frame itself, starting with 1.
    // It may not be increasing monotonically because some frames may be missing,
    // for both real cameras (when the computer was too slow to process) and .oni recordings
    // (when the computer was too slow to record).

    if (lastFrameIndexDepth == numFramesDepth)
    {
        return false;
    }

    if (lastFrameIndexRGB == numFramesRGB)
    {
        return false;
    }

    // Note: readFrame() hangs on 100% CPU when setSpeed(-1) and there
    // is no more frame.
    oni_check("read depth", depth_stream.readFrame(&out_depth_frame));
    oni_check("read rgb", rgb_stream.readFrame(&out_rgb_frame));

    bool catchUp = true;

    const int64_t fps = 30;
    const int64_t maxTimeBetweenDepthAndRGB_usecs = 1000000/fps/2;

    while (catchUp)
    {

        uint64_t frameTimestampDepth = out_depth_frame.getTimestamp();
        uint64_t frameTimestampRGB = out_rgb_frame.getTimestamp();

        int64_t timeDiff = (int64_t) frameTimestampDepth - (int64_t) frameTimestampRGB;

        int frameIndexDepth = out_depth_frame.getFrameIndex();
        int frameIndexRGB = out_rgb_frame.getFrameIndex();

        bool depthLagsBehindRGB = timeDiff < -maxTimeBetweenDepthAndRGB_usecs;
        bool rgbLagsBehindDepth = timeDiff >  maxTimeBetweenDepthAndRGB_usecs;
        catchUp = depthLagsBehindRGB || rgbLagsBehindDepth;

        if (depthLagsBehindRGB)
        {
            if (frameIndexDepth == numFramesDepth)
            {
                return false;
            }
            if (printSkips)
            {
                std::cerr << "Skipping depth frame with index " << frameIndexDepth << " (timeDiff " << timeDiff << ")" << std::endl;
            }
            oni_check("read depth", depth_stream.readFrame(&out_depth_frame));
        }
        else if (rgbLagsBehindDepth && frameIndexRGB != numFramesRGB)
        {
            if (frameIndexRGB == numFramesRGB)
            {
                return false;
            }
            if (printSkips)
            {
                std::cerr << "Skipping colour frame with index " << frameIndexRGB << " (timeDiff " << timeDiff << ")" << std::endl;
            }
            oni_check("read rgb", rgb_stream.readFrame(&out_rgb_frame));
        }
    }
    // Depth and colour are reasonably close by

    return true;
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
    numFramesDepth = oni_device.getPlaybackControl()->getNumberOfFrames(depth_stream);
    numFramesRGB = oni_device.getPlaybackControl()->getNumberOfFrames(rgb_stream);
    finished = false;
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
    bool gotNextFrame = getNextDepthAndRGBFrame(
        depth_stream,
        rgb_stream,
        numFramesDepth,
        numFramesRGB,
        depth_frame.isValid() ? depth_frame.getFrameIndex() : 0,
        rgb_frame.isValid() ? rgb_frame.getFrameIndex() : 0,
        depth_frame,
        rgb_frame,
        true);

    if (!gotNextFrame) // end of stream
    {
      finished = true;
      return;
    }

    int frameIndexDepth = depth_frame.getFrameIndex();
    int frameIndexRGB = rgb_frame.getFrameIndex();

    uint64_t frameTimestampDepth = depth_frame.getTimestamp();
    uint64_t frameTimestampRGB = rgb_frame.getTimestamp();
    int64_t timeDiff = (int64_t) frameTimestampDepth - (int64_t) frameTimestampRGB;

    currentFrame = std::min(frameIndexDepth, frameIndexRGB);

    std::cerr
      << "Frame " << currentFrame
      << " (indices: depth " << frameIndexDepth << "/" << numFramesDepth
      << ", RGB " << frameIndexRGB << "/" << numFramesRGB << ")"
      << " (times: depth " << frameTimestampDepth << ", RGB " << frameTimestampRGB << ", diff " << timeDiff << ")..."
      << std::endl;

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
    return !finished;
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
