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

#ifndef ONILOGREADER_H_
#define ONILOGREADER_H_

#include <stack>

#include <OpenNI.h>

#include "LogReader.h"

class OniLogReader : public LogReader
{
    public:
        OniLogReader(std::string file, bool flipColors);

        virtual ~OniLogReader();

        void getNext();

        void getBack();

        int getNumFrames();

        bool hasMore();

        bool rewound();

        void fastForward(int frame);

        const std::string getFile();

        void setAuto(bool value);

        std::stack<int> filePointers;

    private:
        openni::Device oni_device;
        openni::VideoStream depth_stream;
        openni::VideoStream rgb_stream;
};

#endif /* ONILOGREADER_H_ */
