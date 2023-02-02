/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/core/Signature.h"
#include <opencv2/highgui/highgui.hpp>

#include <rtabmap/utilite/UtiLite.h>

namespace rtabmap
{

Signature::Signature() :
    _id(0), // invalid id
    _mapId(-1),
    _sec(0),
    _nsec(0),
    _weight(0),
    _saved(false),
    _modified(true),
    _enabled(false),
    _invalidWordsCount(0)
{
}

Signature::Signature(
        int id,
        int mapId,
        int weight,
        double stamp,
        const std::string & label,
        const Transform & pose,
        const Transform & groundTruthPose,
        const SensorData & sensorData):
    _id(id),
    _mapId(mapId),
    _sec(0),
    _nsec(0),
    _weight(weight),
    _label(label),
    _saved(false),
    _modified(true),
    _enabled(false),
    _invalidWordsCount(0),
    _pose(pose),
    _groundTruthPose(groundTruthPose),
    _sensorData(sensorData)
{
    std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);

    if(_sensorData.id() == 0)
    {
        _sensorData.setId(id);
    }
    UASSERT(_sensorData.id() == _id);
}

Signature::Signature(const SensorData & data) :
    _id(data.id()),
    _mapId(-1),
    _sec(data.sec()),
    _nsec(data.nsec()),
    _weight(0),
    _label(""),
    _saved(false),
    _modified(true),
    _enabled(false),
    _invalidWordsCount(0),
    _pose(Transform::getIdentity()),
    _groundTruthPose(data.groundTruth()),
    _sensorData(data)
{

}

Signature::~Signature()
{
    //UDEBUG("id=%d", _id);
}

void Signature::changeWordsRef(int oldWordId, int activeWordId)
{
    std::list<int> words = uValues(_words, oldWordId);
    if(words.size())
    {
        if(oldWordId<=0)
        {
            _invalidWordsCount-=(int)_words.erase(oldWordId);
            UASSERT(_invalidWordsCount>=0);
        }
        else
        {
            _words.erase(oldWordId);
        }

        _wordsChanged.insert(std::make_pair(oldWordId, activeWordId));
        for(std::list<int>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
        {
            _words.insert(std::pair<int, int>(activeWordId, (*iter)));
        }
    }
}

void Signature::setWords(const std::multimap<int, int> & words,
        const std::vector<cv::KeyPoint> & keypoints,
        const std::vector<cv::Point3f> & points,
        const cv::Mat & descriptors)
{
    UASSERT_MSG(descriptors.empty() || descriptors.rows == (int)words.size(), uFormat("words=%d, descriptors=%d", (int)words.size(), descriptors.rows).c_str());
    UASSERT_MSG(points.empty() || points.size() == words.size(),  uFormat("words=%d, points=%d", (int)words.size(), (int)points.size()).c_str());
    UASSERT_MSG(keypoints.empty() || keypoints.size() == words.size(),  uFormat("words=%d, descriptors=%d", (int)words.size(), (int)keypoints.size()).c_str());
    UASSERT(words.empty() || !keypoints.empty() || !points.empty() || !descriptors.empty());

    _invalidWordsCount = 0;
    for(std::multimap<int, int>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
    {
        if(iter->first<=0)
        {
            ++_invalidWordsCount;
        }
        // make sure indexes are all valid!
        UASSERT_MSG(iter->second >=0 && iter->second < (int)words.size(), uFormat("iter->second=%d words.size()=%d", iter->second, (int)words.size()).c_str());
    }

    _enabled = false;
    _words = words;
    _wordsKpts = keypoints;
    _words3 = points;
    _wordsDescriptors = descriptors.clone();
}

bool Signature::isBadSignature() const
{
    return _words.size()-_invalidWordsCount <= 0;
}

void Signature::removeAllWords()
{
    _words.clear();
    _wordsKpts.clear();
    _words3.clear();
    _wordsDescriptors = cv::Mat();
    _invalidWordsCount = 0;
}

void Signature::setWordsDescriptors(const cv::Mat & descriptors)
{
    if(descriptors.empty())
    {
        if(_wordsKpts.empty() && _words3.empty())
        {
            removeAllWords();
        }
        else
        {
            _wordsDescriptors = cv::Mat();
        }
    }
    else
    {
        UASSERT(descriptors.rows == (int)_words.size());
        _wordsDescriptors = descriptors.clone();
    }
}

cv::Mat Signature::getPoseCovariance() const
{
    cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
    return covariance;
}

} //namespace rtabmap
