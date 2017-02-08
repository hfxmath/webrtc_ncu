/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_AUDIO_PROCESSING_VAD_VOICE_ACTIVITY_DETECTOR_H_
#define WEBRTC_MODULES_AUDIO_PROCESSING_VAD_VOICE_ACTIVITY_DETECTOR_H_

#include <vector>

#include "webrtc/base/scoped_ptr.h"
#include "webrtc/common_audio/resampler/include/resampler.h"
#include "webrtc/modules/audio_processing/vad/vad_audio_proc.h"
#include "webrtc/modules/audio_processing/vad/common.h"
#include "webrtc/modules/audio_processing/vad/pitch_based_vad.h"
#include "webrtc/modules/audio_processing/vad/standalone_vad.h"

namespace webrtc
{

    // A Voice Activity Detector (VAD) that combines the voice probability from the
    // StandaloneVad and PitchBasedVad to get a more robust estimation.
    class VoiceActivityDetector
    {
    public:
        VoiceActivityDetector();

        // Processes each audio chunk and estimates the voice probability. The maximum
        // supported sample rate is 32kHz.
        // TODO(aluebs): Change |length| to size_t.
        void ProcessChunk ( const int16_t *audio, int length, int sample_rate_hz );

        // Returns a vector of voice probabilities for each chunk. It can be empty for
        // some chunks, but it catches up afterwards returning multiple values at
        // once.
        const std::vector<double> &chunkwise_voice_probabilities() const
        {
            return chunkwise_voice_probabilities_;
        }

        // Returns a vector of RMS values for each chunk. It has the same length as
        // chunkwise_voice_probabilities().
        const std::vector<double> &chunkwise_rms() const
        {
            return chunkwise_rms_;
        }

        // Returns the last voice probability, regardless of the internal
        // implementation, although it has a few chunks of delay.
        float last_voice_probability() const
        {
            return last_voice_probability_;
        }

    private:
        // TODO(aluebs): Change these to float.
        std::vector<double> chunkwise_voice_probabilities_;
        std::vector<double> chunkwise_rms_;

        float last_voice_probability_;

        Resampler resampler_;
        VadAudioProc audio_processing_;

        rtc::scoped_ptr<StandaloneVad> standalone_vad_;
        PitchBasedVad pitch_based_vad_;

        int16_t resampled_[kLength10Ms];
        AudioFeatures features_;
    };

}  // namespace webrtc

#endif  // WEBRTC_MODULES_AUDIO_PROCESSING_VAD_VOICE_ACTIVITY_DETECTOR_H_