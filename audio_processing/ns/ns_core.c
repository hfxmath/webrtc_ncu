/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "webrtc/base/checks.h"
#include "webrtc/common_audio/fft4g.h"
#include "webrtc/common_audio/signal_processing/include/signal_processing_library.h"
#include "webrtc/modules/audio_processing/ns/noise_suppression.h"
#include "webrtc/modules/audio_processing/ns/ns_core.h"
#include "webrtc/modules/audio_processing/ns/windows_private.h"

// Set Feature Extraction Parameters.
static void set_feature_extraction_parameters ( NoiseSuppressionC *self )
{
    // Bin size of histogram.
    self->featureExtractionParams.binSizeLrt = 0.1f;
    self->featureExtractionParams.binSizeSpecFlat = 0.05f;
    self->featureExtractionParams.binSizeSpecDiff = 0.1f;
    // Range of histogram over which LRT threshold is computed.
    self->featureExtractionParams.rangeAvgHistLrt = 1.f;
    // Scale parameters: multiply dominant peaks of the histograms by scale factor
    // to obtain thresholds for prior model.
    // For LRT and spectral difference.
    self->featureExtractionParams.factor1ModelPars = 1.2f;
    // For spectral_flatness: used when noise is flatter than speech.
    self->featureExtractionParams.factor2ModelPars = 0.9f;
    // Peak limit for spectral flatness (varies between 0 and 1).
    self->featureExtractionParams.thresPosSpecFlat = 0.6f;
    // Limit on spacing of two highest peaks in histogram: spacing determined by
    // bin size.
    self->featureExtractionParams.limitPeakSpacingSpecFlat =
        2 * self->featureExtractionParams.binSizeSpecFlat;
    self->featureExtractionParams.limitPeakSpacingSpecDiff =
        2 * self->featureExtractionParams.binSizeSpecDiff;
    // Limit on relevance of second peak.
    self->featureExtractionParams.limitPeakWeightsSpecFlat = 0.5f;
    self->featureExtractionParams.limitPeakWeightsSpecDiff = 0.5f;
    // Fluctuation limit of LRT feature.
    self->featureExtractionParams.thresFluctLrt = 0.05f;
    // Limit on the max and min values for the feature thresholds.
    self->featureExtractionParams.maxLrt = 1.f;
    self->featureExtractionParams.minLrt = 0.2f;
    self->featureExtractionParams.maxSpecFlat = 0.95f;
    self->featureExtractionParams.minSpecFlat = 0.1f;
    self->featureExtractionParams.maxSpecDiff = 1.f;
    self->featureExtractionParams.minSpecDiff = 0.16f;
    // Criteria of weight of histogram peak to accept/reject feature.
    self->featureExtractionParams.thresWeightSpecFlat =
        ( int ) ( 0.3 * ( self->modelUpdatePars[1] ) ); // For spectral flatness.
    self->featureExtractionParams.thresWeightSpecDiff =
        ( int ) ( 0.3 * ( self->modelUpdatePars[1] ) ); // For spectral difference.
}

// Initialize state.
int WebRtcNs_InitCore ( NoiseSuppressionC *self, uint32_t fs )
{
    int i;

    // Check for valid pointer.
    if ( self == NULL )
    {
        return -1;
    }

    // Initialization of struct.
    if ( fs == 8000 || fs == 16000 || fs == 32000 || fs == 48000 )
    {
        self->fs = fs;
    }

    else
    {
        return -1;
    }

    self->windShift = 0;

    // We only support 10ms frames.
    if ( fs == 8000 )
    {
        self->blockLen = 80;
        self->anaLen = 128;
        self->window = kBlocks80w128;
    }

    else
    {
        self->blockLen = 160;
        self->anaLen = 256;
        self->window = kBlocks160w256;
    }

    self->magnLen = self->anaLen / 2 + 1;  // Number of frequency bins.FFT变换是对称的
    // Initialize FFT work arrays.
    self->ip[0] = 0;  // Setting this triggers initialization.
    memset ( self->dataBuf, 0, sizeof ( float ) * ANAL_BLOCKL_MAX );
    WebRtc_rdft ( self->anaLen, 1, self->dataBuf, self->ip, self->wfft );
    memset ( self->analyzeBuf, 0, sizeof ( float ) * ANAL_BLOCKL_MAX );
    memset ( self->dataBuf, 0, sizeof ( float ) * ANAL_BLOCKL_MAX );
    memset ( self->syntBuf, 0, sizeof ( float ) * ANAL_BLOCKL_MAX );
    // For HB processing.
    memset ( self->dataBufHB,
             0,
             sizeof ( float ) * NUM_HIGH_BANDS_MAX * ANAL_BLOCKL_MAX );
    // For quantile noise estimation.
    memset ( self->quantile, 0, sizeof ( float ) * HALF_ANAL_BLOCKL );

    for ( i = 0; i < SIMULT * HALF_ANAL_BLOCKL; i++ )
    {
        //注意，这里的lquantile是以对数保存，原因是幅度值太大，
        //采用log保存数据，则数据很小，这里是8,对应的能量值为:2980左右(math.exp(8))
        self->lquantile[i] = 8.f;
        self->density[i] = 0.3f;
    }

    for ( i = 0; i < SIMULT; i++ )
    {
        self->counter[i] =
            ( int ) floor ( ( float ) ( END_STARTUP_LONG * ( i + 1 ) ) / ( float ) SIMULT );
    }

    self->updates = 0;

    // Wiener filter initialization.
    for ( i = 0; i < HALF_ANAL_BLOCKL; i++ )
    {
        self->smooth[i] = 1.f;
    }

    // Set the aggressiveness: default.
    self->aggrMode = 0;
    // Initialize variables for new method.
    self->priorSpeechProb = 0.5f;  // Prior prob for speech/noise.
    // Previous analyze mag spectrum.
    memset ( self->magnPrevAnalyze, 0, sizeof ( float ) * HALF_ANAL_BLOCKL );
    // Previous process mag spectrum.
    memset ( self->magnPrevProcess, 0, sizeof ( float ) * HALF_ANAL_BLOCKL );
    // Current noise-spectrum.
    memset ( self->noise, 0, sizeof ( float ) * HALF_ANAL_BLOCKL );
    // Previous noise-spectrum.
    memset ( self->noisePrev, 0, sizeof ( float ) * HALF_ANAL_BLOCKL );
    // Conservative noise spectrum estimate.
    memset ( self->magnAvgPause, 0, sizeof ( float ) * HALF_ANAL_BLOCKL );
    // For estimation of HB in second pass.
    memset ( self->speechProb, 0, sizeof ( float ) * HALF_ANAL_BLOCKL );
    // Initial average magnitude spectrum.
    memset ( self->initMagnEst, 0, sizeof ( float ) * HALF_ANAL_BLOCKL );

    for ( i = 0; i < HALF_ANAL_BLOCKL; i++ )
    {
        // Smooth LR (same as threshold).
        self->logLrtTimeAvg[i] = LRT_FEATURE_THR;
    }

    // Feature quantities.
    // Spectral flatness (start on threshold).
    self->featureData[0] = SF_FEATURE_THR;
    self->featureData[1] = 0.f;  // Spectral entropy: not used in this version.
    self->featureData[2] = 0.f;  // Spectral variance: not used in this version.
    // Average LRT factor (start on threshold).
    self->featureData[3] = LRT_FEATURE_THR;
    // Spectral template diff (start on threshold).
    self->featureData[4] = SF_FEATURE_THR;
    self->featureData[5] = 0.f;  // Normalization for spectral difference.
    // Window time-average of input magnitude spectrum.
    self->featureData[6] = 0.f;
    // Histogram quantities: used to estimate/update thresholds for features.
    memset ( self->histLrt, 0, sizeof ( int ) * HIST_PAR_EST );
    memset ( self->histSpecFlat, 0, sizeof ( int ) * HIST_PAR_EST );
    memset ( self->histSpecDiff, 0, sizeof ( int ) * HIST_PAR_EST );
    self->blockInd = -1;  // Frame counter.
    // Default threshold for LRT feature.
    self->priorModelPars[0] = LRT_FEATURE_THR;
    // Threshold for spectral flatness: determined on-line.
    self->priorModelPars[1] = 0.5f;
    // sgn_map par for spectral measure: 1 for flatness measure.
    self->priorModelPars[2] = 1.f;
    // Threshold for template-difference feature: determined on-line.
    self->priorModelPars[3] = 0.5f;
    // Default weighting parameter for LRT feature.
    self->priorModelPars[4] = 1.f;
    // Default weighting parameter for spectral flatness feature.
    self->priorModelPars[5] = 0.f;
    // Default weighting parameter for spectral difference feature.
    self->priorModelPars[6] = 0.f;
    // Update flag for parameters:
    // 0 no update, 1 = update once, 2 = update every window.
    self->modelUpdatePars[0] = 2;
    self->modelUpdatePars[1] = 500;  // Window for update.
    // Counter for update of conservative noise spectrum.
    self->modelUpdatePars[2] = 0;
    // Counter if the feature thresholds are updated during the sequence.
    self->modelUpdatePars[3] = self->modelUpdatePars[1];
    self->signalEnergy = 0.0;
    self->sumMagn = 0.0;
    self->whiteNoiseLevel = 0.0;
    self->pinkNoiseNumerator = 0.0;
    self->pinkNoiseExp = 0.0;
    set_feature_extraction_parameters ( self );
    // Default mode.
    WebRtcNs_set_policy_core ( self, 0 );
    self->initFlag = 1;
    return 0;
}

// Estimate noise.
static void NoiseEstimation ( NoiseSuppressionC *self,
                              float *magn,
                              float *noise )
{
    size_t i, s, offset;
    float lmagn[HALF_ANAL_BLOCKL], delta;

    if ( self->updates < END_STARTUP_LONG )
    {
        self->updates++;
    }

    for ( i = 0; i < self->magnLen; i++ )
    {
        lmagn[i] = ( float ) log ( magn[i] ); //log 是以 e 为底的对数函数　
    }

    // Loop over simultaneous estimates.
    for ( s = 0; s < SIMULT; s++ )
    {
        offset = s * self->magnLen;  //0,129, 258

        // newquantest(...)
        for ( i = 0; i < self->magnLen; i++ )
        {
            // Compute delta.
            if ( self->density[offset + i] > 1.0 )
            {
                delta = FACTOR * 1.f / self->density[offset + i];
            }

            else
            {
                delta = FACTOR;
            }

            // Update log quantile estimate.
            //>= 3000(能量值)
            if ( lmagn[i] > self->lquantile[offset + i] )
            {
                self->lquantile[offset + i] +=  QUANTILE * delta / ( float ) ( self->counter[s] + 1 );
            }

            else
            {
                self->lquantile[offset + i] -=
                    ( 1.f - QUANTILE ) * delta / ( float ) ( self->counter[s] + 1 );
            }

            // Update density estimate.
            if ( fabs ( lmagn[i] - self->lquantile[offset + i] ) < WIDTH )
            {
                self->density[offset + i] =
                    ( ( float ) self->counter[s] * self->density[offset + i] +
                      1.f / ( 2.f * WIDTH ) ) / ( float ) ( self->counter[s] + 1 );
            }
        }  // End loop over magnitude spectrum.

        if ( self->counter[s] >= END_STARTUP_LONG )
        {
            self->counter[s] = 0;

            if ( self->updates >= END_STARTUP_LONG )
            {
                for ( i = 0; i < self->magnLen; i++ )
                {
                    self->quantile[i] = ( float ) exp ( self->lquantile[offset + i] );
                }
            }
        }

        self->counter[s]++;
    }  // End loop over simultaneous estimates.

    // Sequentially update the noise during startup.
    if ( self->updates < END_STARTUP_LONG )
    {
        // Use the last "s" to get noise during startup that differ from zero.
        for ( i = 0; i < self->magnLen; i++ )
        {
            self->quantile[i] = ( float ) exp ( self->lquantile[offset + i] );
        }
    }

    for ( i = 0; i < self->magnLen; i++ )
    {
        noise[i] = self->quantile[i];
    }
}

// Extract thresholds for feature parameters.
// Histograms are computed over some window size (given by
// self->modelUpdatePars[1]).
// Thresholds and weights are extracted every window.
// |flag| = 0 updates histogram only, |flag| = 1 computes the threshold/weights.
// Threshold and weights are returned in: self->priorModelPars.
static void FeatureParameterExtraction ( NoiseSuppressionC *self, int flag )
{
    int i, useFeatureSpecFlat, useFeatureSpecDiff, numHistLrt;
    int maxPeak1, maxPeak2;
    int weightPeak1SpecFlat, weightPeak2SpecFlat, weightPeak1SpecDiff,
        weightPeak2SpecDiff;
    float binMid, featureSum;
    float posPeak1SpecFlat, posPeak2SpecFlat, posPeak1SpecDiff, posPeak2SpecDiff;
    float fluctLrt, avgHistLrt, avgSquareHistLrt, avgHistLrtCompl;

    // 3 features: LRT, flatness, difference.
    // lrt_feature = self->featureData[3];  = .5  //似然比
    // flat_feature = self->featureData[0]; = .5  //平坦性
    // diff_feature = self->featureData[4]; = .5  //差异性

    // Update histograms.
    if ( flag == 0 )
    {
        // LRT
        if ( ( self->featureData[3] <
                HIST_PAR_EST * self->featureExtractionParams.binSizeLrt ) &&
                ( self->featureData[3] >= 0.0 ) )
        {
            i = ( int ) ( self->featureData[3] / self->featureExtractionParams.binSizeLrt );
            self->histLrt[i]++;
        }

        // Spectral flatness.
        if ( ( self->featureData[0] <
                HIST_PAR_EST * self->featureExtractionParams.binSizeSpecFlat ) &&
                ( self->featureData[0] >= 0.0 ) )
        {
            i = ( int ) ( self->featureData[0] / self->featureExtractionParams.binSizeSpecFlat );
            self->histSpecFlat[i]++;
        }

        // Spectral difference.
        if ( ( self->featureData[4] <
                HIST_PAR_EST * self->featureExtractionParams.binSizeSpecDiff ) &&
                ( self->featureData[4] >= 0.0 ) )
        {
            i = ( int ) ( self->featureData[4] /
                          self->featureExtractionParams.binSizeSpecDiff );
            self->histSpecDiff[i]++;
        }
    }

    // Extract parameters for speech/noise probability.
    if ( flag == 1 )
    {
        // LRT feature: compute the average over
        // self->featureExtractionParams.rangeAvgHistLrt.
        avgHistLrt = 0.0;
        avgHistLrtCompl = 0.0;
        avgSquareHistLrt = 0.0;
        numHistLrt = 0;

        for ( i = 0; i < HIST_PAR_EST; i++ )
        {
            binMid = ( ( float ) i + 0.5f ) * self->featureExtractionParams.binSizeLrt;

            if ( binMid <= self->featureExtractionParams.rangeAvgHistLrt )
            {
                avgHistLrt += self->histLrt[i] * binMid;
                numHistLrt += self->histLrt[i];
            }

            avgSquareHistLrt += self->histLrt[i] * binMid * binMid;
            avgHistLrtCompl += self->histLrt[i] * binMid;
        }

        if ( numHistLrt > 0 )
        {
            avgHistLrt = avgHistLrt / ( ( float ) numHistLrt );
        }

        avgHistLrtCompl = avgHistLrtCompl / ( ( float ) self->modelUpdatePars[1] );
        avgSquareHistLrt = avgSquareHistLrt / ( ( float ) self->modelUpdatePars[1] );
        fluctLrt = avgSquareHistLrt - avgHistLrt * avgHistLrtCompl;

        // Get threshold for LRT feature.
        if ( fluctLrt < self->featureExtractionParams.thresFluctLrt )
        {
            // Very low fluctuation, so likely noise.
            self->priorModelPars[0] = self->featureExtractionParams.maxLrt;
        }

        else
        {
            self->priorModelPars[0] =
                self->featureExtractionParams.factor1ModelPars * avgHistLrt;

            // Check if value is within min/max range.
            if ( self->priorModelPars[0] < self->featureExtractionParams.minLrt )
            {
                self->priorModelPars[0] = self->featureExtractionParams.minLrt;
            }

            if ( self->priorModelPars[0] > self->featureExtractionParams.maxLrt )
            {
                self->priorModelPars[0] = self->featureExtractionParams.maxLrt;
            }
        }

        // Done with LRT feature.
        // For spectral flatness and spectral difference: compute the main peaks of
        // histogram.
        maxPeak1 = 0;
        maxPeak2 = 0;
        posPeak1SpecFlat = 0.0;
        posPeak2SpecFlat = 0.0;
        weightPeak1SpecFlat = 0;
        weightPeak2SpecFlat = 0;

        // Peaks for flatness.
        for ( i = 0; i < HIST_PAR_EST; i++ )
        {
            binMid =
                ( i + 0.5f ) * self->featureExtractionParams.binSizeSpecFlat;

            if ( self->histSpecFlat[i] > maxPeak1 )
            {
                // Found new "first" peak.
                maxPeak2 = maxPeak1;
                weightPeak2SpecFlat = weightPeak1SpecFlat;
                posPeak2SpecFlat = posPeak1SpecFlat;
                maxPeak1 = self->histSpecFlat[i];
                weightPeak1SpecFlat = self->histSpecFlat[i];
                posPeak1SpecFlat = binMid;
            }

            else if ( self->histSpecFlat[i] > maxPeak2 )
            {
                // Found new "second" peak.
                maxPeak2 = self->histSpecFlat[i];
                weightPeak2SpecFlat = self->histSpecFlat[i];
                posPeak2SpecFlat = binMid;
            }
        }

        // Compute two peaks for spectral difference.
        maxPeak1 = 0;
        maxPeak2 = 0;
        posPeak1SpecDiff = 0.0;
        posPeak2SpecDiff = 0.0;
        weightPeak1SpecDiff = 0;
        weightPeak2SpecDiff = 0;

        // Peaks for spectral difference.
        for ( i = 0; i < HIST_PAR_EST; i++ )
        {
            binMid =
                ( ( float ) i + 0.5f ) * self->featureExtractionParams.binSizeSpecDiff;

            if ( self->histSpecDiff[i] > maxPeak1 )
            {
                // Found new "first" peak.
                maxPeak2 = maxPeak1;
                weightPeak2SpecDiff = weightPeak1SpecDiff;
                posPeak2SpecDiff = posPeak1SpecDiff;
                maxPeak1 = self->histSpecDiff[i];
                weightPeak1SpecDiff = self->histSpecDiff[i];
                posPeak1SpecDiff = binMid;
            }

            else if ( self->histSpecDiff[i] > maxPeak2 )
            {
                // Found new "second" peak.
                maxPeak2 = self->histSpecDiff[i];
                weightPeak2SpecDiff = self->histSpecDiff[i];
                posPeak2SpecDiff = binMid;
            }
        }

        // For spectrum flatness feature.
        useFeatureSpecFlat = 1;

        // Merge the two peaks if they are close.
        if ( ( fabs ( posPeak2SpecFlat - posPeak1SpecFlat ) <
                self->featureExtractionParams.limitPeakSpacingSpecFlat ) &&
                ( weightPeak2SpecFlat >
                  self->featureExtractionParams.limitPeakWeightsSpecFlat *
                  weightPeak1SpecFlat ) )
        {
            weightPeak1SpecFlat += weightPeak2SpecFlat;
            posPeak1SpecFlat = 0.5f * ( posPeak1SpecFlat + posPeak2SpecFlat );
        }

        // Reject if weight of peaks is not large enough, or peak value too small.
        if ( weightPeak1SpecFlat <
                self->featureExtractionParams.thresWeightSpecFlat ||
                posPeak1SpecFlat < self->featureExtractionParams.thresPosSpecFlat )
        {
            useFeatureSpecFlat = 0;
        }

        // If selected, get the threshold.
        if ( useFeatureSpecFlat == 1 )
        {
            // Compute the threshold.
            self->priorModelPars[1] =
                self->featureExtractionParams.factor2ModelPars * posPeak1SpecFlat;

            // Check if value is within min/max range.
            if ( self->priorModelPars[1] < self->featureExtractionParams.minSpecFlat )
            {
                self->priorModelPars[1] = self->featureExtractionParams.minSpecFlat;
            }

            if ( self->priorModelPars[1] > self->featureExtractionParams.maxSpecFlat )
            {
                self->priorModelPars[1] = self->featureExtractionParams.maxSpecFlat;
            }
        }

        // Done with flatness feature.
        // For template feature.
        useFeatureSpecDiff = 1;

        // Merge the two peaks if they are close.
        if ( ( fabs ( posPeak2SpecDiff - posPeak1SpecDiff ) <
                self->featureExtractionParams.limitPeakSpacingSpecDiff ) &&
                ( weightPeak2SpecDiff >
                  self->featureExtractionParams.limitPeakWeightsSpecDiff *
                  weightPeak1SpecDiff ) )
        {
            weightPeak1SpecDiff += weightPeak2SpecDiff;
            posPeak1SpecDiff = 0.5f * ( posPeak1SpecDiff + posPeak2SpecDiff );
        }

        // Get the threshold value.
        self->priorModelPars[3] =
            self->featureExtractionParams.factor1ModelPars * posPeak1SpecDiff;

        // Reject if weight of peaks is not large enough.
        if ( weightPeak1SpecDiff <
                self->featureExtractionParams.thresWeightSpecDiff )
        {
            useFeatureSpecDiff = 0;
        }

        // Check if value is within min/max range.
        if ( self->priorModelPars[3] < self->featureExtractionParams.minSpecDiff )
        {
            self->priorModelPars[3] = self->featureExtractionParams.minSpecDiff;
        }

        if ( self->priorModelPars[3] > self->featureExtractionParams.maxSpecDiff )
        {
            self->priorModelPars[3] = self->featureExtractionParams.maxSpecDiff;
        }

        // Done with spectral difference feature.

        // Don't use template feature if fluctuation of LRT feature is very low:
        // most likely just noise state.
        if ( fluctLrt < self->featureExtractionParams.thresFluctLrt )
        {
            useFeatureSpecDiff = 0;
        }

        // Select the weights between the features.
        // self->priorModelPars[4] is weight for LRT: always selected.
        // self->priorModelPars[5] is weight for spectral flatness.
        // self->priorModelPars[6] is weight for spectral difference.
        featureSum = ( float ) ( 1 + useFeatureSpecFlat + useFeatureSpecDiff );
        self->priorModelPars[4] = 1.f / featureSum;
        self->priorModelPars[5] = ( ( float ) useFeatureSpecFlat ) / featureSum;
        self->priorModelPars[6] = ( ( float ) useFeatureSpecDiff ) / featureSum;

        // Set hists to zero for next update.
        if ( self->modelUpdatePars[0] >= 1 )
        {
            for ( i = 0; i < HIST_PAR_EST; i++ )
            {
                self->histLrt[i] = 0;
                self->histSpecFlat[i] = 0;
                self->histSpecDiff[i] = 0;
            }
        }
    }  // End of flag == 1.
}

// Compute spectral flatness on input spectrum.
// |magnIn| is the magnitude spectrum.
// Spectral flatness is returned in self->featureData[0].
static void ComputeSpectralFlatness ( NoiseSuppressionC *self,
                                      const float *magnIn )
{
    size_t i;
    size_t shiftLP = 1;  // Option to remove first bin(s) from spectral measures.
    float avgSpectralFlatnessNum, avgSpectralFlatnessDen, spectralTmp;
    // Compute spectral measures.
    // For flatness.
    avgSpectralFlatnessNum = 0.0;
    avgSpectralFlatnessDen = self->sumMagn;

    for ( i = 0; i < shiftLP; i++ )
    {
        avgSpectralFlatnessDen -= magnIn[i];
    }

    //几何平均值与算术平均数之比
    // Compute log of ratio of the geometric to arithmetic mean:
    //check for log(0) case.
    for ( i = shiftLP; i < self->magnLen; i++ )
    {
        if ( magnIn[i] > 0.0 )
        {
            avgSpectralFlatnessNum += ( float ) log ( magnIn[i] );
        }

        else
        {
            self->featureData[0] -= SPECT_FL_TAVG * self->featureData[0];
            return;
        }
    }

    // Normalize.
    avgSpectralFlatnessDen = avgSpectralFlatnessDen / self->magnLen;
    avgSpectralFlatnessNum = avgSpectralFlatnessNum / self->magnLen;
    // Ratio and inverse log: check for case of log(0).
    spectralTmp = ( float ) exp ( avgSpectralFlatnessNum ) / avgSpectralFlatnessDen;
    // Time-avg update of spectral flatness feature.
    self->featureData[0] += SPECT_FL_TAVG * ( spectralTmp - self->featureData[0] );
    // Done with flatness feature.
}

// Compute prior and post SNR based on quantile noise estimation.
// Compute DD estimate of prior SNR.
// Inputs:
//   * |magn| is the signal magnitude spectrum estimate.
//   * |noise| is the magnitude noise spectrum estimate.
// Outputs:
//   * |snrLocPrior| is the computed prior SNR.
//   * |snrLocPost| is the computed post SNR.
static void ComputeSnr ( const NoiseSuppressionC *self,
                         const float *magn,
                         const float *noise,
                         float *snrLocPrior,
                         float *snrLocPost )
{
    size_t i;

    for ( i = 0; i < self->magnLen; i++ )
    {
        // Previous post SNR.
        // Previous estimate: based on previous frame with gain filter.
        //前一帧先验SNR估计,加了平滑
        float previousEstimateStsa = self->magnPrevAnalyze[i] /  ( self->noisePrev[i] + 0.0001f ) * self->smooth[i];
        // Post SNR.
        snrLocPost[i] = 0.f;

        // DD estimate is sum of two terms: current estimate and previous estimate.
        //Directed decision update of snrPrior.
        //magn:包括语音与噪声；noise:噪声
        if ( magn[i] > noise[i] )
        {
            snrLocPost[i] = magn[i] / ( noise[i] + 0.0001f ) - 1.f;
        }

        //当前先验SNR估计 = ｒ*前一帧先验SNR估计　+ (1-ｒ)*当前后验SNR估计  r = 0.98
        snrLocPrior[i] = DD_PR_SNR * previousEstimateStsa + ( 1.f - DD_PR_SNR ) * snrLocPost[i];
    }  // End of loop over frequencies.
}

// Compute the difference measure between input spectrum and a template/learned
// noise spectrum.
// |magnIn| is the input spectrum.
// The reference/template spectrum is self->magnAvgPause[i].
// Returns (normalized) spectral difference in self->featureData[4].
static void ComputeSpectralDifference ( NoiseSuppressionC *self,
                                        const float *magnIn )
{
    // avgDiffNormMagn = var(magnIn) - cov(magnIn, magnAvgPause)^2 /
    // var(magnAvgPause)
    size_t i;
    float avgPause, avgMagn, covMagnPause, varPause, varMagn, avgDiffNormMagn;
    avgPause = 0.0;
    avgMagn = self->sumMagn;

    // Compute average quantities.
    for ( i = 0; i < self->magnLen; i++ )
    {
        // Conservative smooth noise spectrum from pause frames.
        avgPause += self->magnAvgPause[i];
    }

    avgPause /= self->magnLen;
    avgMagn /= self->magnLen;
    covMagnPause = 0.0;
    varPause = 0.0;
    varMagn = 0.0;

    // Compute variance and covariance quantities.
    for ( i = 0; i < self->magnLen; i++ )
    {
        covMagnPause += ( magnIn[i] - avgMagn ) * ( self->magnAvgPause[i] - avgPause );
        varPause += ( self->magnAvgPause[i] - avgPause ) * ( self->magnAvgPause[i] - avgPause );
        varMagn += ( magnIn[i] - avgMagn ) * ( magnIn[i] - avgMagn );
    }

    covMagnPause /= self->magnLen;
    varPause /= self->magnLen;
    varMagn /= self->magnLen;
    // Update of average magnitude spectrum.
    self->featureData[6] += self->signalEnergy;
    avgDiffNormMagn = varMagn - ( covMagnPause * covMagnPause ) / ( varPause + 0.0001f );
    // Normalize and compute time-avg update of difference feature.
    avgDiffNormMagn = ( float ) ( avgDiffNormMagn / ( self->featureData[5] + 0.0001f ) );
    self->featureData[4] += SPECT_DIFF_TAVG * ( avgDiffNormMagn - self->featureData[4] );
}

// Compute speech/noise probability.
// Speech/noise probability is returned in |probSpeechFinal|.
// |magn| is the input magnitude spectrum.
// |noise| is the noise spectrum.
// |snrLocPrior| is the prior SNR for each frequency.
// |snrLocPost| is the post SNR for each frequency.
static void SpeechNoiseProb ( NoiseSuppressionC *self,
                              float *probSpeechFinal,
                              const float *snrLocPrior,
                              const float *snrLocPost )
{
    size_t i;
    int sgnMap;
    float invLrt, gainPrior, indPrior;
    float logLrtTimeAvgKsum, besselTmp;
    float indicator0, indicator1, indicator2;
    float tmpFloat1, tmpFloat2;
    float weightIndPrior0, weightIndPrior1, weightIndPrior2;
    float threshPrior0, threshPrior1, threshPrior2;
    float widthPrior, widthPrior0, widthPrior1, widthPrior2;
    widthPrior0 = WIDTH_PR_MAP;
    // Width for pause region: lower range, so increase width in tanh map.
    widthPrior1 = 2.f * WIDTH_PR_MAP;
    widthPrior2 = 2.f * WIDTH_PR_MAP;  // For spectral-difference measure.
    // Threshold parameters for features.
    threshPrior0 = self->priorModelPars[0];
    threshPrior1 = self->priorModelPars[1];
    threshPrior2 = self->priorModelPars[3];
    // Sign for flatness feature.
    sgnMap = ( int ) ( self->priorModelPars[2] );
    // Weight parameters for features.
    weightIndPrior0 = self->priorModelPars[4];
    weightIndPrior1 = self->priorModelPars[5];
    weightIndPrior2 = self->priorModelPars[6];
    // Compute feature based on average LR factor.
    // This is the average over all frequencies of the smooth log LRT.
    logLrtTimeAvgKsum = 0.0;

    for ( i = 0; i < self->magnLen; i++ )
    {
        tmpFloat1 = 1.f + 2.f * snrLocPrior[i];
        tmpFloat2 = 2.f * snrLocPrior[i] / ( tmpFloat1 + 0.0001f );
        besselTmp = ( snrLocPost[i] + 1.f ) * tmpFloat2;
        self->logLrtTimeAvg[i] += LRT_TAVG * ( besselTmp - ( float ) log ( tmpFloat1 ) - self->logLrtTimeAvg[i] );
        logLrtTimeAvgKsum += self->logLrtTimeAvg[i];
    }

    logLrtTimeAvgKsum = ( float ) logLrtTimeAvgKsum / ( self->magnLen );
    self->featureData[3] = logLrtTimeAvgKsum;
    // Done with computation of LR factor.
    // Compute the indicator functions.
    // Average LRT feature.
    widthPrior = widthPrior0;

    // Use larger width in tanh map for pause regions.
    if ( logLrtTimeAvgKsum < threshPrior0 )
    {
        widthPrior = widthPrior1;
    }

    // Compute indicator function: sigmoid map.
    indicator0 =
        0.5f *
        ( ( float ) tanh ( widthPrior * ( logLrtTimeAvgKsum - threshPrior0 ) ) + 1.f );
    // Spectral flatness feature.
    tmpFloat1 = self->featureData[0];
    widthPrior = widthPrior0;

    // Use larger width in tanh map for pause regions.
    if ( sgnMap == 1 && ( tmpFloat1 > threshPrior1 ) )
    {
        widthPrior = widthPrior1;
    }

    if ( sgnMap == -1 && ( tmpFloat1 < threshPrior1 ) )
    {
        widthPrior = widthPrior1;
    }

    // Compute indicator function: sigmoid map.
    indicator1 =
        0.5f *
        ( ( float ) tanh ( ( float ) sgnMap * widthPrior * ( threshPrior1 - tmpFloat1 ) ) +
          1.f );
    // For template spectrum-difference.
    tmpFloat1 = self->featureData[4];
    widthPrior = widthPrior0;

    // Use larger width in tanh map for pause regions.
    if ( tmpFloat1
