/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_AUDIO_PROCESSING_NS_NS_CORE_H_
#define WEBRTC_MODULES_AUDIO_PROCESSING_NS_NS_CORE_H_

#include "webrtc/modules/audio_processing/ns/defines.h"

typedef struct NSParaExtract_
{
    // Bin size of histogram.
    float binSizeLrt;
    float binSizeSpecFlat;
    float binSizeSpecDiff;
    // Range of histogram over which LRT threshold is computed.
    float rangeAvgHistLrt;
    // Scale parameters: multiply dominant peaks of the histograms by scale factor
    // to obtain thresholds for prior model.
    float factor1ModelPars;  // For LRT and spectral difference.
    float factor2ModelPars;  // For spectral_flatness: used when noise is flatter
    // than speech.
    // Peak limit for spectral flatness (varies between 0 and 1).
    float thresPosSpecFlat;
    // Limit on spacing of two highest peaks in histogram: spacing determined by
    // bin size.
    float limitPeakSpacingSpecFlat;
    float limitPeakSpacingSpecDiff;
    // Limit on relevance of second peak.
    float limitPeakWeightsSpecFlat;
    float limitPeakWeightsSpecDiff;
    // Limit on fluctuation of LRT feature.
    float thresFluctLrt;
    // Limit on the max and min values for the feature thresholds.
    float maxLrt;
    float minLrt;
    float maxSpecFlat;
    float minSpecFlat;
    float maxSpecDiff;
    float minSpecDiff;
    // Criteria of weight of histogram peak to accept/reject feature.
    int thresWeightSpecFlat;
    int thresWeightSpecDiff;

} NSParaExtract;

typedef struct NoiseSuppressionC_
{
    uint32_t fs;       //采样率
    size_t blockLen;	//10ms对应处理的数据长度: 80 or 160
    size_t windShift;	//窗口长度
    size_t anaLen;		//FFT分析长度
    size_t magnLen;		//由于fft是对称的，所以只处理前一半长度的数据 i.e. magnLen = anaLen / 2
    int aggrMode;		//降噪激进度
    const float *window;	//窗口函数系数
    float analyzeBuf[ANAL_BLOCKL_MAX];	//分析数据的数组
    float dataBuf[ANAL_BLOCKL_MAX];		//数据缓存数组
    float syntBuf[ANAL_BLOCKL_MAX];		//谱减法缓存数组

    int initFlag;		//开始标记位
    // Parameters for quantile noise estimation.
    float density[SIMULT * HALF_ANAL_BLOCKL];　//概率密度数组
    float lquantile[SIMULT * HALF_ANAL_BLOCKL];	//对数分位数数组
    float quantile[HALF_ANAL_BLOCKL];	//分位数数组(临时保存噪声的数组)
    int counter[SIMULT];	//记数
    int updates;	//更新记数
    // Parameters for Wiener filter.
    float smooth[HALF_ANAL_BLOCKL];　//
光滑系统
    float overdrive;	//降噪级别
    float denoiseBound;	//降噪分赃
    int gainmap;	//增举益
    // FFT work arrays.
    size_t ip[IP_LENGTH];　//实部
    float wfft[W_LENGTH];

    // Parameters for new method: some not needed, will reduce/cleanup later.
    int32_t blockInd;  // Frame index counter帧索引计数器 .
    int modelUpdatePars[4];  // Parameters for updating or estimating更新或估计参数 .
    // Thresholds/weights for prior model.
    float priorModelPars[7];  // Parameters for prior model先验模型参数.
    float noise[HALF_ANAL_BLOCKL];  // Noise spectrum from current frame当前帧噪声谱 .
    float noisePrev[HALF_ANAL_BLOCKL];  // Noise spectrum from previous frame前一帧噪声谱.
    // Magnitude spectrum of previous analyze frame前一帧分析的幅度谱.
    float magnPrevAnalyze[HALF_ANAL_BLOCKL];
    // Magnitude spectrum of previous process frame前处理帧的幅度谱.
    float magnPrevProcess[HALF_ANAL_BLOCKL];
    float logLrtTimeAvg[HALF_ANAL_BLOCKL];  // Log LRT factor with time-smoothing对数似然比时间平滑因子.
    float priorSpeechProb;  // Prior speech/noise probability先验语音/噪声概率.
    float featureData[7];
    // Conservative noise spectrum estimate保守噪声谱估计.
    float magnAvgPause[HALF_ANAL_BLOCKL];
    float signalEnergy;  // Energy of |magn|.
    float sumMagn;
    float whiteNoiseLevel;  // Initial noise estimate初始噪声估计.
    float initMagnEst[HALF_ANAL_BLOCKL];  // Initial magnitude spectrum estimate谱估计的初始大小.
    float pinkNoiseNumerator;  // Pink noise parameter: numerator.
    float pinkNoiseExp;  // Pink noise parameter: power of frequencies粉红噪声参数：频率功率.
    float parametricNoise[HALF_ANAL_BLOCKL];
    // Parameters for feature extraction特征提取参数.
    NSParaExtract featureExtractionParams;
    // Histograms for parameter estimation参数估计直方图.
    int histLrt[HIST_PAR_EST];
    int histSpecFlat[HIST_PAR_EST];
    int histSpecDiff[HIST_PAR_EST];
    // Quantities for high band estimate高带估计量.
    float speechProb[HALF_ANAL_BLOCKL];  // Final speech/noise prob: prior + LRT.
    // Buffering data for HB HB缓冲数据.
    float dataBufHB[NUM_HIGH_BANDS_MAX][ANAL_BLOCKL_MAX];

} NoiseSuppressionC;

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * WebRtcNs_InitCore(...)
 *
 * This function initializes a noise suppression instance
 *
 * Input:
 *      - self          : Instance that should be initialized
 *      - fs            : Sampling frequency
 *
 * Output:
 *      - self          : Initialized instance
 *
 * Return value         :  0 - Ok
 *                        -1 - Error
 */
int WebRtcNs_InitCore ( NoiseSuppressionC *self, uint32_t fs );

/****************************************************************************
 * WebRtcNs_set_policy_core(...)
 *
 * This changes the aggressiveness of the noise suppression method.
 *
 * Input:
 *      - self          : Instance that should be initialized
 *      - mode          : 0: Mild (6dB), 1: Medium (10dB), 2: Aggressive (15dB)
 *
 * Output:
 *      - self          : Initialized instance
 *
 * Return value         :  0 - Ok
 *                        -1 - Error
 */
int WebRtcNs_set_policy_core ( NoiseSuppressionC *self, int mode );

/****************************************************************************
 * WebRtcNs_AnalyzeCore
 *
 * Estimate the background noise.
 *
 * Input:
 *      - self          : Instance that should be initialized
 *      - speechFrame   : Input speech frame for lower band
 *
 * Output:
 *      - self          : Updated instance
 */
void WebRtcNs_AnalyzeCore ( NoiseSuppressionC *self, const float *speechFrame );

/****************************************************************************
 * WebRtcNs_ProcessCore
 *
 * Do noise suppression.
 *
 * Input:
 *      - self          : Instance that should be initialized
 *      - inFrame       : Input speech frame for each band
 *      - num_bands     : Number of bands
 *
 * Output:
 *      - self          : Updated instance
 *      - outFrame      : Output speech frame for each band
 */
void WebRtcNs_ProcessCore ( NoiseSuppressionC *self,
                            const float *const *inFrame,
                            size_t num_bands,
                            float *const *outFrame );

#ifdef __cplusplus
}
#endif
#endif  // WEBRTC_MODULES_AUDIO_PROCESSING_NS_NS_CORE_H_
