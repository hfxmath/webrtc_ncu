// WebRtcAudioTest.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Windows.h>
#include "noise_suppression.h"
#include "signal_processing_library.h"

/*
*@src_file:   输入文件
*@out_file:   输出文件
*@sample:     采样率
*@mode:       降噪强度
**/
void NS_48K(char *src_file, char *out_file, int sample, int mode)
{
	NsHandle *p_inst = NULL;
	FILE *fpIn = NULL;
	FILE *fpOut = NULL;

	char *pInBuffer = NULL;
	char *pOutBuffer = NULL;

	int nFileSize = 0;
	int nTime = 0;
	p_inst = WebRtcNs_Create();
	WebRtcNs_Init(p_inst, sample);
	WebRtcNs_set_policy(p_inst, mode);

	fopen_s(&fpIn, src_file, "rb");
	fopen_s(&fpOut, out_file, "wb");

	fseek(fpIn, 0, SEEK_END);
	nFileSize = ftell(fpIn);
	fseek(fpIn, 0, SEEK_SET);
	pInBuffer = (char *)malloc(nFileSize);
	memset(pInBuffer, 0, nFileSize);

	fread(pInBuffer, sizeof(char), nFileSize, fpIn);

	pOutBuffer = (char *)malloc(nFileSize);
	memset(pOutBuffer, 0, nFileSize);


	for (int i = 0; i < nFileSize; i += 640)
	{
		float *shBufferIn = NULL;
		memcpy(shBufferIn, pInBuffer + i, 320);

		float *shBufferIn_ = NULL;
		WebRtcNs_Process(p_inst, reinterpret_cast<const float* const*>(&shBufferIn), 2, reinterpret_cast<float* const*>(&shBufferIn_));
		memcpy(pOutBuffer + i, shBufferIn_, 320);
		
	}
	fwrite(pOutBuffer, sizeof(char), nFileSize, fpOut);

	WebRtcNs_Free(p_inst);
	fclose(fpIn);
	fclose(fpOut);
	free(pInBuffer);
	free(pOutBuffer);

}

int main ()
{
	NS_48K( "test---48k.pcm", "test---48Kout.pcm", 48000, 3);
    printf ( "NS---gameOver........\n" );
    return 0;
}
