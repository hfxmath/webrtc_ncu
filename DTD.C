int dtdNdx;
/* For Normalized Least Means Square - Pre-whitening */
#define NLMS_LEN (240*8) /* maximum NLMS filter length in taps
    #define DTD_LEN 16 // block size in taps to optimize DTD
float max_x[NLMS_LEN/DTD_LEN];
/* Geigel Double-Talk Detector * 
* in d: microphone sample (PCM as floating point value) 
* in x: loudspeaker sample (PCM as floating point value) 
* return: 0 for no talking, 1 for talking */
int dtd ( float d, float x );
int AEC::dtd ( float d, float x )
{
    // optimized implementation of max(|x[0]|, |x[1]|, .., |x[L-1]|):
    // calculate max of block (DTD_LEN values)
    x = fabsf ( x );

    if ( x > max_x[dtdNdx] )
    {
        max_x[dtdNdx] = x;

        if ( x > max_max_x )
        {
            max_max_x = x;
        }
    }

    if ( ++dtdCnt >= DTD_LEN )
    {
        dtdCnt = 0;
        // calculate max of max
        max_max_x = 0.0f;

        for ( int i = 0; i < NLMS_LEN / DTD_LEN; ++i )
        {
            if ( max_x[i] > max_max_x )
            {
                max_max_x = max_x[i];
            }
        }

        // rotate Ndx
        if ( ++dtdNdx >= NLMS_LEN / DTD_LEN )
        {
            dtdNdx = 0;
        }

        max_x[dtdNdx] = 0.0f;
    }

    // The Geigel DTD algorithm with Hangover timer Thold
    if ( fabsf ( d ) >= GeigelThreshold * max_max_x )
    {
        hangover = Thold;
    }

    if ( hangover )
    {
        --hangover;
    }

    if ( max_max_x < UpdateThreshold )
    {
        // avoid update with silence or noise
        return 1;
    }

    else
    {
        return ( hangover > 0 );
    }
}`
