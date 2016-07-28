#define S_FUNCTION_NAME quaternionNormalisation /* Defines and Includes */
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 4);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, 4);

    ssSetNumSampleTimes(S, 1);

    /* Take care when specifying exception free code - see sfuntmpl.doc */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
    }
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}
static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType quat = ssGetInputPortRealSignalPtrs(S,0);
    real_T *y = ssGetOutputPortRealSignal(S,0);
    
    double norm = 0;
    
    for(int i = 0; i < 4; i++)
    {
        norm += (double)(*quat[i]) * (double)(*quat[i]);
    }
    
    norm = sqrt(norm);
    
    if (norm == 0)
    {
        for(int i = 0; i < 4; i++)
        {
            y[i] = 0;
        }
    }
    else
    {
        for(int i = 0; i < 4; i++)
        {
            y[i] = (double)(*quat[i]) / norm;
        }
    }
}
static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif