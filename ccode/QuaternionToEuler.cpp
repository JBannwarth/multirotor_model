#define S_FUNCTION_NAME QuaternionToEuler /* Defines and Includes */
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
    ssSetOutputPortWidth(S, 0, 3);

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
    int_T i;
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    real_T *y = ssGetOutputPortRealSignal(S,0);
    int_T width = ssGetOutputPortWidth(S,0);

    y[0] = atan2(2.0 * (*uPtrs[0] * *uPtrs[1] + *uPtrs[2] * *uPtrs[3]),
            1.0 - 2.0 * (*uPtrs[1] * *uPtrs[1] + *uPtrs[2] * *uPtrs[2]));
	y[1] = asin(2.0 * (*uPtrs[0] * *uPtrs[2] - *uPtrs[3] * *uPtrs[1]));
	y[2] = atan2(2.0 * (*uPtrs[0] * *uPtrs[3] + *uPtrs[1] * *uPtrs[2]),
            1.0 - 2.0 * (*uPtrs[2] * *uPtrs[2] + *uPtrs[3] * *uPtrs[3]));
}
static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif