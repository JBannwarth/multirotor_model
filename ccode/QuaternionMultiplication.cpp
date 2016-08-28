#define S_FUNCTION_NAME QuaternionMultiplication /* Defines and Includes */
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 4);
    ssSetInputPortWidth(S, 1, 4);
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
    InputRealPtrsType quat1 = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType quat2 = ssGetInputPortRealSignalPtrs(S,0);
    real_T *y = ssGetOutputPortRealSignal(S,0);
    
     double q1_0 = *quat1[0];
     double q1_1 = *quat1[1];
     double q1_2 = *quat1[2];
     double q1_3 = *quat1[3];
     double q2_0 = *quat2[0];
     double q2_1 = *quat2[1];
     double q2_2 = *quat2[2];
     double q2_3 = *quat2[3];
    
    y[0] = q1_0 * q2_0 - q1_1 * q2_1 - q1_2 * q2_2 - q1_3 * q2_3;
    y[1] = q1_0 * q2_1 + q1_1 * q2_0 + q1_2 * q2_3 - q1_3 * q2_2;
    y[2] = q1_0 * q2_2 - q1_1 * q2_3 + q1_2 * q2_0 + q1_3 * q2_1;
    y[3] = q1_0 * q2_3 + q1_1 * q2_2 - q1_2 * q2_1 + q1_3 * q2_0;
}
static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif