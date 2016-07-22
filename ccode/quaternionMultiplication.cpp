#define S_FUNCTION_NAME quaternionMultiplication /* Defines and Includes */
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
    int_T i;
    InputRealPtrsType quat1 = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType quat2 = ssGetInputPortRealSignalPtrs(S,0);
    real_T *y = ssGetOutputPortRealSignal(S,0);
    
     
    
    double roll  = *uPtrs[0];
    double pitch = *uPtrs[1];
    double yaw   = *uPtrs[2];
    
    double cosPhi_2   = cos(roll / 2.0);
    double sinPhi_2   = sin(roll / 2.0);
    double cosTheta_2 = cos(pitch / 2.0);
    double sinTheta_2 = sin(pitch / 2.0);
    double cosPsi_2   = cos(yaw / 2.0);
    double sinPsi_2   = sin(yaw / 2.0);
    
    y[0] = data[0] * q.data[0] - data[1] * q.data[1] - data[2] * q.data[2] - data[3] * q.data[3];
    y[1] = data[0] * q.data[1] + data[1] * q.data[0] + data[2] * q.data[3] - data[3] * q.data[2];
    y[2] = data[0] * q.data[2] - data[1] * q.data[3] + data[2] * q.data[0] + data[3] * q.data[1];
    y[3] = data[0] * q.data[3] + data[1] * q.data[2] - data[2] * q.data[1] + data[3] * q.data[0];
}
static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif