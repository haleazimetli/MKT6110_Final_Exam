/*  File    : mav_dynamics.c
 */

#define S_FUNCTION_NAME mav_dynamics
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"

#define mass_PARAM(S)   *mxGetPr(ssGetSFcnParam(S,0))
#define Jx_PARAM(S)     *mxGetPr(ssGetSFcnParam(S,1))
#define Jy_PARAM(S)     *mxGetPr(ssGetSFcnParam(S,2))
#define Jz_PARAM(S)     *mxGetPr(ssGetSFcnParam(S,3))
#define Jxz_PARAM(S)    *mxGetPr(ssGetSFcnParam(S,4))
#define pn0_PARAM(S)    *mxGetPr(ssGetSFcnParam(S,5))
#define pe0_PARAM(S)    *mxGetPr(ssGetSFcnParam(S,6))
#define pd0_PARAM(S)    *mxGetPr(ssGetSFcnParam(S,7))
#define u0_PARAM(S)     *mxGetPr(ssGetSFcnParam(S,8))
#define v0_PARAM(S)     *mxGetPr(ssGetSFcnParam(S,9))
#define w0_PARAM(S)     *mxGetPr(ssGetSFcnParam(S,10))
#define phi0_PARAM(S)   *mxGetPr(ssGetSFcnParam(S,11))
#define theta0_PARAM(S) *mxGetPr(ssGetSFcnParam(S,12))
#define psi0_PARAM(S)   *mxGetPr(ssGetSFcnParam(S,13))
#define p0_PARAM(S)     *mxGetPr(ssGetSFcnParam(S,14))
#define q0_PARAM(S)     *mxGetPr(ssGetSFcnParam(S,15))
#define r0_PARAM(S)     *mxGetPr(ssGetSFcnParam(S,16))

#define Force(element)  (*forcePtrs[element])  /* Pointer to Input Port0 */
#define Torque(element) (*torquePtrs[element]) /* Pointer to Input Port1 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 5+12);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    ssSetNumContStates(S, 12);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetInputPortDirectFeedThrough(S, 1, 0);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 12);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    S-function is comprised of only continuous sample time elements
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}



#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    Initialize continuous states to zero
 */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *x0 = ssGetContStates(S);

    /* set initial conditions */
    x0[0]  = pn0_PARAM(S);
    x0[1]  = pe0_PARAM(S);
    x0[2]  = pd0_PARAM(S);
    x0[3]  = u0_PARAM(S);
    x0[4]  = v0_PARAM(S);
    x0[5]  = w0_PARAM(S);
    x0[6]  = phi0_PARAM(S);
    x0[7]  = theta0_PARAM(S);
    x0[8]  = psi0_PARAM(S);
    x0[9]  = p0_PARAM(S);
    x0[10] = q0_PARAM(S);
    x0[11] = r0_PARAM(S);
}



/* Function: mdlOutputs =======================================================
 * Abstract:
 *      output function
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetContStates(S);
 
    UNUSED_ARG(tid); /* not used in single tasking mode */
    
    /* output the states */
    y[0]  = x[0];
    y[1]  = x[1];
    y[2]  = x[2];
    y[3]  = x[3];
    y[4]  = x[4];
    y[5]  = x[5];
    y[6]  = x[6];
    y[7]  = x[7];
    y[8]  = x[8];
    y[9]  = x[9];
    y[10] = x[10];
    y[11] = x[11];
}



#define MDL_DERIVATIVES
/* Function: mdlDerivatives =================================================
 * Abstract:
 *      Calculate state-space derivatives
 */
static void mdlDerivatives(SimStruct *S)
{
    real_T            *dx   = ssGetdX(S);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType forcePtrs  = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType torquePtrs = ssGetInputPortRealSignalPtrs(S,1);

    const real_T      mass = mass_PARAM(S);
    const real_T      Jx   = Jx_PARAM(S);
    const real_T      Jy   = Jy_PARAM(S);
    const real_T      Jz   = Jz_PARAM(S);
    const real_T      Jxz  = Jxz_PARAM(S);
    const real_T      pn    = x[0];
    const real_T      pe    = x[1];
    const real_T      pd    = x[2];
    const real_T      u     = x[3];
    const real_T      v     = x[4];
    const real_T      w     = x[5];
    const real_T      phi   = x[6];
    const real_T      theta = x[7];
    const real_T      psi   = x[8];
    const real_T      p     = x[9];
    const real_T      q     = x[10];
    const real_T      r     = x[11];
    const real_T      Gamma = Jx*Jz-Jxz*Jxz;
    const real_T      Gamma1 = (Jxz*(Jx-Jy+Jz))/Gamma;
    const real_T      Gamma2 = (Jz*(Jz-Jy)+Jxz*Jxz)/Gamma;
    const real_T      Gamma3 = Jx/Gamma;
    const real_T      Gamma4 = Jxz/Gamma;
    const real_T      Gamma5 = (Jz-Jx)/Gamma;
    const real_T      Gamma7 = (Jx*(Jx-Jy)+Jxz*Jxz)/Gamma;
    const real_T      Gamma8 = Jx/Gamma;
    
    /* update pn */
    dx[0] = cos(theta)*cos(psi)*u 
            + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v 
            + (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
    
    /* update pe */
    dx[1] = cos(theta)*sin(psi)*u 
            + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v 
            + (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;
    
    /* update pd */
    dx[2] = -sin(theta)*u 
            + sin(phi)*cos(theta)*w 
            + cos(phi)*cos(theta)*w;
    
    /* update u */
    dx[3] = r*v - q*w + (1/mass)*Force(0);
    
    /* update v */
    dx[4] = p*w - r*u + (1/mass)*Force(1); 
    
    /* update w */
    dx[5] = q*u - p*v + (1/mass)*Force(2);
    
    /* update phi */
    dx[6] = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    
    /* update theta */
    dx[7] = cos(phi)*q - sin(phi)*r;
    
    /* update psi */
    dx[8] = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    
    /* update p */
    dx[9] = Gamma1*p*q - Gamma2*q*r + Gamma3*Torque(0) + Gamma4*Torque(2);
    
    /* update q */
    dx[10] = Gamma5*p*r - Gamma4*(p*p-r*r) + (1/Jy)*Torque(1);
    
    /* update r */
    dx[11] = Gamma7*p*q - Gamma1*q*r + Gamma4*Torque(0) + Gamma8*Torque(2);
        
}



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
