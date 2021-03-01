#ifndef RTW_HEADER_Simulacao_CubeSat_2U_h_
#define RTW_HEADER_Simulacao_CubeSat_2U_h_
#include <stddef.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef Simulacao_CubeSat_2U_COMMON_INCLUDES_
#define Simulacao_CubeSat_2U_COMMON_INCLUDES_
#include <stdlib.h>
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "sigstream_rtw.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging.h"
#include "dt_info.h"
#include "ext_work.h"
#endif
#include "Simulacao_CubeSat_2U_types.h"
#include "multiword_types.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME Simulacao_CubeSat_2U
#define NSAMPLE_TIMES (3) 
#define NINPUTS (0)       
#define NOUTPUTS (3)     
#define NBLOCKIO (111) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (28)   
#elif NCSTATES != 28
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T be2k3eehde [ 3 ] ; real_T bw5bw3vp2b [ 3 ] ; real_T
aybmmyshj1 ; real_T mdhpjh2adi ; real_T dxigiaxorc ; real_T puqs4ihhyx ;
real_T jliilwwxof [ 3 ] ; real_T d14tgqcbbt ; real_T eb41sd25gs ; real_T
a1cc0ujdnb ; real_T ggo2whvj2q ; real_T mbwrtcrvid ; real_T h5ntalntsc ;
real_T dtorfd0zl5 ; real_T gbvzeqi2ho ; real_T nvgssnwpvd ; real_T euqixwxw5j
; real_T oh5z4jek0e ; real_T m4ws5llhyu ; real_T k0nyghzbp5 ; real_T
ildifdanq4 ; real_T czz21insjf ; real_T jlh0khwdqp ; real_T h34m1yd3v1 ;
real_T he1z03zbh1 ; real_T awws3phtzw ; real_T ozljxlqodx ; real_T dpbaerun23
; real_T hjhqehj3x3 ; real_T n2wfqwfqmg ; real_T d32cmmaovb ; real_T
kv2qrw5w4v [ 3 ] ; real_T jtngam5jo1 [ 3 ] ; real_T pukllw55sc [ 3 ] ; real_T
oawzrd2fap [ 3 ] ; real_T ixt1lyi0uk [ 3 ] ; real_T mwsgemvnjp [ 3 ] ; real_T
cw2qkgdy3d [ 3 ] ; real_T pt050oqs33 [ 3 ] ; real_T jcfxyc04gv ; real_T
m15qtoxlrb ; real_T o3yaxognir ; real_T bkerdhlz21 ; real_T cnr1sap0ta ;
real_T c53gqvzui4 ; real_T lr2oau0b2h [ 3 ] ; real_T itrtld5gdv ; real_T
g4yplqoljb ; real_T mef4idopue ; real_T jjrfmdioo5 [ 6 ] ; real_T kpkzm3xrw0
[ 3 ] ; real_T mcpejsoler ; real_T po3v4hxd2x ; real_T i2uukkt5c5 ; real_T
omvaw4qbz5 ; real_T cdf4j2oia5 ; real_T d1km4p5z5z ; real_T fxgwwlskk5 [ 3 ]
; real_T lwso1yhdt3 [ 3 ] ; real_T lzhk1bat0b [ 3 ] ; real_T f14n0vhlpm ;
real_T lltotnako5 ; real_T dfclg52ijb ; real_T nnyu5bhnzg ; real_T c13zwqewsd
; real_T ct4qzfubwd ; real_T dwcijdvmru ; real_T bubyfumtha ; real_T
fmcxkih41a ; real_T hxkjpgzbj0 ; real_T ldgnkshida ; real_T n1a0nanns0 ;
real_T ghzw50stn2 ; real_T hgqpvrhtn1 ; real_T jbxhal5qlm ; real_T evg2nnzcup
; real_T djenag5lsa ; real_T nxta40z0ko ; real_T pdrmt5tfxg ; real_T
lgdwswu25a ; real_T lyw51fa2jx ; real_T kir1w2sib3 ; real_T jcuns1utoi ;
real_T pt22dgyfvj ; real_T kj20n3vogf ; real_T j1yo03ylgg ; real_T mxulceln2p
; real_T nu1y2cd1qq ; real_T azerdqlxje ; real_T ds3hegtm1k ; real_T
lbrkyipcu1 ; real_T pif2vdtegn ; real_T jo1d214ytt ; real_T fzqohd2it0 ;
real_T aqx4tvtzvm ; real_T lajbwoy2xm ; real_T k5axb1m3zv ; real_T m1cqvkhqk0
; real_T b1a4hrzps4 ; real_T hr4yo0kcst ; real_T a1qq2dxnqb ; real_T
f0jsmwtplm ; real_T nh2m1oytuh ; real_T kxgyu2harm ; real_T m0qcww2pvi ;
real_T kmqaciilve ; real_T fngth1py0k ; real_T kx32hccv01 ; real_T cf14ylhfvy
[ 9 ] ; real_T og2fpsoerg [ 3 ] ; real_T ouqbmp1b5z [ 3 ] ; } B ; typedef
struct { real_T fhygfszyec [ 9 ] ; real_T i1vgghla5i [ 9 ] ; real_T
hhdvp3o3pb [ 9 ] ; struct { void * LoggedData ; } me2bozl00d ; struct { void
* LoggedData ; } g4w3wrop4k ; struct { void * AQHandles ; void * SlioLTF ; }
n5ugrmujkv ; void * ed03rf3e4o ; int32_T oqblwxzcpy [ 3 ] ; int_T phbdv43v20
; } DW ; typedef struct { real_T dcnmiwpca4 [ 3 ] ; real_T o212iwmhcr ;
real_T ek4na1z4yp ; real_T oqdfossbpr ; real_T kwhjgsltwm ; real_T m5042oisqd
; real_T n5o4uxbgia ; real_T e2f4sjt1iw ; real_T chmzyjmfc4 ; real_T
lwnibsigzh ; real_T dea0nf5vx3 ; real_T p4a4q40pcu [ 3 ] ; real_T oawe0mghaa
[ 3 ] ; real_T pmg0blojut ; real_T jl3mspj5c3 ; real_T c0el2g45hw ; real_T
cknsuq2zlh ; real_T c52k0nbe0q ; real_T gntqn5o133 ; real_T j1uzwdz34w ;
real_T iiwn0ghbg0 ; real_T nvmluflpaj ; } X ; typedef struct { real_T
dcnmiwpca4 [ 3 ] ; real_T o212iwmhcr ; real_T ek4na1z4yp ; real_T oqdfossbpr
; real_T kwhjgsltwm ; real_T m5042oisqd ; real_T n5o4uxbgia ; real_T
e2f4sjt1iw ; real_T chmzyjmfc4 ; real_T lwnibsigzh ; real_T dea0nf5vx3 ;
real_T p4a4q40pcu [ 3 ] ; real_T oawe0mghaa [ 3 ] ; real_T pmg0blojut ;
real_T jl3mspj5c3 ; real_T c0el2g45hw ; real_T cknsuq2zlh ; real_T c52k0nbe0q
; real_T gntqn5o133 ; real_T j1uzwdz34w ; real_T iiwn0ghbg0 ; real_T
nvmluflpaj ; } XDot ; typedef struct { boolean_T dcnmiwpca4 [ 3 ] ; boolean_T
o212iwmhcr ; boolean_T ek4na1z4yp ; boolean_T oqdfossbpr ; boolean_T
kwhjgsltwm ; boolean_T m5042oisqd ; boolean_T n5o4uxbgia ; boolean_T
e2f4sjt1iw ; boolean_T chmzyjmfc4 ; boolean_T lwnibsigzh ; boolean_T
dea0nf5vx3 ; boolean_T p4a4q40pcu [ 3 ] ; boolean_T oawe0mghaa [ 3 ] ;
boolean_T pmg0blojut ; boolean_T jl3mspj5c3 ; boolean_T c0el2g45hw ;
boolean_T cknsuq2zlh ; boolean_T c52k0nbe0q ; boolean_T gntqn5o133 ;
boolean_T j1uzwdz34w ; boolean_T iiwn0ghbg0 ; boolean_T nvmluflpaj ; } XDis ;
typedef struct { real_T dcnmiwpca4 [ 3 ] ; real_T o212iwmhcr ; real_T
ek4na1z4yp ; real_T oqdfossbpr ; real_T kwhjgsltwm ; real_T m5042oisqd ;
real_T n5o4uxbgia ; real_T e2f4sjt1iw ; real_T chmzyjmfc4 ; real_T lwnibsigzh
; real_T dea0nf5vx3 ; real_T p4a4q40pcu [ 3 ] ; real_T oawe0mghaa [ 3 ] ;
real_T pmg0blojut ; real_T jl3mspj5c3 ; real_T c0el2g45hw ; real_T cknsuq2zlh
; real_T c52k0nbe0q ; real_T gntqn5o133 ; real_T j1uzwdz34w ; real_T
iiwn0ghbg0 ; real_T nvmluflpaj ; } CStateAbsTol ; typedef struct { real_T
dcnmiwpca4 [ 3 ] ; real_T o212iwmhcr ; real_T ek4na1z4yp ; real_T oqdfossbpr
; real_T kwhjgsltwm ; real_T m5042oisqd ; real_T n5o4uxbgia ; real_T
e2f4sjt1iw ; real_T chmzyjmfc4 ; real_T lwnibsigzh ; real_T dea0nf5vx3 ;
real_T p4a4q40pcu [ 3 ] ; real_T oawe0mghaa [ 3 ] ; real_T pmg0blojut ;
real_T jl3mspj5c3 ; real_T c0el2g45hw ; real_T cknsuq2zlh ; real_T c52k0nbe0q
; real_T gntqn5o133 ; real_T j1uzwdz34w ; real_T iiwn0ghbg0 ; real_T
nvmluflpaj ; } CXPtMin ; typedef struct { real_T dcnmiwpca4 [ 3 ] ; real_T
o212iwmhcr ; real_T ek4na1z4yp ; real_T oqdfossbpr ; real_T kwhjgsltwm ;
real_T m5042oisqd ; real_T n5o4uxbgia ; real_T e2f4sjt1iw ; real_T chmzyjmfc4
; real_T lwnibsigzh ; real_T dea0nf5vx3 ; real_T p4a4q40pcu [ 3 ] ; real_T
oawe0mghaa [ 3 ] ; real_T pmg0blojut ; real_T jl3mspj5c3 ; real_T c0el2g45hw
; real_T cknsuq2zlh ; real_T c52k0nbe0q ; real_T gntqn5o133 ; real_T
j1uzwdz34w ; real_T iiwn0ghbg0 ; real_T nvmluflpaj ; } CXPtMax ; typedef
struct { real_T isfgwtajws [ 3 ] ; } ExtY ; typedef struct {
rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T J ; real_T
Jb [ 9 ] ; real_T Kb ; real_T Km ; real_T Te ; real_T Wo [ 3 ] ; real_T
PIDController_D ; real_T PIDController2_D ; real_T PIDController3_D ; real_T
PIDController1_D ; real_T PIDController_I ; real_T PIDController2_I ; real_T
PIDController3_I ; real_T PIDController1_I ; real_T
PIDController_I_g1oln5gyqa ; real_T PIDController_I_kd2qtfttcs ; real_T
PIDController_I_gnsusa3wqb ; real_T PIDController_InitialConditionForFilter ;
real_T PIDController2_InitialConditionForFilter ; real_T
PIDController3_InitialConditionForFilter ; real_T
PIDController1_InitialConditionForFilter ; real_T
PIDController_InitialConditionForIntegrator ; real_T
PIDController2_InitialConditionForIntegrator ; real_T
PIDController3_InitialConditionForIntegrator ; real_T
PIDController1_InitialConditionForIntegrator ; real_T
PIDController_InitialConditionForIntegrator_nrg5o00kkv ; real_T
PIDController_InitialConditionForIntegrator_bo4ekjqhoq ; real_T
PIDController_InitialConditionForIntegrator_ng4h3bnigz ; real_T
PIDController_N ; real_T PIDController2_N ; real_T PIDController3_N ; real_T
PIDController1_N ; real_T PIDController_P ; real_T PIDController2_P ; real_T
PIDController3_P ; real_T PIDController_P_jlpxdayd0n ; real_T
PIDController_P_c11psn2fmh ; real_T PIDController_P_er1lkjtlz2 ; real_T q0_IC
; real_T q1_IC ; real_T q2_IC ; real_T q3_IC ; real_T TransferFcn1_A ; real_T
TransferFcn1_C ; real_T TransferFcn1_A_cnuk0h4tar ; real_T
TransferFcn1_C_gfroigzrf4 ; real_T TransferFcn1_A_gcfopbqkvy ; real_T
TransferFcn1_C_ls3sy5taih ; real_T TransferFcn_A ; real_T TransferFcn_C ;
real_T TransferFcn_A_kbikinkyoh ; real_T TransferFcn_C_dcilw4n42x ; real_T
TransferFcn_A_ljiqx5nlbh ; real_T TransferFcn_C_cjzp5uwabc ; real_T Gain_Gain
; real_T Gain1_Gain ; real_T Gain2_Gain ; real_T Gain3_Gain ; real_T
AtitudeDesejadaAquisiao_Value [ 3 ] ; real_T VelocidadeAngularDetumble_Value
[ 3 ] ; } ; extern const char * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ;
extern X rtX ; extern DW rtDW ; extern ExtY rtY ; extern P rtP ; extern const
rtwCAPI_ModelMappingStaticInfo * Simulacao_CubeSat_2U_GetCAPIStaticMap ( void
) ; extern SimStruct * const rtS ; extern const int_T gblNumToFiles ; extern
const int_T gblNumFrFiles ; extern const int_T gblNumFrWksBlocks ; extern
rtInportTUtable * gblInportTUtables ; extern const char * gblInportFileName ;
extern const int_T gblNumRootInportBlks ; extern const int_T
gblNumModelInputs ; extern const int_T gblInportDataTypeIdx [ ] ; extern
const int_T gblInportDims [ ] ; extern const int_T gblInportComplex [ ] ;
extern const int_T gblInportInterpoFlag [ ] ; extern const int_T
gblInportContinuous [ ] ; extern const int_T gblParameterTuningTid ; extern
DataMapInfo * rt_dataMapInfoPtr ; extern rtwCAPI_ModelMappingInfo *
rt_modelMapInfoPtr ; void MdlOutputs ( int_T tid ) ; void
MdlOutputsParameterSampleTime ( int_T tid ) ; void MdlUpdate ( int_T tid ) ;
void MdlTerminate ( void ) ; void MdlInitializeSizes ( void ) ; void
MdlInitializeSampleTimes ( void ) ; SimStruct * raccel_register_model ( void
) ;
#endif
