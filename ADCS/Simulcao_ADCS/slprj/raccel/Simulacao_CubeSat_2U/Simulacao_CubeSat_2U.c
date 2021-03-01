#include "rt_logging_mmi.h"
#include "Simulacao_CubeSat_2U_capi.h"
#include <math.h>
#include "Simulacao_CubeSat_2U.h"
#include "Simulacao_CubeSat_2U_private.h"
#include "Simulacao_CubeSat_2U_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; RTWExtModeInfo * gblRTWExtModeInfo = NULL ; extern boolean_T
gblExtModeStartPktReceived ; void raccelForceExtModeShutdown ( ) { if ( !
gblExtModeStartPktReceived ) { boolean_T stopRequested = false ;
rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 2 , & stopRequested ) ; }
rtExtModeShutdown ( 2 ) ; }
#include "slsv_diagnostic_codegen_c_api.h"
const int_T gblNumToFiles = 0 ; const int_T gblNumFrFiles = 0 ; const int_T
gblNumFrWksBlocks = 0 ;
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 0 ; int_T gbl_raccel_NumST = 3 ; const char_T
* gbl_raccel_Version = "9.3 (R2020a) 18-Nov-2019" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#else
UNUSED_PARAMETER ( S ) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; const char * gblSlvrJacPatternFileName =
"slprj\\raccel\\Simulacao_CubeSat_2U\\Simulacao_CubeSat_2U_Jpattern.mat" ;
const int_T gblNumRootInportBlks = 0 ; const int_T gblNumModelInputs = 0 ;
extern rtInportTUtable * gblInportTUtables ; extern const char *
gblInportFileName ; extern void * gblAperiodicPartitionHitTimes ; const int_T
gblInportDataTypeIdx [ ] = { - 1 } ; const int_T gblInportDims [ ] = { - 1 }
; const int_T gblInportComplex [ ] = { - 1 } ; const int_T
gblInportInterpoFlag [ ] = { - 1 } ; const int_T gblInportContinuous [ ] = {
- 1 } ; int_T enableFcnCallFlag [ ] = { 1 , 1 , 1 } ; const char *
raccelLoadInputsAndAperiodicHitTimes ( const char * inportFileName , int *
matFileFormat ) { return rt_RapidReadInportsMatFile ( inportFileName ,
matFileFormat , 1 ) ; }
#include "simstruc.h"
#include "fixedpoint.h"
B rtB ; X rtX ; DW rtDW ; ExtY rtY ; static SimStruct model_S ; SimStruct *
const rtS = & model_S ; void rt_invd3x3_snf ( const real_T u [ 9 ] , real_T y
[ 9 ] ) { real_T u_p [ 9 ] ; int32_T zero ; int32_T three ; int32_T six ;
int32_T p1 ; real_T absx11 ; real_T absx21 ; real_T absx31 ; real_T y_p ; for
( three = 0 ; three < 9 ; three ++ ) { absx31 = u [ three ] ; u_p [ three ] =
absx31 ; } three = 3 ; six = 6 ; p1 = 0 ; absx31 = u_p [ 0 ] ; absx11 =
muDoubleScalarAbs ( absx31 ) ; absx31 = u_p [ 1 ] ; absx21 =
muDoubleScalarAbs ( absx31 ) ; absx31 = u_p [ 2 ] ; absx31 =
muDoubleScalarAbs ( absx31 ) ; if ( ( absx21 > absx11 ) && ( absx21 > absx31
) ) { p1 = 3 ; three = 0 ; absx21 = u_p [ 0 ] ; u_p [ 0 ] = u_p [ 1 ] ; u_p [
1 ] = absx21 ; absx21 = u_p [ 3 ] ; u_p [ 3 ] = u_p [ 4 ] ; u_p [ 4 ] =
absx21 ; absx21 = u_p [ 6 ] ; u_p [ 6 ] = u_p [ 7 ] ; u_p [ 7 ] = absx21 ; }
else { if ( absx31 > absx11 ) { p1 = 6 ; six = 0 ; absx21 = u_p [ 0 ] ; u_p [
0 ] = u_p [ 2 ] ; u_p [ 2 ] = absx21 ; absx21 = u_p [ 3 ] ; u_p [ 3 ] = u_p [
5 ] ; u_p [ 5 ] = absx21 ; absx21 = u_p [ 6 ] ; u_p [ 6 ] = u_p [ 8 ] ; u_p [
8 ] = absx21 ; } } absx31 = u_p [ 1 ] ; y_p = u_p [ 0 ] ; absx31 /= y_p ; u_p
[ 1 ] = absx31 ; absx31 = u_p [ 2 ] ; y_p = u_p [ 0 ] ; absx31 /= y_p ; u_p [
2 ] = absx31 ; u_p [ 4 ] -= u_p [ 1 ] * u_p [ 3 ] ; u_p [ 5 ] -= u_p [ 2 ] *
u_p [ 3 ] ; u_p [ 7 ] -= u_p [ 1 ] * u_p [ 6 ] ; u_p [ 8 ] -= u_p [ 2 ] * u_p
[ 6 ] ; absx31 = u_p [ 5 ] ; y_p = muDoubleScalarAbs ( absx31 ) ; absx31 =
u_p [ 4 ] ; absx31 = muDoubleScalarAbs ( absx31 ) ; if ( y_p > absx31 ) {
zero = three ; three = six ; six = zero ; absx21 = u_p [ 1 ] ; u_p [ 1 ] =
u_p [ 2 ] ; u_p [ 2 ] = absx21 ; absx21 = u_p [ 4 ] ; u_p [ 4 ] = u_p [ 5 ] ;
u_p [ 5 ] = absx21 ; absx21 = u_p [ 7 ] ; u_p [ 7 ] = u_p [ 8 ] ; u_p [ 8 ] =
absx21 ; } absx31 = u_p [ 5 ] ; y_p = u_p [ 4 ] ; absx31 /= y_p ; u_p [ 5 ] =
absx31 ; u_p [ 8 ] -= u_p [ 5 ] * u_p [ 7 ] ; absx31 = u_p [ 5 ] * u_p [ 1 ]
- u_p [ 2 ] ; y_p = u_p [ 8 ] ; absx11 = absx31 / y_p ; absx31 = - ( u_p [ 7
] * absx11 + u_p [ 1 ] ) ; y_p = u_p [ 4 ] ; absx21 = absx31 / y_p ; zero =
p1 ; absx31 = ( 1.0 - u_p [ 3 ] * absx21 ) - u_p [ 6 ] * absx11 ; y_p = u_p [
0 ] ; absx31 /= y_p ; y [ zero ] = absx31 ; zero = p1 + 1 ; y [ zero ] =
absx21 ; zero = p1 + 2 ; y [ zero ] = absx11 ; absx31 = - u_p [ 5 ] ; y_p =
u_p [ 8 ] ; absx11 = absx31 / y_p ; absx31 = 1.0 - u_p [ 7 ] * absx11 ; y_p =
u_p [ 4 ] ; absx21 = absx31 / y_p ; zero = three ; absx31 = - ( u_p [ 3 ] *
absx21 + u_p [ 6 ] * absx11 ) ; y_p = u_p [ 0 ] ; absx31 /= y_p ; y [ zero ]
= absx31 ; zero = three + 1 ; y [ zero ] = absx21 ; zero = three + 2 ; y [
zero ] = absx11 ; y_p = u_p [ 8 ] ; absx11 = 1.0 / y_p ; absx31 = - u_p [ 7 ]
* absx11 ; y_p = u_p [ 4 ] ; absx21 = absx31 / y_p ; zero = six ; absx31 = -
( u_p [ 3 ] * absx21 + u_p [ 6 ] * absx11 ) ; y_p = u_p [ 0 ] ; absx31 /= y_p
; y [ zero ] = absx31 ; zero = six + 1 ; y [ zero ] = absx21 ; zero = six + 2
; y [ zero ] = absx11 ; } void MdlInitialize ( void ) { boolean_T tmp ; rtDW
. phbdv43v20 = 1 ; if ( ssIsFirstInitCond ( rtS ) ) { rtX . dcnmiwpca4 [ 0 ]
= 0.0 ; rtX . dcnmiwpca4 [ 1 ] = 0.0 ; rtX . dcnmiwpca4 [ 2 ] =
6.2831853071795853 ; tmp = slIsRapidAcceleratorSimulating ( ) ; if ( tmp ) {
tmp = ssGetGlobalInitialStatesAvailable ( rtS ) ; rtDW . phbdv43v20 = ! tmp ;
} else { rtDW . phbdv43v20 = 1 ; } } rtX . o212iwmhcr = rtP . q0_IC ; rtX .
ek4na1z4yp = rtP . q1_IC ; rtX . oqdfossbpr = rtP . q2_IC ; rtX . kwhjgsltwm
= rtP . q3_IC ; rtX . m5042oisqd = rtP .
PIDController_InitialConditionForFilter ; rtX . n5o4uxbgia = rtP .
PIDController_InitialConditionForIntegrator ; rtX . e2f4sjt1iw = rtP .
PIDController2_InitialConditionForFilter ; rtX . chmzyjmfc4 = rtP .
PIDController2_InitialConditionForIntegrator ; rtX . lwnibsigzh = rtP .
PIDController3_InitialConditionForFilter ; rtX . dea0nf5vx3 = rtP .
PIDController3_InitialConditionForIntegrator ; rtX . p4a4q40pcu [ 0 ] = rtP .
PIDController1_InitialConditionForFilter ; rtX . oawe0mghaa [ 0 ] = rtP .
PIDController1_InitialConditionForIntegrator ; rtX . p4a4q40pcu [ 1 ] = rtP .
PIDController1_InitialConditionForFilter ; rtX . oawe0mghaa [ 1 ] = rtP .
PIDController1_InitialConditionForIntegrator ; rtX . p4a4q40pcu [ 2 ] = rtP .
PIDController1_InitialConditionForFilter ; rtX . oawe0mghaa [ 2 ] = rtP .
PIDController1_InitialConditionForIntegrator ; rtX . pmg0blojut = 0.0 ; rtX .
jl3mspj5c3 = 0.0 ; rtX . c0el2g45hw = 0.0 ; rtX . cknsuq2zlh = 0.0 ; rtX .
c52k0nbe0q = 0.0 ; rtX . gntqn5o133 = 0.0 ; rtX . j1uzwdz34w = rtP .
PIDController_InitialConditionForIntegrator_nrg5o00kkv ; rtX . iiwn0ghbg0 =
rtP . PIDController_InitialConditionForIntegrator_bo4ekjqhoq ; rtX .
nvmluflpaj = rtP . PIDController_InitialConditionForIntegrator_ng4h3bnigz ; }
void MdlStart ( void ) { void * catalog ; { void * * slioCatalogueAddr =
rt_slioCatalogueAddr ( ) ; void * r2 = ( NULL ) ; void * *
pOSigstreamManagerAddr = ( NULL ) ; const int maxErrorBufferSize = 16384 ;
char errMsgCreatingOSigstreamManager [ 16384 ] ; bool
errorCreatingOSigstreamManager = false ; const char *
errorAddingR2SharedResource = ( NULL ) ; * slioCatalogueAddr =
rtwGetNewSlioCatalogue ( rt_GetMatSigLogSelectorFileName ( ) ) ;
errorAddingR2SharedResource = rtwAddR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) , 1 ) ; if (
errorAddingR2SharedResource != ( NULL ) ) { rtwTerminateSlioCatalogue (
slioCatalogueAddr ) ; * slioCatalogueAddr = ( NULL ) ; ssSetErrorStatus ( rtS
, errorAddingR2SharedResource ) ; return ; } r2 = rtwGetR2SharedResource (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ) ;
pOSigstreamManagerAddr = rt_GetOSigstreamManagerAddr ( ) ;
errorCreatingOSigstreamManager = rtwOSigstreamManagerCreateInstance (
rt_GetMatSigLogSelectorFileName ( ) , r2 , pOSigstreamManagerAddr ,
errMsgCreatingOSigstreamManager , maxErrorBufferSize ) ; if (
errorCreatingOSigstreamManager ) { * pOSigstreamManagerAddr = ( NULL ) ;
ssSetErrorStatus ( rtS , errMsgCreatingOSigstreamManager ) ; return ; } } {
bool externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( ) ; rtwISigstreamManagerGetInputIsInDatasetFormat (
pISigstreamManager , & externalInputIsInDatasetFormat ) ; if (
externalInputIsInDatasetFormat ) { } } { void * slioCatalogue =
rt_slioCatalogue ( ) ? rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) :
sdiGetSlioCatalogue ( rt_dataMapInfo . mmi . InstanceMap . fullPath ) ; if (
! slioCatalogue || ! rtwIsLoggingToFile ( slioCatalogue ) ) { { {
sdiSignalSourceInfoU srcInfo ; sdiLabelU loggedName = sdiGetLabelFromChars (
"" ) ; sdiLabelU origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU
propName = sdiGetLabelFromChars ( "" ) ; sdiLabelU blockPath =
sdiGetLabelFromChars ( "Simulacao_CubeSat_2U/Velocidade Angular CubeSat" ) ;
sdiLabelU blockSID = sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath =
sdiGetLabelFromChars ( "" ) ; sdiDims sigDims ; sdiLabelU sigName =
sdiGetLabelFromChars ( "" ) ; sdiAsyncRepoDataTypeHandle hDT =
sdiAsyncRepoGetBuiltInDataTypeHandle ( DATA_TYPE_DOUBLE ) ; { sdiComplexity
sigComplexity = REAL ; sdiSampleTimeContinuity stCont =
SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray [ 2 ] = { 3 , 1 } ; sigDims .
nDims = 2 ; sigDims . dimensions = sigDimsArray ; srcInfo . numBlockPathElems
= 1 ; srcInfo . fullBlockPath = ( sdiFullBlkPathU ) & blockPath ; srcInfo .
SID = ( sdiSignalIDU ) & blockSID ; srcInfo . subPath = subPath ; srcInfo .
portIndex = 0 + 1 ; srcInfo . signalName = sigName ; srcInfo . sigSourceUUID
= 0 ; rtDW . n5ugrmujkv . AQHandles = sdiAsyncRepoCreateAsyncioQueue ( hDT ,
& srcInfo , rt_dataMapInfo . mmi . InstanceMap . fullPath ,
"e6397b9c-3747-457f-86b2-bed3358f472c" , sigComplexity , & sigDims ,
DIMENSIONS_MODE_FIXED , stCont , "" ) ; if ( rtDW . n5ugrmujkv . AQHandles )
{ sdiSetSignalSampleTimeString ( rtDW . n5ugrmujkv . AQHandles , "Continuous"
, 0.0 , ssGetTFinal ( rtS ) ) ; sdiSetRunStartTime ( rtDW . n5ugrmujkv .
AQHandles , ssGetTaskTime ( rtS , 1 ) ) ; sdiAsyncRepoSetSignalExportSettings
( rtDW . n5ugrmujkv . AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName (
rtDW . n5ugrmujkv . AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetSignalDomainType ( rtDW . n5ugrmujkv . AQHandles , "outport" )
; sdiAsyncRepoSetSignalExportOrder ( rtDW . n5ugrmujkv . AQHandles , 1 ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } } } rtDW .
n5ugrmujkv . SlioLTF = ( NULL ) ; } rtB . og2fpsoerg [ 0 ] = rtP . Wo [ 0 ] ;
rtB . og2fpsoerg [ 1 ] = rtP . Wo [ 1 ] ; rtB . og2fpsoerg [ 2 ] = rtP . Wo [
2 ] ; catalog = rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ; rtDW .
ed03rf3e4o = rt_SlioAccessorAddClientAssessmentSdi ( 1 , 4 , catalog , rtDW .
ed03rf3e4o , "Assertion" ,
"Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Assertion" ) ;
MdlInitialize ( ) ; } void MdlOutputs ( int_T tid ) { real_T C11 ; real_T C12
; real_T C23 ; real_T C33 ; int32_T assessmentVar = - 1 ; void *
assessmentPtrVar ; int32_T i ; SimStruct * S ; void * diag ; const real_T *
tmp ; if ( rtDW . phbdv43v20 != 0 ) { rtX . dcnmiwpca4 [ 0 ] = rtB .
og2fpsoerg [ 0 ] ; rtX . dcnmiwpca4 [ 1 ] = rtB . og2fpsoerg [ 1 ] ; rtX .
dcnmiwpca4 [ 2 ] = rtB . og2fpsoerg [ 2 ] ; } rtB . be2k3eehde [ 0 ] = rtX .
dcnmiwpca4 [ 0 ] ; rtB . bw5bw3vp2b [ 0 ] = rtB . be2k3eehde [ 0 ] ; rtY .
isfgwtajws [ 0 ] = rtB . bw5bw3vp2b [ 0 ] ; rtB . be2k3eehde [ 1 ] = rtX .
dcnmiwpca4 [ 1 ] ; rtB . bw5bw3vp2b [ 1 ] = rtB . be2k3eehde [ 1 ] ; rtY .
isfgwtajws [ 1 ] = rtB . bw5bw3vp2b [ 1 ] ; rtB . be2k3eehde [ 2 ] = rtX .
dcnmiwpca4 [ 2 ] ; rtB . bw5bw3vp2b [ 2 ] = rtB . be2k3eehde [ 2 ] ; rtY .
isfgwtajws [ 2 ] = rtB . bw5bw3vp2b [ 2 ] ; rtB . aybmmyshj1 = rtX .
o212iwmhcr ; rtB . mdhpjh2adi = rtX . ek4na1z4yp ; rtB . dxigiaxorc = rtX .
oqdfossbpr ; rtB . puqs4ihhyx = rtX . kwhjgsltwm ; C11 = ( ( rtB . aybmmyshj1
* rtB . aybmmyshj1 + rtB . mdhpjh2adi * rtB . mdhpjh2adi ) - rtB . dxigiaxorc
* rtB . dxigiaxorc ) - rtB . puqs4ihhyx * rtB . puqs4ihhyx ; C12 = ( rtB .
mdhpjh2adi * rtB . dxigiaxorc + rtB . aybmmyshj1 * rtB . puqs4ihhyx ) * 2.0 ;
C23 = ( rtB . dxigiaxorc * rtB . puqs4ihhyx + rtB . aybmmyshj1 * rtB .
mdhpjh2adi ) * 2.0 ; C33 = ( ( rtB . aybmmyshj1 * rtB . aybmmyshj1 - rtB .
mdhpjh2adi * rtB . mdhpjh2adi ) - rtB . dxigiaxorc * rtB . dxigiaxorc ) + rtB
. puqs4ihhyx * rtB . puqs4ihhyx ; if ( ( C12 >= 0.0 ) && ( C11 >= 0.0 ) ) {
if ( ( C23 >= 0.0 ) && ( C33 >= 0.0 ) ) { C11 = muDoubleScalarAtan ( C12 /
C11 ) * 57.295779513082323 ; C23 = muDoubleScalarAtan ( C23 / C33 ) *
57.295779513082323 ; } else if ( ( C23 >= 0.0 ) && ( C33 <= 0.0 ) ) { C11 =
muDoubleScalarAtan ( C12 / C11 ) * 57.295779513082323 ; C23 =
muDoubleScalarAtan ( C23 / C33 ) * 57.295779513082323 + 180.0 ; } else if ( (
C23 <= 0.0 ) && ( C33 <= 0.0 ) ) { C11 = muDoubleScalarAtan ( C12 / C11 ) *
57.295779513082323 ; C23 = muDoubleScalarAtan ( C23 / C33 ) *
57.295779513082323 - 180.0 ; } else if ( ( C23 <= 0.0 ) && ( C33 >= 0.0 ) ) {
C11 = muDoubleScalarAtan ( C12 / C11 ) * 57.295779513082323 ; C23 =
muDoubleScalarAtan ( C23 / C33 ) * 57.295779513082323 ; } else { C11 = 0.0 ;
C23 = 0.0 ; } } else if ( ( C12 >= 0.0 ) && ( C11 <= 0.0 ) ) { if ( ( C23 >=
0.0 ) && ( C33 >= 0.0 ) ) { C11 = muDoubleScalarAtan ( C12 / C11 ) *
57.295779513082323 + 180.0 ; C23 = muDoubleScalarAtan ( C23 / C33 ) *
57.295779513082323 ; } else if ( ( C23 >= 0.0 ) && ( C33 <= 0.0 ) ) { if ( (
rtB . mdhpjh2adi == 0.0 ) && ( rtB . puqs4ihhyx == 0.0 ) ) { C11 =
muDoubleScalarAtan ( C12 / C11 ) * 57.295779513082323 ; C23 =
muDoubleScalarAtan ( C23 / C33 ) * 57.295779513082323 ; } else { C11 =
muDoubleScalarAtan ( C12 / C11 ) * 57.295779513082323 + 180.0 ; C23 =
muDoubleScalarAtan ( C23 / C33 ) * 57.295779513082323 + 180.0 ; } } else if (
( C23 <= 0.0 ) && ( C33 <= 0.0 ) ) { C11 = muDoubleScalarAtan ( C12 / C11 ) *
57.295779513082323 + 180.0 ; C23 = muDoubleScalarAtan ( C23 / C33 ) *
57.295779513082323 - 180.0 ; } else if ( ( C23 <= 0.0 ) && ( C33 >= 0.0 ) ) {
C11 = muDoubleScalarAtan ( C12 / C11 ) * 57.295779513082323 + 180.0 ; C23 =
muDoubleScalarAtan ( C23 / C33 ) * 57.295779513082323 ; } else { C11 = 0.0 ;
C23 = 0.0 ; } } else if ( ( C12 <= 0.0 ) && ( C11 <= 0.0 ) ) { if ( ( C23 >=
0.0 ) && ( C33 >= 0.0 ) ) { C11 = muDoubleScalarAtan ( C12 / C11 ) *
57.295779513082323 - 180.0 ; C23 = muDoubleScalarAtan ( C23 / C33 ) *
57.295779513082323 ; } else if ( ( C23 >= 0.0 ) && ( C33 <= 0.0 ) ) { C11 =
muDoubleScalarAtan ( C12 / C11 ) * 57.295779513082323 - 180.0 ; C23 =
muDoubleScalarAtan ( C23 / C33 ) * 57.295779513082323 + 180.0 ; } else if ( (
C23 <= 0.0 ) && ( C33 <= 0.0 ) ) { C11 = muDoubleScalarAtan ( C12 / C11 ) *
57.295779513082323 - 180.0 ; C23 = muDoubleScalarAtan ( C23 / C33 ) *
57.295779513082323 - 180.0 ; } else if ( ( C23 <= 0.0 ) && ( C33 >= 0.0 ) ) {
C11 = muDoubleScalarAtan ( C12 / C11 ) * 57.295779513082323 - 180.0 ; C23 =
muDoubleScalarAtan ( C23 / C33 ) * 57.295779513082323 ; } else { C11 = 0.0 ;
C23 = 0.0 ; } } else if ( ( C12 <= 0.0 ) && ( C11 >= 0.0 ) ) { if ( ( C23 >=
0.0 ) && ( C33 >= 0.0 ) ) { C11 = muDoubleScalarAtan ( C12 / C11 ) *
57.295779513082323 ; C23 = muDoubleScalarAtan ( C23 / C33 ) *
57.295779513082323 ; } else if ( ( C23 >= 0.0 ) && ( C33 <= 0.0 ) ) { C11 =
muDoubleScalarAtan ( C12 / C11 ) * 57.295779513082323 ; C23 =
muDoubleScalarAtan ( C23 / C33 ) * 57.295779513082323 + 180.0 ; } else if ( (
C23 <= 0.0 ) && ( C33 <= 0.0 ) ) { C11 = muDoubleScalarAtan ( C12 / C11 ) *
57.295779513082323 ; C23 = muDoubleScalarAtan ( C23 / C33 ) *
57.295779513082323 - 180.0 ; } else if ( ( C23 <= 0.0 ) && ( C33 >= 0.0 ) ) {
C11 = muDoubleScalarAtan ( C12 / C11 ) * 57.295779513082323 ; C23 =
muDoubleScalarAtan ( C23 / C33 ) * 57.295779513082323 ; } else { C11 = 0.0 ;
C23 = 0.0 ; } } else { C11 = 0.0 ; C23 = 0.0 ; } rtB . ouqbmp1b5z [ 0 ] = C11
; rtB . ouqbmp1b5z [ 1 ] = muDoubleScalarAsin ( - ( ( rtB . mdhpjh2adi * rtB
. puqs4ihhyx - rtB . aybmmyshj1 * rtB . dxigiaxorc ) * 2.0 ) ) *
57.295779513082323 ; rtB . ouqbmp1b5z [ 2 ] = C23 ; if ( ssIsSampleHit ( rtS
, 1 , 0 ) ) { { if ( ( rtDW . n5ugrmujkv . AQHandles || rtDW . n5ugrmujkv .
SlioLTF ) && ssGetLogOutput ( rtS ) ) { sdiSlioSdiWriteSignal ( rtDW .
n5ugrmujkv . AQHandles , rtDW . n5ugrmujkv . SlioLTF , 0 , ssGetTaskTime (
rtS , 1 ) , ( char * ) & rtB . bw5bw3vp2b [ 0 ] + 0 ) ; } } } rtB .
eb41sd25gs = rtX . m5042oisqd ; rtB . mbwrtcrvid = rtX . n5o4uxbgia ; rtB .
euqixwxw5j = rtX . e2f4sjt1iw ; rtB . k0nyghzbp5 = rtX . chmzyjmfc4 ; rtB .
he1z03zbh1 = rtX . lwnibsigzh ; rtB . dpbaerun23 = rtX . dea0nf5vx3 ; rtB .
jliilwwxof [ 0 ] = rtP . VelocidadeAngularDetumble_Value [ 0 ] - rtB .
bw5bw3vp2b [ 0 ] ; rtB . kv2qrw5w4v [ 0 ] = rtP .
AtitudeDesejadaAquisiao_Value [ 0 ] - rtB . ouqbmp1b5z [ 0 ] ; rtB .
jtngam5jo1 [ 0 ] = rtP . PIDController1_D * rtB . kv2qrw5w4v [ 0 ] ; rtB .
pukllw55sc [ 0 ] = rtX . p4a4q40pcu [ 0 ] ; rtB . oawzrd2fap [ 0 ] = rtB .
jtngam5jo1 [ 0 ] - rtB . pukllw55sc [ 0 ] ; rtB . ixt1lyi0uk [ 0 ] = rtP .
PIDController1_I * rtB . kv2qrw5w4v [ 0 ] ; rtB . mwsgemvnjp [ 0 ] = rtX .
oawe0mghaa [ 0 ] ; rtB . cw2qkgdy3d [ 0 ] = rtP . PIDController1_N * rtB .
oawzrd2fap [ 0 ] ; rtB . jliilwwxof [ 1 ] = rtP .
VelocidadeAngularDetumble_Value [ 1 ] - rtB . bw5bw3vp2b [ 1 ] ; rtB .
kv2qrw5w4v [ 1 ] = rtP . AtitudeDesejadaAquisiao_Value [ 1 ] - rtB .
ouqbmp1b5z [ 1 ] ; rtB . jtngam5jo1 [ 1 ] = rtP . PIDController1_D * rtB .
kv2qrw5w4v [ 1 ] ; rtB . pukllw55sc [ 1 ] = rtX . p4a4q40pcu [ 1 ] ; rtB .
oawzrd2fap [ 1 ] = rtB . jtngam5jo1 [ 1 ] - rtB . pukllw55sc [ 1 ] ; rtB .
ixt1lyi0uk [ 1 ] = rtP . PIDController1_I * rtB . kv2qrw5w4v [ 1 ] ; rtB .
mwsgemvnjp [ 1 ] = rtX . oawe0mghaa [ 1 ] ; rtB . cw2qkgdy3d [ 1 ] = rtP .
PIDController1_N * rtB . oawzrd2fap [ 1 ] ; rtB . jliilwwxof [ 2 ] = rtP .
VelocidadeAngularDetumble_Value [ 2 ] - rtB . bw5bw3vp2b [ 2 ] ; rtB .
kv2qrw5w4v [ 2 ] = rtP . AtitudeDesejadaAquisiao_Value [ 2 ] - rtB .
ouqbmp1b5z [ 2 ] ; rtB . jtngam5jo1 [ 2 ] = rtP . PIDController1_D * rtB .
kv2qrw5w4v [ 2 ] ; rtB . pukllw55sc [ 2 ] = rtX . p4a4q40pcu [ 2 ] ; rtB .
oawzrd2fap [ 2 ] = rtB . jtngam5jo1 [ 2 ] - rtB . pukllw55sc [ 2 ] ; rtB .
ixt1lyi0uk [ 2 ] = rtP . PIDController1_I * rtB . kv2qrw5w4v [ 2 ] ; rtB .
mwsgemvnjp [ 2 ] = rtX . oawe0mghaa [ 2 ] ; rtB . cw2qkgdy3d [ 2 ] = rtP .
PIDController1_N * rtB . oawzrd2fap [ 2 ] ; rtB . d14tgqcbbt = rtP .
PIDController_D * rtB . jliilwwxof [ 0 ] ; rtB . a1cc0ujdnb = rtB .
d14tgqcbbt - rtB . eb41sd25gs ; rtB . ggo2whvj2q = rtP . PIDController_I *
rtB . jliilwwxof [ 0 ] ; rtB . h5ntalntsc = rtP . PIDController_N * rtB .
a1cc0ujdnb ; rtB . dtorfd0zl5 = rtP . PIDController_P * rtB . jliilwwxof [ 0
] ; rtB . gbvzeqi2ho = ( rtB . dtorfd0zl5 + rtB . mbwrtcrvid ) + rtB .
h5ntalntsc ; rtB . nvgssnwpvd = rtP . PIDController2_D * rtB . jliilwwxof [ 1
] ; rtB . oh5z4jek0e = rtB . nvgssnwpvd - rtB . euqixwxw5j ; rtB . m4ws5llhyu
= rtP . PIDController2_I * rtB . jliilwwxof [ 1 ] ; rtB . ildifdanq4 = rtP .
PIDController2_N * rtB . oh5z4jek0e ; rtB . czz21insjf = rtP .
PIDController2_P * rtB . jliilwwxof [ 1 ] ; rtB . jlh0khwdqp = ( rtB .
czz21insjf + rtB . k0nyghzbp5 ) + rtB . ildifdanq4 ; rtB . h34m1yd3v1 = rtP .
PIDController3_D * rtB . jliilwwxof [ 2 ] ; rtB . awws3phtzw = rtB .
h34m1yd3v1 - rtB . he1z03zbh1 ; rtB . ozljxlqodx = rtP . PIDController3_I *
rtB . jliilwwxof [ 2 ] ; rtB . hjhqehj3x3 = rtP . PIDController3_N * rtB .
awws3phtzw ; rtB . n2wfqwfqmg = rtP . PIDController3_P * rtB . jliilwwxof [ 2
] ; rtB . d32cmmaovb = ( rtB . n2wfqwfqmg + rtB . dpbaerun23 ) + rtB .
hjhqehj3x3 ; tmp = & rtP . Jb [ 0 ] ; C11 = rtB . bw5bw3vp2b [ 0 ] ; C12 =
rtB . bw5bw3vp2b [ 1 ] ; C23 = rtB . bw5bw3vp2b [ 2 ] ; rtB . itrtld5gdv =
0.0 ; rtB . itrtld5gdv += rtP . TransferFcn1_C * rtX . pmg0blojut ; rtB .
g4yplqoljb = 0.0 ; rtB . g4yplqoljb += rtP . TransferFcn1_C_gfroigzrf4 * rtX
. jl3mspj5c3 ; rtB . mef4idopue = 0.0 ; rtB . mef4idopue += rtP .
TransferFcn1_C_ls3sy5taih * rtX . c0el2g45hw ; rtB . jjrfmdioo5 [ 0 ] = rtB .
itrtld5gdv * rtB . bw5bw3vp2b [ 2 ] ; rtB . jjrfmdioo5 [ 1 ] = rtB .
g4yplqoljb * rtB . bw5bw3vp2b [ 0 ] ; rtB . jjrfmdioo5 [ 2 ] = rtB .
mef4idopue * rtB . bw5bw3vp2b [ 1 ] ; rtB . jjrfmdioo5 [ 3 ] = rtB .
g4yplqoljb * rtB . bw5bw3vp2b [ 1 ] ; rtB . jjrfmdioo5 [ 4 ] = rtB .
mef4idopue * rtB . bw5bw3vp2b [ 2 ] ; rtB . jjrfmdioo5 [ 5 ] = rtB .
itrtld5gdv * rtB . bw5bw3vp2b [ 0 ] ; for ( i = 0 ; i < 3 ; i ++ ) { rtB .
pt050oqs33 [ i ] = 0.0 ; rtB . pt050oqs33 [ i ] += tmp [ i ] * C11 ; rtB .
pt050oqs33 [ i ] += tmp [ i + 3 ] * C12 ; rtB . pt050oqs33 [ i ] += tmp [ i +
6 ] * C23 ; rtB . kpkzm3xrw0 [ i ] = rtB . jjrfmdioo5 [ i ] - rtB .
jjrfmdioo5 [ i + 3 ] ; } rtB . jcfxyc04gv = rtB . bw5bw3vp2b [ 0 ] * rtB .
pt050oqs33 [ 1 ] ; rtB . m15qtoxlrb = rtB . bw5bw3vp2b [ 1 ] * rtB .
pt050oqs33 [ 2 ] ; rtB . o3yaxognir = rtB . bw5bw3vp2b [ 2 ] * rtB .
pt050oqs33 [ 0 ] ; rtB . bkerdhlz21 = rtB . bw5bw3vp2b [ 0 ] * rtB .
pt050oqs33 [ 2 ] ; rtB . cnr1sap0ta = rtB . bw5bw3vp2b [ 1 ] * rtB .
pt050oqs33 [ 0 ] ; rtB . c53gqvzui4 = rtB . bw5bw3vp2b [ 2 ] * rtB .
pt050oqs33 [ 1 ] ; rtB . lr2oau0b2h [ 0 ] = rtB . m15qtoxlrb - rtB .
c53gqvzui4 ; rtB . lr2oau0b2h [ 1 ] = rtB . o3yaxognir - rtB . bkerdhlz21 ;
rtB . lr2oau0b2h [ 2 ] = rtB . jcfxyc04gv - rtB . cnr1sap0ta ; if (
ssIsSampleHit ( rtS , 1 , 0 ) ) { assessmentPtrVar = ( void * ) &
assessmentVar ; if ( rtB . kx32hccv01 != 0.0 ) { assessmentVar = 0 ;
rt_SlioAccessorUpdate ( 1 , 3 , rtDW . ed03rf3e4o , ssGetT ( rtS ) ,
assessmentPtrVar ) ; } else { S = rtS ; C11 = ssGetT ( rtS ) ; diag =
CreateDiagnosticAsVoidPtr ( "Simulink:blocks:AssertionAssert" , 2 , 5 ,
"Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Assertion" , 2 ,
C11 ) ; rt_ssSet_slErrMsg ( S , diag ) ; ssSetStopRequested ( rtS , ( int )
ssGetT ( rtS ) ) ; assessmentVar = 1 ; rt_SlioAccessorUpdate ( 1 , 3 , rtDW .
ed03rf3e4o , ssGetT ( rtS ) , assessmentPtrVar ) ; } } rtB . mcpejsoler = 0.0
; rtB . mcpejsoler += rtP . TransferFcn_C * rtX . cknsuq2zlh ; rtB .
po3v4hxd2x = rtP . Km * rtB . mcpejsoler ; rtB . i2uukkt5c5 = 0.0 ; rtB .
i2uukkt5c5 += rtP . TransferFcn_C_dcilw4n42x * rtX . c52k0nbe0q ; rtB .
omvaw4qbz5 = rtP . Km * rtB . i2uukkt5c5 ; rtB . cdf4j2oia5 = 0.0 ; rtB .
cdf4j2oia5 += rtP . TransferFcn_C_cjzp5uwabc * rtX . gntqn5o133 ; rtB .
d1km4p5z5z = rtP . Km * rtB . cdf4j2oia5 ; rtB . fxgwwlskk5 [ 0 ] = rtP . J *
rtB . kpkzm3xrw0 [ 0 ] ; rtB . fxgwwlskk5 [ 1 ] = rtP . J * rtB . kpkzm3xrw0
[ 1 ] ; rtB . fxgwwlskk5 [ 2 ] = rtP . J * rtB . kpkzm3xrw0 [ 2 ] ; rtB .
lwso1yhdt3 [ 0 ] = ( ( rtP . Te - rtB . po3v4hxd2x ) - rtB . lr2oau0b2h [ 0 ]
) - rtB . fxgwwlskk5 [ 0 ] ; rtB . lwso1yhdt3 [ 1 ] = ( ( rtP . Te - rtB .
omvaw4qbz5 ) - rtB . lr2oau0b2h [ 1 ] ) - rtB . fxgwwlskk5 [ 1 ] ; rtB .
lwso1yhdt3 [ 2 ] = ( ( rtP . Te - rtB . d1km4p5z5z ) - rtB . lr2oau0b2h [ 2 ]
) - rtB . fxgwwlskk5 [ 2 ] ; tmp = & rtB . cf14ylhfvy [ 0 ] ; C11 = rtB .
lwso1yhdt3 [ 0 ] ; C12 = rtB . lwso1yhdt3 [ 1 ] ; C23 = rtB . lwso1yhdt3 [ 2
] ; for ( i = 0 ; i < 3 ; i ++ ) { rtB . lzhk1bat0b [ i ] = 0.0 ; rtB .
lzhk1bat0b [ i ] += tmp [ i ] * C11 ; rtB . lzhk1bat0b [ i ] += tmp [ i + 3 ]
* C12 ; rtB . lzhk1bat0b [ i ] += tmp [ i + 6 ] * C23 ; } rtB . f14n0vhlpm =
rtP . Gain_Gain * rtB . puqs4ihhyx ; rtB . lltotnako5 = rtP . Gain1_Gain *
rtB . dxigiaxorc ; rtB . dfclg52ijb = rtP . Gain2_Gain * rtB . mdhpjh2adi ;
rtB . nnyu5bhnzg = rtP . Gain3_Gain * rtB . aybmmyshj1 ; rtB . c13zwqewsd =
rtB . bw5bw3vp2b [ 0 ] * rtB . f14n0vhlpm ; rtB . ct4qzfubwd = rtB .
bw5bw3vp2b [ 1 ] * rtB . lltotnako5 ; rtB . dwcijdvmru = rtB . bw5bw3vp2b [ 1
] * rtB . dfclg52ijb ; rtB . bubyfumtha = rtB . bw5bw3vp2b [ 2 ] * rtB .
lltotnako5 ; rtB . fmcxkih41a = rtB . bw5bw3vp2b [ 2 ] * rtB . dfclg52ijb ;
rtB . hxkjpgzbj0 = rtB . bw5bw3vp2b [ 0 ] * rtB . lltotnako5 ; rtB .
ldgnkshida = rtB . bw5bw3vp2b [ 2 ] * rtB . nnyu5bhnzg ; rtB . n1a0nanns0 =
rtB . bw5bw3vp2b [ 1 ] * rtB . f14n0vhlpm ; rtB . ghzw50stn2 = rtB .
bw5bw3vp2b [ 0 ] * rtB . dfclg52ijb ; rtB . hgqpvrhtn1 = rtB . bw5bw3vp2b [ 1
] * rtB . nnyu5bhnzg ; rtB . jbxhal5qlm = rtB . bw5bw3vp2b [ 2 ] * rtB .
f14n0vhlpm ; rtB . evg2nnzcup = rtB . bw5bw3vp2b [ 0 ] * rtB . nnyu5bhnzg ;
rtB . djenag5lsa = ( rtB . c13zwqewsd - rtB . ct4qzfubwd ) + rtB . fmcxkih41a
; rtB . nxta40z0ko = ( rtB . hxkjpgzbj0 + rtB . n1a0nanns0 ) - rtB .
ldgnkshida ; rtB . pdrmt5tfxg = ( rtB . hgqpvrhtn1 - rtB . ghzw50stn2 ) + rtB
. jbxhal5qlm ; rtB . lgdwswu25a = ( ( 0.0 - rtB . evg2nnzcup ) - rtB .
dwcijdvmru ) - rtB . bubyfumtha ; rtB . lyw51fa2jx = rtP . Kb * rtB .
mef4idopue ; rtB . kir1w2sib3 = rtB . gbvzeqi2ho - rtB . mef4idopue ; rtB .
jcuns1utoi = rtP . PIDController_I_g1oln5gyqa * rtB . kir1w2sib3 ; rtB .
pt22dgyfvj = rtX . j1uzwdz34w ; rtB . kj20n3vogf = rtP .
PIDController_P_jlpxdayd0n * rtB . kir1w2sib3 ; rtB . j1yo03ylgg = rtB .
kj20n3vogf + rtB . pt22dgyfvj ; rtB . mxulceln2p = rtB . j1yo03ylgg - rtB .
lyw51fa2jx ; rtB . nu1y2cd1qq = rtP . Kb * rtB . itrtld5gdv ; rtB .
azerdqlxje = rtB . jlh0khwdqp - rtB . itrtld5gdv ; rtB . ds3hegtm1k = rtP .
PIDController_I_kd2qtfttcs * rtB . azerdqlxje ; rtB . lbrkyipcu1 = rtX .
iiwn0ghbg0 ; rtB . pif2vdtegn = rtP . PIDController_P_c11psn2fmh * rtB .
azerdqlxje ; rtB . jo1d214ytt = rtB . pif2vdtegn + rtB . lbrkyipcu1 ; rtB .
fzqohd2it0 = rtB . jo1d214ytt - rtB . nu1y2cd1qq ; rtB . aqx4tvtzvm = rtP .
Kb * rtB . g4yplqoljb ; rtB . lajbwoy2xm = rtB . d32cmmaovb - rtB .
g4yplqoljb ; rtB . k5axb1m3zv = rtP . PIDController_I_gnsusa3wqb * rtB .
lajbwoy2xm ; rtB . m1cqvkhqk0 = rtX . nvmluflpaj ; rtB . b1a4hrzps4 = rtP .
PIDController_P_er1lkjtlz2 * rtB . lajbwoy2xm ; rtB . hr4yo0kcst = rtB .
b1a4hrzps4 + rtB . m1cqvkhqk0 ; rtB . a1qq2dxnqb = rtB . hr4yo0kcst - rtB .
aqx4tvtzvm ; UNUSED_PARAMETER ( tid ) ; } void MdlOutputsTID2 ( int_T tid ) {
real_T uTmp_idx_0 ; real_T uTmp_idx_1 ; real_T uTmp_idx_2 ; uTmp_idx_0 = rtP
. Jb [ 0 ] ; uTmp_idx_1 = rtP . Jb [ 4 ] ; uTmp_idx_2 = rtP . Jb [ 8 ] ;
uTmp_idx_0 *= uTmp_idx_1 ; uTmp_idx_0 *= uTmp_idx_2 ; rtB . f0jsmwtplm =
uTmp_idx_0 ; uTmp_idx_0 = rtP . Jb [ 0 ] ; uTmp_idx_1 = rtP . Jb [ 5 ] ;
uTmp_idx_2 = rtP . Jb [ 7 ] ; uTmp_idx_0 *= uTmp_idx_1 ; uTmp_idx_0 *=
uTmp_idx_2 ; rtB . nh2m1oytuh = uTmp_idx_0 ; uTmp_idx_0 = rtP . Jb [ 1 ] ;
uTmp_idx_1 = rtP . Jb [ 3 ] ; uTmp_idx_2 = rtP . Jb [ 8 ] ; uTmp_idx_0 *=
uTmp_idx_1 ; uTmp_idx_0 *= uTmp_idx_2 ; rtB . kxgyu2harm = uTmp_idx_0 ;
uTmp_idx_0 = rtP . Jb [ 2 ] ; uTmp_idx_1 = rtP . Jb [ 3 ] ; uTmp_idx_2 = rtP
. Jb [ 7 ] ; uTmp_idx_0 *= uTmp_idx_1 ; uTmp_idx_0 *= uTmp_idx_2 ; rtB .
m0qcww2pvi = uTmp_idx_0 ; uTmp_idx_0 = rtP . Jb [ 1 ] ; uTmp_idx_1 = rtP . Jb
[ 5 ] ; uTmp_idx_2 = rtP . Jb [ 6 ] ; uTmp_idx_0 *= uTmp_idx_1 ; uTmp_idx_0
*= uTmp_idx_2 ; rtB . kmqaciilve = uTmp_idx_0 ; uTmp_idx_0 = rtP . Jb [ 2 ] ;
uTmp_idx_1 = rtP . Jb [ 4 ] ; uTmp_idx_2 = rtP . Jb [ 6 ] ; uTmp_idx_0 *=
uTmp_idx_1 ; uTmp_idx_0 *= uTmp_idx_2 ; rtB . fngth1py0k = uTmp_idx_0 ; rtB .
kx32hccv01 = ( ( ( ( rtB . f0jsmwtplm - rtB . nh2m1oytuh ) - rtB . kxgyu2harm
) + rtB . m0qcww2pvi ) + rtB . kmqaciilve ) - rtB . fngth1py0k ;
rt_invd3x3_snf ( rtP . Jb , rtB . cf14ylhfvy ) ; rtB . og2fpsoerg [ 0 ] = rtP
. Wo [ 0 ] ; rtB . og2fpsoerg [ 1 ] = rtP . Wo [ 1 ] ; rtB . og2fpsoerg [ 2 ]
= rtP . Wo [ 2 ] ; UNUSED_PARAMETER ( tid ) ; } void MdlUpdate ( int_T tid )
{ rtDW . phbdv43v20 = 0 ; UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID2 (
int_T tid ) { UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) {
XDot * _rtXdot ; _rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ; _rtXdot ->
o212iwmhcr = rtB . djenag5lsa ; _rtXdot -> ek4na1z4yp = rtB . nxta40z0ko ;
_rtXdot -> oqdfossbpr = rtB . pdrmt5tfxg ; _rtXdot -> kwhjgsltwm = rtB .
lgdwswu25a ; _rtXdot -> m5042oisqd = rtB . h5ntalntsc ; _rtXdot -> n5o4uxbgia
= rtB . ggo2whvj2q ; _rtXdot -> e2f4sjt1iw = rtB . ildifdanq4 ; _rtXdot ->
chmzyjmfc4 = rtB . m4ws5llhyu ; _rtXdot -> lwnibsigzh = rtB . hjhqehj3x3 ;
_rtXdot -> dea0nf5vx3 = rtB . ozljxlqodx ; _rtXdot -> dcnmiwpca4 [ 0 ] = rtB
. lzhk1bat0b [ 0 ] ; _rtXdot -> p4a4q40pcu [ 0 ] = rtB . cw2qkgdy3d [ 0 ] ;
_rtXdot -> oawe0mghaa [ 0 ] = rtB . ixt1lyi0uk [ 0 ] ; _rtXdot -> dcnmiwpca4
[ 1 ] = rtB . lzhk1bat0b [ 1 ] ; _rtXdot -> p4a4q40pcu [ 1 ] = rtB .
cw2qkgdy3d [ 1 ] ; _rtXdot -> oawe0mghaa [ 1 ] = rtB . ixt1lyi0uk [ 1 ] ;
_rtXdot -> dcnmiwpca4 [ 2 ] = rtB . lzhk1bat0b [ 2 ] ; _rtXdot -> p4a4q40pcu
[ 2 ] = rtB . cw2qkgdy3d [ 2 ] ; _rtXdot -> oawe0mghaa [ 2 ] = rtB .
ixt1lyi0uk [ 2 ] ; _rtXdot -> pmg0blojut = 0.0 ; _rtXdot -> pmg0blojut += rtP
. TransferFcn1_A * rtX . pmg0blojut ; _rtXdot -> pmg0blojut += rtB .
omvaw4qbz5 ; _rtXdot -> jl3mspj5c3 = 0.0 ; _rtXdot -> jl3mspj5c3 += rtP .
TransferFcn1_A_cnuk0h4tar * rtX . jl3mspj5c3 ; _rtXdot -> jl3mspj5c3 += rtB .
d1km4p5z5z ; _rtXdot -> c0el2g45hw = 0.0 ; _rtXdot -> c0el2g45hw += rtP .
TransferFcn1_A_gcfopbqkvy * rtX . c0el2g45hw ; _rtXdot -> c0el2g45hw += rtB .
po3v4hxd2x ; _rtXdot -> cknsuq2zlh = 0.0 ; _rtXdot -> cknsuq2zlh += rtP .
TransferFcn_A * rtX . cknsuq2zlh ; _rtXdot -> cknsuq2zlh += rtB . mxulceln2p
; _rtXdot -> c52k0nbe0q = 0.0 ; _rtXdot -> c52k0nbe0q += rtP .
TransferFcn_A_kbikinkyoh * rtX . c52k0nbe0q ; _rtXdot -> c52k0nbe0q += rtB .
fzqohd2it0 ; _rtXdot -> gntqn5o133 = 0.0 ; _rtXdot -> gntqn5o133 += rtP .
TransferFcn_A_ljiqx5nlbh * rtX . gntqn5o133 ; _rtXdot -> gntqn5o133 += rtB .
a1qq2dxnqb ; _rtXdot -> j1uzwdz34w = rtB . jcuns1utoi ; _rtXdot -> iiwn0ghbg0
= rtB . ds3hegtm1k ; _rtXdot -> nvmluflpaj = rtB . k5axb1m3zv ; } void
MdlProjection ( void ) { } void MdlTerminate ( void ) { rtDW . ed03rf3e4o =
rt_SlioAccessorRelease ( 1 , 1 , rtDW . ed03rf3e4o ) ; { if ( rtDW .
n5ugrmujkv . AQHandles ) { sdiTerminateStreaming ( & rtDW . n5ugrmujkv .
AQHandles ) ; } if ( rtDW . n5ugrmujkv . SlioLTF ) {
rtwDestructAccessorPointer ( rtDW . n5ugrmujkv . SlioLTF ) ; } } if (
rt_slioCatalogue ( ) != ( NULL ) ) { void * * slioCatalogueAddr =
rt_slioCatalogueAddr ( ) ; rtwSaveDatasetsToMatFile (
rtwGetPointerFromUniquePtr ( rt_slioCatalogue ( ) ) ,
rt_GetMatSigstreamLoggingFileName ( ) ) ; rtwTerminateSlioCatalogue (
slioCatalogueAddr ) ; * slioCatalogueAddr = NULL ; } } void
MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS , 28 ) ;
ssSetNumPeriodicContStates ( rtS , 0 ) ; ssSetNumY ( rtS , 3 ) ; ssSetNumU (
rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ; ssSetNumSampleTimes ( rtS ,
2 ) ; ssSetNumBlocks ( rtS , 122 ) ; ssSetNumBlockIO ( rtS , 111 ) ;
ssSetNumBlockParams ( rtS , 74 ) ; } void MdlInitializeSampleTimes ( void ) {
ssSetSampleTime ( rtS , 0 , 0.0 ) ; ssSetSampleTime ( rtS , 1 , 0.0 ) ;
ssSetOffsetTime ( rtS , 0 , 0.0 ) ; ssSetOffsetTime ( rtS , 1 , 1.0 ) ; }
void raccel_set_checksum ( ) { ssSetChecksumVal ( rtS , 0 , 150730323U ) ;
ssSetChecksumVal ( rtS , 1 , 1743980095U ) ; ssSetChecksumVal ( rtS , 2 ,
3800048342U ) ; ssSetChecksumVal ( rtS , 3 , 1027532726U ) ; }
#if defined(_MSC_VER)
#pragma optimize( "", off )
#endif
SimStruct * raccel_register_model ( void ) { static struct _ssMdlInfo mdlInfo
; ( void ) memset ( ( char * ) rtS , 0 , sizeof ( SimStruct ) ) ; ( void )
memset ( ( char * ) & mdlInfo , 0 , sizeof ( struct _ssMdlInfo ) ) ;
ssSetMdlInfoPtr ( rtS , & mdlInfo ) ; { static time_T mdlPeriod [
NSAMPLE_TIMES ] ; static time_T mdlOffset [ NSAMPLE_TIMES ] ; static time_T
mdlTaskTimes [ NSAMPLE_TIMES ] ; static int_T mdlTsMap [ NSAMPLE_TIMES ] ;
static int_T mdlSampleHits [ NSAMPLE_TIMES ] ; static boolean_T
mdlTNextWasAdjustedPtr [ NSAMPLE_TIMES ] ; static int_T mdlPerTaskSampleHits
[ NSAMPLE_TIMES * NSAMPLE_TIMES ] ; static time_T mdlTimeOfNextSampleHit [
NSAMPLE_TIMES ] ; { int_T i ; for ( i = 0 ; i < NSAMPLE_TIMES ; i ++ ) {
mdlPeriod [ i ] = 0.0 ; mdlOffset [ i ] = 0.0 ; mdlTaskTimes [ i ] = 0.0 ;
mdlTsMap [ i ] = i ; mdlSampleHits [ i ] = 1 ; } } ssSetSampleTimePtr ( rtS ,
& mdlPeriod [ 0 ] ) ; ssSetOffsetTimePtr ( rtS , & mdlOffset [ 0 ] ) ;
ssSetSampleTimeTaskIDPtr ( rtS , & mdlTsMap [ 0 ] ) ; ssSetTPtr ( rtS , &
mdlTaskTimes [ 0 ] ) ; ssSetSampleHitPtr ( rtS , & mdlSampleHits [ 0 ] ) ;
ssSetTNextWasAdjustedPtr ( rtS , & mdlTNextWasAdjustedPtr [ 0 ] ) ;
ssSetPerTaskSampleHitsPtr ( rtS , & mdlPerTaskSampleHits [ 0 ] ) ;
ssSetTimeOfNextSampleHitPtr ( rtS , & mdlTimeOfNextSampleHit [ 0 ] ) ; }
ssSetSolverMode ( rtS , SOLVER_MODE_SINGLETASKING ) ; { ssSetBlockIO ( rtS ,
( ( void * ) & rtB ) ) ; ( void ) memset ( ( ( void * ) & rtB ) , 0 , sizeof
( B ) ) ; } { ssSetY ( rtS , & rtY ) ; ( void ) memset ( & rtY . isfgwtajws [
0 ] , 0 , 3U * sizeof ( real_T ) ) ; } { real_T * x = ( real_T * ) & rtX ;
ssSetContStates ( rtS , x ) ; ( void ) memset ( ( void * ) x , 0 , sizeof ( X
) ) ; } { void * dwork = ( void * ) & rtDW ; ssSetRootDWork ( rtS , dwork ) ;
( void ) memset ( dwork , 0 , sizeof ( DW ) ) ; } { static DataTypeTransInfo
dtInfo ; ( void ) memset ( ( char_T * ) & dtInfo , 0 , sizeof ( dtInfo ) ) ;
ssSetModelMappingInfo ( rtS , & dtInfo ) ; dtInfo . numDataTypes = 14 ;
dtInfo . dataTypeSizes = & rtDataTypeSizes [ 0 ] ; dtInfo . dataTypeNames = &
rtDataTypeNames [ 0 ] ; dtInfo . BTransTable = & rtBTransTable ; dtInfo .
PTransTable = & rtPTransTable ; dtInfo . dataTypeInfoTable =
rtDataTypeInfoTable ; } Simulacao_CubeSat_2U_InitializeDataMapInfo ( ) ;
ssSetIsRapidAcceleratorActive ( rtS , true ) ; ssSetRootSS ( rtS , rtS ) ;
ssSetVersion ( rtS , SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS ,
"Simulacao_CubeSat_2U" ) ; ssSetPath ( rtS , "Simulacao_CubeSat_2U" ) ;
ssSetTStart ( rtS , 0.0 ) ; ssSetTFinal ( rtS , 30.0 ) ; { static RTWLogInfo
rt_DataLoggingInfo ; rt_DataLoggingInfo . loggingInterval = NULL ;
ssSetRTWLogInfo ( rtS , & rt_DataLoggingInfo ) ; } { { static int_T
rt_LoggedStateWidths [ ] = { 3 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 3 ,
3 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 } ; static int_T
rt_LoggedStateNumDimensions [ ] = { 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1
, 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 } ; static int_T
rt_LoggedStateDimensions [ ] = { 3 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 ,
3 , 3 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 } ; static boolean_T
rt_LoggedStateIsVarDims [ ] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } ; static BuiltInDTypeId
rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE , SS_DOUBLE ,
SS_DOUBLE } ; static int_T rt_LoggedStateComplexSignals [ ] = { 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } ;
static RTWPreprocessingFcnPtr rt_LoggingStatePreprocessingFcnPtrs [ ] = { (
NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , (
NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , (
NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , ( NULL ) , (
NULL ) } ; static const char_T * rt_LoggedStateLabels [ ] = { "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" ,
"CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" , "CSTATE" }
; static const char_T * rt_LoggedStateBlockNames [ ] = {
"Simulacao_CubeSat_2U/CubeSat Dynamics/Integrator" ,
"Simulacao_CubeSat_2U/Kinematics/q0" , "Simulacao_CubeSat_2U/Kinematics/q1" ,
"Simulacao_CubeSat_2U/Kinematics/q2" , "Simulacao_CubeSat_2U/Kinematics/q3" ,
"Simulacao_CubeSat_2U/Control/PID Controller/Filter/Cont. Filter/Filter" ,
"Simulacao_CubeSat_2U/Control/PID Controller/Integrator/Continuous/Integrator"
, "Simulacao_CubeSat_2U/Control/PID Controller2/Filter/Cont. Filter/Filter" ,
"Simulacao_CubeSat_2U/Control/PID Controller2/Integrator/Continuous/Integrator"
, "Simulacao_CubeSat_2U/Control/PID Controller3/Filter/Cont. Filter/Filter" ,
"Simulacao_CubeSat_2U/Control/PID Controller3/Integrator/Continuous/Integrator"
, "Simulacao_CubeSat_2U/Control/PID Controller1/Filter/Cont. Filter/Filter" ,
"Simulacao_CubeSat_2U/Control/PID Controller1/Integrator/Continuous/Integrator"
, "Simulacao_CubeSat_2U/Reaction Wheel Y/Transfer Fcn1" ,
"Simulacao_CubeSat_2U/Reaction Wheel Z/Transfer Fcn1" ,
"Simulacao_CubeSat_2U/Reaction Wheel X/Transfer Fcn1" ,
"Simulacao_CubeSat_2U/Reaction Wheel X/Transfer Fcn" ,
"Simulacao_CubeSat_2U/Reaction Wheel Y/Transfer Fcn" ,
"Simulacao_CubeSat_2U/Reaction Wheel Z/Transfer Fcn" ,
 "Simulacao_CubeSat_2U/Reaction Wheel X/PID Controller/Integrator/Continuous/Integrator"
,
 "Simulacao_CubeSat_2U/Reaction Wheel Y/PID Controller/Integrator/Continuous/Integrator"
,
 "Simulacao_CubeSat_2U/Reaction Wheel Z/PID Controller/Integrator/Continuous/Integrator"
} ; static const char_T * rt_LoggedStateNames [ ] = { "" , "" , "" , "" , ""
, "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" , "" ,
"" , "" } ; static boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 , 0 , 0
, 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 } ;
static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE
, SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 ,
0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 ,
SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } } ; static RTWLogSignalInfo rt_LoggedStateSignalInfo =
{ 22 , rt_LoggedStateWidths , rt_LoggedStateNumDimensions ,
rt_LoggedStateDimensions , rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) ,
rt_LoggedStateDataTypeIds , rt_LoggedStateComplexSignals , ( NULL ) ,
rt_LoggingStatePreprocessingFcnPtrs , { rt_LoggedStateLabels } , ( NULL ) , (
NULL ) , ( NULL ) , { rt_LoggedStateBlockNames } , { rt_LoggedStateNames } ,
rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert } ; static void *
rt_LoggedStateSignalPtrs [ 22 ] ; rtliSetLogXSignalPtrs ( ssGetRTWLogInfo (
rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . dcnmiwpca4 [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . o212iwmhcr ;
rt_LoggedStateSignalPtrs [ 2 ] = ( void * ) & rtX . ek4na1z4yp ;
rt_LoggedStateSignalPtrs [ 3 ] = ( void * ) & rtX . oqdfossbpr ;
rt_LoggedStateSignalPtrs [ 4 ] = ( void * ) & rtX . kwhjgsltwm ;
rt_LoggedStateSignalPtrs [ 5 ] = ( void * ) & rtX . m5042oisqd ;
rt_LoggedStateSignalPtrs [ 6 ] = ( void * ) & rtX . n5o4uxbgia ;
rt_LoggedStateSignalPtrs [ 7 ] = ( void * ) & rtX . e2f4sjt1iw ;
rt_LoggedStateSignalPtrs [ 8 ] = ( void * ) & rtX . chmzyjmfc4 ;
rt_LoggedStateSignalPtrs [ 9 ] = ( void * ) & rtX . lwnibsigzh ;
rt_LoggedStateSignalPtrs [ 10 ] = ( void * ) & rtX . dea0nf5vx3 ;
rt_LoggedStateSignalPtrs [ 11 ] = ( void * ) & rtX . p4a4q40pcu [ 0 ] ;
rt_LoggedStateSignalPtrs [ 12 ] = ( void * ) & rtX . oawe0mghaa [ 0 ] ;
rt_LoggedStateSignalPtrs [ 13 ] = ( void * ) & rtX . pmg0blojut ;
rt_LoggedStateSignalPtrs [ 14 ] = ( void * ) & rtX . jl3mspj5c3 ;
rt_LoggedStateSignalPtrs [ 15 ] = ( void * ) & rtX . c0el2g45hw ;
rt_LoggedStateSignalPtrs [ 16 ] = ( void * ) & rtX . cknsuq2zlh ;
rt_LoggedStateSignalPtrs [ 17 ] = ( void * ) & rtX . c52k0nbe0q ;
rt_LoggedStateSignalPtrs [ 18 ] = ( void * ) & rtX . gntqn5o133 ;
rt_LoggedStateSignalPtrs [ 19 ] = ( void * ) & rtX . j1uzwdz34w ;
rt_LoggedStateSignalPtrs [ 20 ] = ( void * ) & rtX . iiwn0ghbg0 ;
rt_LoggedStateSignalPtrs [ 21 ] = ( void * ) & rtX . nvmluflpaj ; }
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "" ) ; rtliSetLogXFinal ( ssGetRTWLogInfo ( rtS ) ,
"" ) ; rtliSetLogVarNameModifier ( ssGetRTWLogInfo ( rtS ) , "none" ) ;
rtliSetLogFormat ( ssGetRTWLogInfo ( rtS ) , 4 ) ; rtliSetLogMaxRows (
ssGetRTWLogInfo ( rtS ) , 0 ) ; rtliSetLogDecimation ( ssGetRTWLogInfo ( rtS
) , 1 ) ; { static void * rt_LoggedOutputSignalPtrs [ ] = { & rtY .
isfgwtajws [ 0 ] } ; rtliSetLogYSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( (
LogSignalPtrsType ) rt_LoggedOutputSignalPtrs ) ) ; } { static int_T
rt_LoggedOutputWidths [ ] = { 3 } ; static int_T rt_LoggedOutputNumDimensions
[ ] = { 2 } ; static int_T rt_LoggedOutputDimensions [ ] = { 3 , 1 } ; static
boolean_T rt_LoggedOutputIsVarDims [ ] = { 0 } ; static void *
rt_LoggedCurrentSignalDimensions [ ] = { ( NULL ) , ( NULL ) } ; static int_T
rt_LoggedCurrentSignalDimensionsSize [ ] = { 4 , 4 } ; static BuiltInDTypeId
rt_LoggedOutputDataTypeIds [ ] = { SS_DOUBLE } ; static int_T
rt_LoggedOutputComplexSignals [ ] = { 0 } ; static RTWPreprocessingFcnPtr
rt_LoggingPreprocessingFcnPtrs [ ] = { ( NULL ) } ; static const char_T *
rt_LoggedOutputLabels [ ] = { "" } ; static const char_T *
rt_LoggedOutputBlockNames [ ] = {
"Simulacao_CubeSat_2U/Velocidade Angular CubeSat" } ; static
RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } } ; static RTWLogSignalInfo
rt_LoggedOutputSignalInfo [ ] = { { 1 , rt_LoggedOutputWidths ,
rt_LoggedOutputNumDimensions , rt_LoggedOutputDimensions ,
rt_LoggedOutputIsVarDims , rt_LoggedCurrentSignalDimensions ,
rt_LoggedCurrentSignalDimensionsSize , rt_LoggedOutputDataTypeIds ,
rt_LoggedOutputComplexSignals , ( NULL ) , rt_LoggingPreprocessingFcnPtrs , {
rt_LoggedOutputLabels } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedOutputBlockNames } , { ( NULL ) } , ( NULL ) ,
rt_RTWLogDataTypeConvert } } ; rtliSetLogYSignalInfo ( ssGetRTWLogInfo ( rtS
) , rt_LoggedOutputSignalInfo ) ; rt_LoggedCurrentSignalDimensions [ 0 ] = &
rt_LoggedOutputWidths [ 0 ] ; rt_LoggedCurrentSignalDimensions [ 1 ] = &
rt_LoggedOutputWidths [ 0 ] ; } rtliSetLogY ( ssGetRTWLogInfo ( rtS ) ,
"yout" ) ; } { static struct _ssStatesInfo2 statesInfo2 ; ssSetStatesInfo2 (
rtS , & statesInfo2 ) ; } { static ssPeriodicStatesInfo periodicStatesInfo ;
ssSetPeriodicStatesInfo ( rtS , & periodicStatesInfo ) ; } { static
ssJacobianPerturbationBounds jacobianPerturbationBounds ;
ssSetJacobianPerturbationBounds ( rtS , & jacobianPerturbationBounds ) ; } {
static ssSolverInfo slvrInfo ; static boolean_T contStatesDisabled [ 28 ] ;
static real_T absTol [ 28 ] = { 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 ,
1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 ,
1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 ,
1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 , 1.0E-8 } ; static
uint8_T absTolControl [ 28 ] = { 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U ,
0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U
, 0U , 0U , 0U } ; static real_T contStateJacPerturbBoundMinVec [ 28 ] ;
static real_T contStateJacPerturbBoundMaxVec [ 28 ] ; { int i ; for ( i = 0 ;
i < 28 ; ++ i ) { contStateJacPerturbBoundMinVec [ i ] = 0 ;
contStateJacPerturbBoundMaxVec [ i ] = rtGetInf ( ) ; } } ssSetSolverRelTol (
rtS , 1.0E-5 ) ; ssSetStepSize ( rtS , 0.0001 ) ; ssSetMinStepSize ( rtS ,
1.0E-9 ) ; ssSetMaxNumMinSteps ( rtS , - 1 ) ; ssSetMinStepViolatedError (
rtS , 0 ) ; ssSetMaxStepSize ( rtS , 0.0001 ) ; ssSetSolverMaxOrder ( rtS , -
1 ) ; ssSetSolverRefineFactor ( rtS , 1 ) ; ssSetOutputTimes ( rtS , ( NULL )
) ; ssSetNumOutputTimes ( rtS , 0 ) ; ssSetOutputTimesOnly ( rtS , 0 ) ;
ssSetOutputTimesIndex ( rtS , 0 ) ; ssSetZCCacheNeedsReset ( rtS , 0 ) ;
ssSetDerivCacheNeedsReset ( rtS , 0 ) ; ssSetNumNonContDerivSigInfos ( rtS ,
0 ) ; ssSetNonContDerivSigInfos ( rtS , ( NULL ) ) ; ssSetSolverInfo ( rtS ,
& slvrInfo ) ; ssSetSolverName ( rtS , "ode23" ) ; ssSetVariableStepSolver (
rtS , 1 ) ; ssSetSolverConsistencyChecking ( rtS , 0 ) ;
ssSetSolverAdaptiveZcDetection ( rtS , 0 ) ; ssSetSolverRobustResetMethod (
rtS , 0 ) ; ssSetAbsTolVector ( rtS , absTol ) ; ssSetAbsTolControlVector (
rtS , absTolControl ) ; ssSetSolverAbsTol_Obsolete ( rtS , absTol ) ;
ssSetSolverAbsTolControl_Obsolete ( rtS , absTolControl ) ;
ssSetJacobianPerturbationBoundsMinVec ( rtS , contStateJacPerturbBoundMinVec
) ; ssSetJacobianPerturbationBoundsMaxVec ( rtS ,
contStateJacPerturbBoundMaxVec ) ; ssSetSolverStateProjection ( rtS , 0 ) ;
ssSetSolverMassMatrixType ( rtS , ( ssMatrixType ) 0 ) ;
ssSetSolverMassMatrixNzMax ( rtS , 0 ) ; ssSetModelOutputs ( rtS , MdlOutputs
) ; ssSetModelLogData ( rtS , rt_UpdateTXYLogVars ) ;
ssSetModelLogDataIfInInterval ( rtS , rt_UpdateTXXFYLogVars ) ;
ssSetModelUpdate ( rtS , MdlUpdate ) ; ssSetModelDerivatives ( rtS ,
MdlDerivatives ) ; ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ;
ssSetSolverShapePreserveControl ( rtS , 2 ) ; ssSetTNextTid ( rtS , INT_MIN )
; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ;
ssSetNumNonsampledZCs ( rtS , 0 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ; }
ssSetChecksumVal ( rtS , 0 , 150730323U ) ; ssSetChecksumVal ( rtS , 1 ,
1743980095U ) ; ssSetChecksumVal ( rtS , 2 , 3800048342U ) ; ssSetChecksumVal
( rtS , 3 , 1027532726U ) ; { static const sysRanDType rtAlwaysEnabled =
SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo rt_ExtModeInfo ; static const
sysRanDType * systemRan [ 2 ] ; gblRTWExtModeInfo = & rt_ExtModeInfo ;
ssSetRTWExtModeInfo ( rtS , & rt_ExtModeInfo ) ;
rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo , systemRan ) ;
systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = & rtAlwaysEnabled ;
rteiSetModelMappingInfoPtr ( ssGetRTWExtModeInfo ( rtS ) , &
ssGetModelMappingInfo ( rtS ) ) ; rteiSetChecksumsPtr ( ssGetRTWExtModeInfo (
rtS ) , ssGetChecksums ( rtS ) ) ; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS )
, ssGetTPtr ( rtS ) ) ; } return rtS ; }
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif
const int_T gblParameterTuningTid = 2 ; void MdlOutputsParameterSampleTime (
int_T tid ) { MdlOutputsTID2 ( tid ) ; }
