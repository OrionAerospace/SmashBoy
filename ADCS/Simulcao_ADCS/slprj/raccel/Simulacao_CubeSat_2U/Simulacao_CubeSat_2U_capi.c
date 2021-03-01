#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "Simulacao_CubeSat_2U_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)    
#else
#include "builtin_typeid_types.h"
#include "Simulacao_CubeSat_2U.h"
#include "Simulacao_CubeSat_2U_capi.h"
#include "Simulacao_CubeSat_2U_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST                  
#define TARGET_STRING(s)               (NULL)                    
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 1 , TARGET_STRING (
"Simulacao_CubeSat_2U/Quaternions to Euler ZYX Rotation" ) , TARGET_STRING (
"" ) , 0 , 0 , 0 , 0 , 0 } , { 1 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Velocidade Angular Inicial" ) , TARGET_STRING ( "" ) ,
0 , 0 , 1 , 0 , 1 } , { 2 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/Sum" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 ,
0 } , { 3 , 0 , TARGET_STRING ( "Simulacao_CubeSat_2U/Control/Sum1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 4 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/Integrator" ) , TARGET_STRING ( "" ) ,
0 , 0 , 2 , 0 , 0 } , { 5 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/Matrix Multiply" ) , TARGET_STRING (
"" ) , 0 , 0 , 2 , 0 , 0 } , { 6 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/Matrix Multiply1" ) , TARGET_STRING (
"" ) , 0 , 0 , 2 , 0 , 0 } , { 7 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/Product" ) , TARGET_STRING ( "" ) , 0
, 0 , 2 , 0 , 0 } , { 8 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/Reshape" ) , TARGET_STRING ( "" ) , 0
, 0 , 2 , 0 , 0 } , { 9 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/Minus" ) , TARGET_STRING ( "" ) , 0 ,
0 , 2 , 0 , 0 } , { 10 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Gain" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 ,
0 , 0 } , { 11 , 0 , TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/Gain1"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 12 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Gain2" ) , TARGET_STRING ( "" ) , 0 , 0 , 3
, 0 , 0 } , { 13 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Gain3" ) , TARGET_STRING ( "" ) , 0 , 0 , 3
, 0 , 0 } , { 14 , 0 , TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/q0" )
, TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 15 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/q1" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0
, 0 } , { 16 , 0 , TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/q2" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 17 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/q3" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0
, 0 } , { 18 , 0 , TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/Product"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 19 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product1" ) , TARGET_STRING ( "" ) , 0 , 0 ,
3 , 0 , 0 } , { 20 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product10" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 21 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product11" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 22 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product2" ) , TARGET_STRING ( "" ) , 0 , 0 ,
3 , 0 , 0 } , { 23 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product3" ) , TARGET_STRING ( "" ) , 0 , 0 ,
3 , 0 , 0 } , { 24 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product4" ) , TARGET_STRING ( "" ) , 0 , 0 ,
3 , 0 , 0 } , { 25 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product5" ) , TARGET_STRING ( "" ) , 0 , 0 ,
3 , 0 , 0 } , { 26 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product6" ) , TARGET_STRING ( "" ) , 0 , 0 ,
3 , 0 , 0 } , { 27 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product7" ) , TARGET_STRING ( "" ) , 0 , 0 ,
3 , 0 , 0 } , { 28 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product8" ) , TARGET_STRING ( "" ) , 0 , 0 ,
3 , 0 , 0 } , { 29 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Product9" ) , TARGET_STRING ( "" ) , 0 , 0 ,
3 , 0 , 0 } , { 30 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Sum" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 ,
0 , 0 } , { 31 , 0 , TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/Sum1" )
, TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 32 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Sum2" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 ,
0 , 0 } , { 33 , 0 , TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/Sum3" )
, TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 34 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Gain" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 35 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Gain1" ) , TARGET_STRING ( "" ) , 0 ,
0 , 3 , 0 , 0 } , { 36 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Sum" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 37 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Sum1" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 38 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Transfer Fcn" ) , TARGET_STRING ( "" )
, 0 , 0 , 3 , 0 , 0 } , { 39 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Transfer Fcn1" ) , TARGET_STRING ( ""
) , 0 , 0 , 3 , 0 , 0 } , { 40 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Gain" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 41 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Gain1" ) , TARGET_STRING ( "" ) , 0 ,
0 , 3 , 0 , 0 } , { 42 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Sum" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 43 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Sum1" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 44 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Transfer Fcn" ) , TARGET_STRING ( "" )
, 0 , 0 , 3 , 0 , 0 } , { 45 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Transfer Fcn1" ) , TARGET_STRING ( ""
) , 0 , 0 , 3 , 0 , 0 } , { 46 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Gain" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 47 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Gain1" ) , TARGET_STRING ( "" ) , 0 ,
0 , 3 , 0 , 0 } , { 48 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Sum" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 49 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Sum1" ) , TARGET_STRING ( "" ) , 0 , 0
, 3 , 0 , 0 } , { 50 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Transfer Fcn" ) , TARGET_STRING ( "" )
, 0 , 0 , 3 , 0 , 0 } , { 51 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Transfer Fcn1" ) , TARGET_STRING ( ""
) , 0 , 0 , 3 , 0 , 0 } , { 52 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/3x3 Cross Product/Sum" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 53 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/Cross Product/Element product" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 4 , 0 , 0 } , { 54 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/Cross Product/Add3" ) , TARGET_STRING
( "" ) , 0 , 0 , 2 , 0 , 0 } , { 55 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Product" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 5 , 0 , 1 } , { 56 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/3x3 Cross Product/Subsystem/i x j" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 57 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/3x3 Cross Product/Subsystem/j x k" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 58 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/3x3 Cross Product/Subsystem/k x i" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 59 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/3x3 Cross Product/Subsystem1/i x k" )
, TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 60 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/3x3 Cross Product/Subsystem1/j x i" )
, TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 61 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/CubeSat Dynamics/3x3 Cross Product/Subsystem1/k x j" )
, TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 62 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Determinant of 3x3 Matrix/Product"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 1 } , { 63 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Determinant of 3x3 Matrix/Product1"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 1 } , { 64 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Determinant of 3x3 Matrix/Product2"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 1 } , { 65 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Determinant of 3x3 Matrix/Product3"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 1 } , { 66 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Determinant of 3x3 Matrix/Product4"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 1 } , { 67 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Determinant of 3x3 Matrix/Product5"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 1 } , { 68 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/CubeSat Dynamics/Invert  3x3 Matrix/Determinant of 3x3 Matrix/Sum"
) , TARGET_STRING ( "det(Matrix)" ) , 0 , 0 , 3 , 0 , 1 } , { 69 , 0 ,
TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller/D Gain/Internal Parameters/Derivative Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 70 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller/Filter/Cont. Filter/Filter" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 71 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller/Filter/Cont. Filter/SumD" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 72 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 73 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller/Integrator/Continuous/Integrator"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 74 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller/N Gain/Internal Parameters/Filter Coefficient"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 75 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller/Parallel P Gain/Internal Parameters/Proportional Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 76 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller/Sum/Sum_PID/Sum" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 77 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller1/D Gain/Internal Parameters/Derivative Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 78 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller1/Filter/Cont. Filter/Filter" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 79 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller1/Filter/Cont. Filter/SumD" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 80 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller1/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 81 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller1/Integrator/Continuous/Integrator"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 82 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller1/N Gain/Internal Parameters/Filter Coefficient"
) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 0 } , { 83 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller2/D Gain/Internal Parameters/Derivative Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 84 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2/Filter/Cont. Filter/Filter" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 85 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2/Filter/Cont. Filter/SumD" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 86 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller2/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 87 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2/Integrator/Continuous/Integrator"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 88 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller2/N Gain/Internal Parameters/Filter Coefficient"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 89 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller2/Parallel P Gain/Internal Parameters/Proportional Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 90 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2/Sum/Sum_PID/Sum" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 91 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller3/D Gain/Internal Parameters/Derivative Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 92 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3/Filter/Cont. Filter/Filter" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 93 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3/Filter/Cont. Filter/SumD" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 94 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller3/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 95 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3/Integrator/Continuous/Integrator"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 96 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller3/N Gain/Internal Parameters/Filter Coefficient"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 97 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Control/PID Controller3/Parallel P Gain/Internal Parameters/Proportional Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 98 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3/Sum/Sum_PID/Sum" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 99 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Reaction Wheel X/PID Controller/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 100 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Reaction Wheel X/PID Controller/Integrator/Continuous/Integrator"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 101 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Reaction Wheel X/PID Controller/Parallel P Gain/Internal Parameters/Proportional Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 102 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/PID Controller/Sum/Sum_PI/Sum" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 103 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Reaction Wheel Y/PID Controller/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 104 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Reaction Wheel Y/PID Controller/Integrator/Continuous/Integrator"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 105 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Reaction Wheel Y/PID Controller/Parallel P Gain/Internal Parameters/Proportional Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 106 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/PID Controller/Sum/Sum_PI/Sum" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 107 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Reaction Wheel Z/PID Controller/I Gain/Internal Parameters/Integral Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 108 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Reaction Wheel Z/PID Controller/Integrator/Continuous/Integrator"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 109 , 0 , TARGET_STRING (
 "Simulacao_CubeSat_2U/Reaction Wheel Z/PID Controller/Parallel P Gain/Internal Parameters/Proportional Gain"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 110 , 0 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/PID Controller/Sum/Sum_PI/Sum" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 0 , 0 , ( NULL ) , ( NULL ) ,
0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_BlockParameters
rtBlockParameters [ ] = { { 111 , TARGET_STRING (
"Simulacao_CubeSat_2U/Atitude Desejada Aquisiçao" ) , TARGET_STRING ( "Value"
) , 0 , 1 , 0 } , { 112 , TARGET_STRING (
"Simulacao_CubeSat_2U/Velocidade Angular Detumble" ) , TARGET_STRING (
"Value" ) , 0 , 1 , 0 } , { 113 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller" ) , TARGET_STRING ( "P" ) , 0 ,
3 , 0 } , { 114 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller" ) , TARGET_STRING ( "I" ) , 0 ,
3 , 0 } , { 115 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller" ) , TARGET_STRING ( "D" ) , 0 ,
3 , 0 } , { 116 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller" ) , TARGET_STRING ( "N" ) , 0 ,
3 , 0 } , { 117 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller" ) , TARGET_STRING (
"InitialConditionForIntegrator" ) , 0 , 3 , 0 } , { 118 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller" ) , TARGET_STRING (
"InitialConditionForFilter" ) , 0 , 3 , 0 } , { 119 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller1" ) , TARGET_STRING ( "I" ) , 0
, 3 , 0 } , { 120 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller1" ) , TARGET_STRING ( "D" ) , 0
, 3 , 0 } , { 121 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller1" ) , TARGET_STRING ( "N" ) , 0
, 3 , 0 } , { 122 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller1" ) , TARGET_STRING (
"InitialConditionForIntegrator" ) , 0 , 3 , 0 } , { 123 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller1" ) , TARGET_STRING (
"InitialConditionForFilter" ) , 0 , 3 , 0 } , { 124 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2" ) , TARGET_STRING ( "P" ) , 0
, 3 , 0 } , { 125 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2" ) , TARGET_STRING ( "I" ) , 0
, 3 , 0 } , { 126 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2" ) , TARGET_STRING ( "D" ) , 0
, 3 , 0 } , { 127 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2" ) , TARGET_STRING ( "N" ) , 0
, 3 , 0 } , { 128 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2" ) , TARGET_STRING (
"InitialConditionForIntegrator" ) , 0 , 3 , 0 } , { 129 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller2" ) , TARGET_STRING (
"InitialConditionForFilter" ) , 0 , 3 , 0 } , { 130 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3" ) , TARGET_STRING ( "P" ) , 0
, 3 , 0 } , { 131 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3" ) , TARGET_STRING ( "I" ) , 0
, 3 , 0 } , { 132 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3" ) , TARGET_STRING ( "D" ) , 0
, 3 , 0 } , { 133 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3" ) , TARGET_STRING ( "N" ) , 0
, 3 , 0 } , { 134 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3" ) , TARGET_STRING (
"InitialConditionForIntegrator" ) , 0 , 3 , 0 } , { 135 , TARGET_STRING (
"Simulacao_CubeSat_2U/Control/PID Controller3" ) , TARGET_STRING (
"InitialConditionForFilter" ) , 0 , 3 , 0 } , { 136 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Gain" ) , TARGET_STRING ( "Gain" ) , 0 , 3 ,
0 } , { 137 , TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/Gain1" ) ,
TARGET_STRING ( "Gain" ) , 0 , 3 , 0 } , { 138 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/Gain2" ) , TARGET_STRING ( "Gain" ) , 0 , 3
, 0 } , { 139 , TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/Gain3" ) ,
TARGET_STRING ( "Gain" ) , 0 , 3 , 0 } , { 140 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/q0" ) , TARGET_STRING ( "InitialCondition" )
, 0 , 3 , 0 } , { 141 , TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/q1"
) , TARGET_STRING ( "InitialCondition" ) , 0 , 3 , 0 } , { 142 ,
TARGET_STRING ( "Simulacao_CubeSat_2U/Kinematics/q2" ) , TARGET_STRING (
"InitialCondition" ) , 0 , 3 , 0 } , { 143 , TARGET_STRING (
"Simulacao_CubeSat_2U/Kinematics/q3" ) , TARGET_STRING ( "InitialCondition" )
, 0 , 3 , 0 } , { 144 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/PID Controller" ) , TARGET_STRING (
"P" ) , 0 , 3 , 0 } , { 145 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/PID Controller" ) , TARGET_STRING (
"I" ) , 0 , 3 , 0 } , { 146 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/PID Controller" ) , TARGET_STRING (
"InitialConditionForIntegrator" ) , 0 , 3 , 0 } , { 147 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Transfer Fcn" ) , TARGET_STRING ( "A"
) , 0 , 3 , 0 } , { 148 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Transfer Fcn" ) , TARGET_STRING ( "C"
) , 0 , 3 , 0 } , { 149 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Transfer Fcn1" ) , TARGET_STRING ( "A"
) , 0 , 3 , 0 } , { 150 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel X/Transfer Fcn1" ) , TARGET_STRING ( "C"
) , 0 , 3 , 0 } , { 151 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/PID Controller" ) , TARGET_STRING (
"P" ) , 0 , 3 , 0 } , { 152 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/PID Controller" ) , TARGET_STRING (
"I" ) , 0 , 3 , 0 } , { 153 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/PID Controller" ) , TARGET_STRING (
"InitialConditionForIntegrator" ) , 0 , 3 , 0 } , { 154 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Transfer Fcn" ) , TARGET_STRING ( "A"
) , 0 , 3 , 0 } , { 155 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Transfer Fcn" ) , TARGET_STRING ( "C"
) , 0 , 3 , 0 } , { 156 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Transfer Fcn1" ) , TARGET_STRING ( "A"
) , 0 , 3 , 0 } , { 157 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Y/Transfer Fcn1" ) , TARGET_STRING ( "C"
) , 0 , 3 , 0 } , { 158 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/PID Controller" ) , TARGET_STRING (
"P" ) , 0 , 3 , 0 } , { 159 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/PID Controller" ) , TARGET_STRING (
"I" ) , 0 , 3 , 0 } , { 160 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/PID Controller" ) , TARGET_STRING (
"InitialConditionForIntegrator" ) , 0 , 3 , 0 } , { 161 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Transfer Fcn" ) , TARGET_STRING ( "A"
) , 0 , 3 , 0 } , { 162 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Transfer Fcn" ) , TARGET_STRING ( "C"
) , 0 , 3 , 0 } , { 163 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Transfer Fcn1" ) , TARGET_STRING ( "A"
) , 0 , 3 , 0 } , { 164 , TARGET_STRING (
"Simulacao_CubeSat_2U/Reaction Wheel Z/Transfer Fcn1" ) , TARGET_STRING ( "C"
) , 0 , 3 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 } } ; static const
rtwCAPI_ModelParameters rtModelParameters [ ] = { { 165 , TARGET_STRING ( "J"
) , 0 , 3 , 0 } , { 166 , TARGET_STRING ( "Jb" ) , 0 , 5 , 0 } , { 167 ,
TARGET_STRING ( "Kb" ) , 0 , 3 , 0 } , { 168 , TARGET_STRING ( "Km" ) , 0 , 3
, 0 } , { 169 , TARGET_STRING ( "Te" ) , 0 , 3 , 0 } , { 170 , TARGET_STRING
( "Wo" ) , 0 , 1 , 0 } , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . ouqbmp1b5z [ 0 ] , & rtB .
og2fpsoerg [ 0 ] , & rtB . jliilwwxof [ 0 ] , & rtB . kv2qrw5w4v [ 0 ] , &
rtB . be2k3eehde [ 0 ] , & rtB . pt050oqs33 [ 0 ] , & rtB . lzhk1bat0b [ 0 ]
, & rtB . fxgwwlskk5 [ 0 ] , & rtB . bw5bw3vp2b [ 0 ] , & rtB . lwso1yhdt3 [
0 ] , & rtB . f14n0vhlpm , & rtB . lltotnako5 , & rtB . dfclg52ijb , & rtB .
nnyu5bhnzg , & rtB . aybmmyshj1 , & rtB . mdhpjh2adi , & rtB . dxigiaxorc , &
rtB . puqs4ihhyx , & rtB . c13zwqewsd , & rtB . ct4qzfubwd , & rtB .
dwcijdvmru , & rtB . bubyfumtha , & rtB . fmcxkih41a , & rtB . hxkjpgzbj0 , &
rtB . ldgnkshida , & rtB . n1a0nanns0 , & rtB . ghzw50stn2 , & rtB .
hgqpvrhtn1 , & rtB . jbxhal5qlm , & rtB . evg2nnzcup , & rtB . djenag5lsa , &
rtB . nxta40z0ko , & rtB . pdrmt5tfxg , & rtB . lgdwswu25a , & rtB .
po3v4hxd2x , & rtB . lyw51fa2jx , & rtB . mxulceln2p , & rtB . kir1w2sib3 , &
rtB . mcpejsoler , & rtB . mef4idopue , & rtB . omvaw4qbz5 , & rtB .
nu1y2cd1qq , & rtB . fzqohd2it0 , & rtB . azerdqlxje , & rtB . i2uukkt5c5 , &
rtB . itrtld5gdv , & rtB . d1km4p5z5z , & rtB . aqx4tvtzvm , & rtB .
a1qq2dxnqb , & rtB . lajbwoy2xm , & rtB . cdf4j2oia5 , & rtB . g4yplqoljb , &
rtB . lr2oau0b2h [ 0 ] , & rtB . jjrfmdioo5 [ 0 ] , & rtB . kpkzm3xrw0 [ 0 ]
, & rtB . cf14ylhfvy [ 0 ] , & rtB . jcfxyc04gv , & rtB . m15qtoxlrb , & rtB
. o3yaxognir , & rtB . bkerdhlz21 , & rtB . cnr1sap0ta , & rtB . c53gqvzui4 ,
& rtB . f0jsmwtplm , & rtB . nh2m1oytuh , & rtB . kxgyu2harm , & rtB .
m0qcww2pvi , & rtB . kmqaciilve , & rtB . fngth1py0k , & rtB . kx32hccv01 , &
rtB . d14tgqcbbt , & rtB . eb41sd25gs , & rtB . a1cc0ujdnb , & rtB .
ggo2whvj2q , & rtB . mbwrtcrvid , & rtB . h5ntalntsc , & rtB . dtorfd0zl5 , &
rtB . gbvzeqi2ho , & rtB . jtngam5jo1 [ 0 ] , & rtB . pukllw55sc [ 0 ] , &
rtB . oawzrd2fap [ 0 ] , & rtB . ixt1lyi0uk [ 0 ] , & rtB . mwsgemvnjp [ 0 ]
, & rtB . cw2qkgdy3d [ 0 ] , & rtB . nvgssnwpvd , & rtB . euqixwxw5j , & rtB
. oh5z4jek0e , & rtB . m4ws5llhyu , & rtB . k0nyghzbp5 , & rtB . ildifdanq4 ,
& rtB . czz21insjf , & rtB . jlh0khwdqp , & rtB . h34m1yd3v1 , & rtB .
he1z03zbh1 , & rtB . awws3phtzw , & rtB . ozljxlqodx , & rtB . dpbaerun23 , &
rtB . hjhqehj3x3 , & rtB . n2wfqwfqmg , & rtB . d32cmmaovb , & rtB .
jcuns1utoi , & rtB . pt22dgyfvj , & rtB . kj20n3vogf , & rtB . j1yo03ylgg , &
rtB . ds3hegtm1k , & rtB . lbrkyipcu1 , & rtB . pif2vdtegn , & rtB .
jo1d214ytt , & rtB . k5axb1m3zv , & rtB . m1cqvkhqk0 , & rtB . b1a4hrzps4 , &
rtB . hr4yo0kcst , & rtP . AtitudeDesejadaAquisiao_Value [ 0 ] , & rtP .
VelocidadeAngularDetumble_Value [ 0 ] , & rtP . PIDController_P , & rtP .
PIDController_I , & rtP . PIDController_D , & rtP . PIDController_N , & rtP .
PIDController_InitialConditionForIntegrator , & rtP .
PIDController_InitialConditionForFilter , & rtP . PIDController1_I , & rtP .
PIDController1_D , & rtP . PIDController1_N , & rtP .
PIDController1_InitialConditionForIntegrator , & rtP .
PIDController1_InitialConditionForFilter , & rtP . PIDController2_P , & rtP .
PIDController2_I , & rtP . PIDController2_D , & rtP . PIDController2_N , &
rtP . PIDController2_InitialConditionForIntegrator , & rtP .
PIDController2_InitialConditionForFilter , & rtP . PIDController3_P , & rtP .
PIDController3_I , & rtP . PIDController3_D , & rtP . PIDController3_N , &
rtP . PIDController3_InitialConditionForIntegrator , & rtP .
PIDController3_InitialConditionForFilter , & rtP . Gain_Gain , & rtP .
Gain1_Gain , & rtP . Gain2_Gain , & rtP . Gain3_Gain , & rtP . q0_IC , & rtP
. q1_IC , & rtP . q2_IC , & rtP . q3_IC , & rtP . PIDController_P_jlpxdayd0n
, & rtP . PIDController_I_g1oln5gyqa , & rtP .
PIDController_InitialConditionForIntegrator_nrg5o00kkv , & rtP .
TransferFcn_A , & rtP . TransferFcn_C , & rtP . TransferFcn1_A_gcfopbqkvy , &
rtP . TransferFcn1_C_ls3sy5taih , & rtP . PIDController_P_c11psn2fmh , & rtP
. PIDController_I_kd2qtfttcs , & rtP .
PIDController_InitialConditionForIntegrator_bo4ekjqhoq , & rtP .
TransferFcn_A_kbikinkyoh , & rtP . TransferFcn_C_dcilw4n42x , & rtP .
TransferFcn1_A , & rtP . TransferFcn1_C , & rtP . PIDController_P_er1lkjtlz2
, & rtP . PIDController_I_gnsusa3wqb , & rtP .
PIDController_InitialConditionForIntegrator_ng4h3bnigz , & rtP .
TransferFcn_A_ljiqx5nlbh , & rtP . TransferFcn_C_cjzp5uwabc , & rtP .
TransferFcn1_A_cnuk0h4tar , & rtP . TransferFcn1_C_gfroigzrf4 , & rtP . J , &
rtP . Jb [ 0 ] , & rtP . Kb , & rtP . Km , & rtP . Te , & rtP . Wo [ 0 ] , }
; static int32_T * rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , SS_DOUBLE , 0 , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_MATRIX_COL_MAJOR , 0 , 2 , 0 } , { rtwCAPI_VECTOR , 2 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 2 , 2 , 0 } , { rtwCAPI_SCALAR , 4 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 6 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 8 , 2 ,
0 } } ; static const uint_T rtDimensionArray [ ] = { 1 , 3 , 3 , 1 , 1 , 1 ,
6 , 1 , 3 , 3 } ; static const real_T rtcapiStoredFloats [ ] = { 0.0 } ;
static const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , 0 } , } ; static const rtwCAPI_SampleTimeMap
rtSampleTimeMap [ ] = { { ( const void * ) & rtcapiStoredFloats [ 0 ] , (
const void * ) & rtcapiStoredFloats [ 0 ] , 0 , 0 } , { ( NULL ) , ( NULL ) ,
2 , 0 } } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { {
rtBlockSignals , 111 , ( NULL ) , 0 , ( NULL ) , 0 } , { rtBlockParameters ,
54 , rtModelParameters , 6 } , { ( NULL ) , 0 } , { rtDataTypeMap ,
rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap ,
rtDimensionArray } , "float" , { 150730323U , 1743980095U , 3800048342U ,
1027532726U } , ( NULL ) , 0 , 0 } ; const rtwCAPI_ModelMappingStaticInfo *
Simulacao_CubeSat_2U_GetCAPIStaticMap ( void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void Simulacao_CubeSat_2U_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion
( ( * rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void Simulacao_CubeSat_2U_host_InitializeDataMapInfo (
Simulacao_CubeSat_2U_host_DataMapInfo_T * dataMap , const char * path ) {
rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ; rtwCAPI_SetStaticMap ( dataMap ->
mmi , & mmiStatic ) ; rtwCAPI_SetDataAddressMap ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , NULL ) ; rtwCAPI_SetPath (
dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , NULL ) ;
rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
