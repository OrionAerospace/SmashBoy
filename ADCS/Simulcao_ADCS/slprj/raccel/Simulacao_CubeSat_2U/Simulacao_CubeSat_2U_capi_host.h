#ifndef RTW_HEADER_Simulacao_CubeSat_2U_cap_host_h_
#define RTW_HEADER_Simulacao_CubeSat_2U_cap_host_h_
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"
typedef struct { rtwCAPI_ModelMappingInfo mmi ; }
Simulacao_CubeSat_2U_host_DataMapInfo_T ;
#ifdef __cplusplus
extern "C" {
#endif
void Simulacao_CubeSat_2U_host_InitializeDataMapInfo (
Simulacao_CubeSat_2U_host_DataMapInfo_T * dataMap , const char * path ) ;
#ifdef __cplusplus
}
#endif
#endif
#endif
