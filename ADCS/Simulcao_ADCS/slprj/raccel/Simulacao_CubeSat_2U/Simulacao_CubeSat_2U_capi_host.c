#include "Simulacao_CubeSat_2U_capi_host.h"
static Simulacao_CubeSat_2U_host_DataMapInfo_T root;
static int initialized = 0;
__declspec( dllexport ) rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        Simulacao_CubeSat_2U_host_InitializeDataMapInfo(&(root), "Simulacao_CubeSat_2U");
    }
    return &root.mmi;
}

rtwCAPI_ModelMappingInfo *mexFunction() {return(getRootMappingInfo());}
