@echo off
set MATLAB=C:\Program Files\Polyspace\R2020a
"%MATLAB%\bin\win64\gmake" -f Simulacao_CubeSat_2U.mk  RSIM_SOLVER_SELECTION=2 PCMATLABROOT="C:\\Program Files\\Polyspace\\R2020a" EXTMODE_STATIC_ALLOC=0 EXTMODE_STATIC_ALLOC_SIZE=1000000 EXTMODE_TRANSPORT=0 TMW_EXTMODE_TESTING=0 RSIM_PARAMETER_LOADING=1 OPTS="-DTGTCONN -DNRT -DRSIM_PARAMETER_LOADING -DRSIM_WITH_SL_SOLVER -DENABLE_SLEXEC_SSBRIDGE=1 -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0 -DON_TARGET_WAIT_FOR_START=0 -DTID01EQ=0"
