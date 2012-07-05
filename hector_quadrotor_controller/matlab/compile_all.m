rtw_config = emlcoder.RTWConfig;
rtw_config.TargetLang = 'C';
rtw_config.TargetFunctionLibrary = 'C89/C90 (ANSI)';
rtw_config.RTWCompilerOptimization = 'On';
rtw_config.GenerateMakefile = true;

emlc -s rtw_config -v -report quadrotorPropulsion motorspeed -o quadrotorPropulsion% -I 'emcprj\rtwlib\motorspeed\'

emlc -s rtw_config -v -report quadrotorDrag -o quadrotorDrag

%{
/*
** main.c
*/
#include <stdio.h>
#include <stdlib.h>

int main()
{
    quadrotorPropulsion_initialize();
    
    printf("quadrotorPropulsion=%g\n", quadrotorPropulsion());
    
    quadrotorPropulsion_terminate();
    
    return 0;
}
%}