rtw_config = coder.config('LIB');
rtw_config.TargetLang = 'C';
rtw_config.CodeReplacementLibrary = 'C89/C90 (ANSI)';
rtw_config.CCompilerOptimization = 'On';

codegen -config rtw_config -v quadrotorPropulsion motorspeed -o quadrotorPropulsion
codegen -config rtw_config -v quadrotorDrag -o quadrotorDrag

