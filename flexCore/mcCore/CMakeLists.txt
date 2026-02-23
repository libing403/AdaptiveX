

add_library(mcCore STATIC "")
#aux_source_directory(. mcCore_src)
message(STATUS "mcCore source file :${mcCore_src}")
if(MSVC)
    set_property(TARGET mcCore PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()
target_sources(mcCore PRIVATE
    #算法模块
    MathLib.c
    onlinetrapeprofile.c
    platformCfg.c
    Queue.c
    velocityPlanning.c
    Interpolation.c
    #单轴状态机
    axisBaseState.c
    axisContinusMotionState.c
    axisControl.c
    axisDisableState.c
    axisDiscreteMotionState.c
    axisErrorStopState.c
    axisHomingState.c
    axisStoppingState.c
    axisStandstillState.c
    axisSyncMotiondState.c
    
    #轴组状态机
    groupBaseState.c
    groupControl.c
    groupDisableState.c
    groupErrorStopState.c
    groupHomingState.c
    groupMovingState.c
    groupStandbyState.c
    groupStoppingState.c

    #算法集成
    mcCore.c
    #日志
     ../3rdParty/ulog/src/ulog.c

 )
target_include_directories(mcCore PUBLIC 
../3rdParty/ulog/src
../
./
${CMAKE_CURRENT_SOURCE_DIR}

)
