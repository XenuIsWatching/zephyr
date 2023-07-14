# Copyright (c) 2023 Meta Platforms
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_CMSIS_NN)

  set(CMSIS_NN_DIR ${ZEPHYR_CURRENT_MODULE_DIR})
  set(cmsis_glue_path ${ZEPHYR_CMSIS_MODULE_DIR})

  zephyr_library()

  zephyr_library_compile_options(-Ofast)

  zephyr_include_directories(
    ${CMSIS_NN_DIR}/Include
    ${cmsis_glue_path}/CMSIS/Core/Include
  )

  if(CONFIG_CMSIS_NN_ACTIVATION)
    file(GLOB SRC "${CMSIS_NN_DIR}/ActivationFunctions/*_s8*.c")
    file(GLOB SRC_S16 "${CMSIS_NN_DIR}/ActivationFunctions/*_s16*.c")
    zephyr_library_sources(${SRC} ${SRC_S16} arm_relu_q7.c arm_relu_q15.c)
  endif()

  if(CONFIG_CMSIS_NN_BASICMATH)
    file(GLOB SRC "${CMSIS_NN_DIR}/BasicMathFunctions/*_*.c")
    zephyr_library_sources(${SRC})
  endif()

  if(CONFIG_CMSIS_NN_CONCATENATION)
    file(GLOB SRC "${CMSIS_NN_DIR}/ConcatenationFunctions/*_*.c")
    zephyr_library_sources(${SRC})
  endif()

  if(CONFIG_CMSIS_NN_CONVOLUTION)
    file(GLOB SRC "${CMSIS_NN_DIR}/ConvolutionFunctions/*_s8*.c")
    file(GLOB SRC_S16 "${CMSIS_NN_DIR}/ConvolutionFunctions/*_s16*.c")
    zephyr_library_sources(${SRC} ${SRC_S16})
  endif()

  if(CONFIG_CMSIS_NN_FULLYCONNECTED)
    file(GLOB SRC "${CMSIS_NN_DIR}/FullyConnectedFunctions/*_s8.c")
    file(GLOB SRC_S16 "${CMSIS_NN_DIR}/FullyConnectedFunctions/*_s16*.c")
    zephyr_library_sources(${SRC} ${SRC_S16})
  endif()

  if(CONFIG_CMSIS_NN_NNSUPPORT)
    file(GLOB SRC "${CMSIS_NN_DIR}/NNSupportFunctions/*_s8*.c")
    file(GLOB SRC_S16 "${CMSIS_NN_DIR}/NNSupportFunctions/*_s16*.c")
    zephyr_library_sources(${SRC} ${SRC_S16} arm_nntables.c
      arm_q7_to_q15_with_offset.c
      arm_s8_to_s16_unordered_with_offset.c)
  endif()

  if(CONFIG_CMSIS_NN_POOLING)
    file(GLOB SRC "${CMSIS_NN_DIR}/PoolingFunctions/*_s8.c")
    file(GLOB SRC_S16 "${CMSIS_NN_DIR}/PoolingFunctions/*_s16.c")
    zephyr_library_sources(${SRC} ${SRC_S16})
  endif()

  if(CONFIG_CMSIS_NN_RESHAPE)
    file(GLOB SRC "${CMSIS_NN_DIR}/ReshapeFunctions/*_*.c")
    zephyr_library_sources(${SRC})
  endif()

  if(CONFIG_CMSIS_NN_SOFTMAX)
    file(GLOB SRC "${CMSIS_NN_DIR}/SoftmaxFunctions/*_s8.c")
    zephyr_library_sources(${SRC} arm_softmax_s8_s16.c
      arm_softmax_s16.c
      arm_nn_softmax_common_s8.c)
  endif()

  if(CONFIG_CMSIS_NN_SVD)
    file(GLOB SRC "${CMSIS_NN_DIR}/SVDFunctions/*_s8.c")
    zephyr_library_sources(${SRC})
  endif()

  if(CONFIG_CMSIS_NN_LSTM)
    file(GLOB SRC "${CMSIS_NN_DIR}/LSTMFunctions/*_s16.c")
    zephyr_library_sources(${SRC})
  endif()

endif()
