if(NOT DEFINED EXE OR NOT DEFINED OUT_DIR)
  message(FATAL_ERROR "V1 determinism test requires EXE and OUT_DIR")
endif()

set(first "${OUT_DIR}/v1-determinism-a.track")
set(second "${OUT_DIR}/v1-determinism-b.track")

execute_process(
  COMMAND "${EXE}" --exporttrack "${first}" 37
  RESULT_VARIABLE first_result
  OUTPUT_QUIET ERROR_VARIABLE first_error)
if(NOT first_result EQUAL 0)
  message(FATAL_ERROR "first V1 export failed: ${first_error}")
endif()

execute_process(
  COMMAND "${EXE}" --exporttrack "${second}" 37
  RESULT_VARIABLE second_result
  OUTPUT_QUIET ERROR_VARIABLE second_error)
if(NOT second_result EQUAL 0)
  message(FATAL_ERROR "second V1 export failed: ${second_error}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E compare_files "${first}" "${second}"
  RESULT_VARIABLE compare_result)
file(REMOVE "${first}" "${second}")
if(NOT compare_result EQUAL 0)
  message(FATAL_ERROR "replaying V1 seed 37 produced different finalized tracks")
endif()
