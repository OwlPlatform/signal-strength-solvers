SET(FingerSources
  fingerprint_solver.cpp
)

add_executable (fingerprint_solver ${FingerSources})
target_link_libraries (fingerprint_solver owl-common owl-solver pthread)

INSTALL(TARGETS fingerprint_solver RUNTIME DESTINATION bin/owl)


SET(MobilitySources
  mobility_solver.cpp
)

add_executable (mobility_solver ${MobilitySources})
target_link_libraries (mobility_solver owl-common owl-solver pthread)

INSTALL(TARGETS mobility_solver RUNTIME DESTINATION bin/owl)


SET(EvaluatorSources
  localization_evaluator.cpp
)

add_executable (localization_evaluator ${EvaluatorSources})
target_link_libraries (localization_evaluator owl-common owl-solver pthread)

INSTALL(TARGETS localization_evaluator RUNTIME DESTINATION bin/owl)
