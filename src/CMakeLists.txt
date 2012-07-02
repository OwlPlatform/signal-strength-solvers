SET(FingerSources
  fingerprint_solver.cpp
)

add_executable (fingerprint_solver ${FingerSources})
target_link_libraries (fingerprint_solver owl owl-solver pthread)

INSTALL(TARGETS fingerprint_solver RUNTIME DESTINATION bin/owl)
