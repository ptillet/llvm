set(LLVM_LINK_COMPONENTS all)

add_llvm_example(NVVM nvvm.cpp)

target_link_libraries(NVVM PRIVATE ${LLVM_PTHREAD_LIB})
