ADD_EXECUTABLE(test test.c)
INcLUDE_DIRECTORIES(/usr/include/hello)
TARGET_LINK_LIBRARIES(test libhello.so)
#TARGET_LINK_LIBRARIES(test libhello.a)
