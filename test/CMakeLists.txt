
ADD_EXECUTABLE(base_variable_test base_variable_test.cpp)
TARGET_LINK_LIBRARIES( base_variable_test myslam ${THIRD_PARTY_LIBS})



ADD_EXECUTABLE(base_error_test base_error_test.cpp)
TARGET_LINK_LIBRARIES( base_error_test myslam ${THIRD_PARTY_LIBS})


ADD_EXECUTABLE(vertices_test vertices_test.cpp)
TARGET_LINK_LIBRARIES( vertices_test myslam ${THIRD_PARTY_LIBS})
