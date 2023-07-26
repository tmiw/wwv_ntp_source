add_executable(wwv wwv.cpp filt.cpp)

set(Q_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/../q)
target_include_directories(wwv PRIVATE ${Q_FOLDER}/q_lib/include ${Q_FOLDER}/infra/include)
