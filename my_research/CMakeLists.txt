# 1. 告诉编译器去哪里找头文件
INCLUDE_DIRECTORIES(
        /usr/include/eigen3
        ${PROJECT_SOURCE_DIR}/inc
        ${PROJECT_SOURCE_DIR}/inc/include
)

# 2. 告诉链接器去哪里找底层的预制件库
LINK_DIRECTORIES(
        /usr/lib/x86_64-linux-gnu
        ${PROJECT_SOURCE_DIR}/lib
)

# 3. 你的算法代码名字（不要加 .cpp）
set(MY_TARGETS
        data_recorder
        get_states
        hello
        zero_gravity
        )

# 4. 自动化编译与链接流水线
foreach (target ${MY_TARGETS})
    add_executable(${target} ${target}.cpp)
    TARGET_LINK_LIBRARIES(${target}
            RCI_Client.a
            xMateModel.a
            orocos-kdl.a
            glog.a
            pthread
            )
endforeach ()