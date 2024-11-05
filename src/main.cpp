#include <ros/ros.h>

#include <exception>
#include <iostream>

#include "ros_wrapper.h"

#include "thread_pool.h"

void main_task() {

}

int main(int argc, char** argv) {
    // Confine single core //
    // 현재(main) 스레드를 특정 CPU 코어에 할당
    // const int cpu_core_index = 0;
    // const int num_max_threads_for_this_cpu =
    //     static_cast<int>(std::thread::hardware_concurrency());

    // if (cpu_core_index >= num_max_threads_for_this_cpu) {
    //     std::printf("Exceed the maximum logical CPU number!");
    //     return -1;
    // }

    // cpu_set_t cpu_set;
    // CPU_ZERO(&cpu_set);
    // CPU_SET(cpu_core_index, &cpu_set);

    // // 현재(main) 스레드에 CPU affinity 적용
    // int result = pthread_setaffinity_np(pthread_self(),
    //                                     sizeof(cpu_set_t), &cpu_set);
    // if (result != 0) {
    //     std::printf("Error calling pthread_setaffinity_np: %d\n", result);
    //     return -1;
    // }

    // std::printf("The main thread is confined to CPU Core [%d]\n", cpu_core_index);

    ros::init(argc, argv, "mapless_moving_node");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Turn on: \"mapless_moving\".\n");

    try {
        std::unique_ptr<ROSWrapper> wrapper;
        wrapper = std::make_unique<ROSWrapper>(nh);
    } catch (std::exception& e) {
        std::cout << " ERROR! the error message is [" << e.what() << std::endl;
    }

    ROS_INFO_STREAM("Turn off: \"mapless_moving\".\n");

    return 0;
}
