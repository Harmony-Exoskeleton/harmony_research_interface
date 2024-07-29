gtest_dep = dependency('gtest_main', main: true, required: true)

tests = [
    ['shared_memory_manager_tests', ['shared_memory_manager_tests.cpp']],
    ['size_tests', ['size_tests.cpp']],
    ['joint_states_tests', ['joint_states_tests.cpp']],
    ['research_interface_tests', ['research_interface_tests.cpp']],
    ['arm_controller_tests', ['arm_controller_tests.cpp']],
    # add new tests here in form ['test_name', ['source1.cpp', source2.cpp]],
]

research_lib = cpp.find_library('harmony_research', dirs : libdir) 

foreach test : tests
    exe = executable(test[0], test[1],
        include_directories: inc,
        dependencies: [gtest_dep, research_lib])
    test(test[0], exe)
endforeach
