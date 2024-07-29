eigen_dep = dependency('eigen3', required: true)
thread_dep = dependency('threads')
research_lib = cpp.find_library('harmony_research', dirs : libdir) 


logger_exe = executable('harmony_logger', 'data_logger.cpp',
    include_directories: inc,
    dependencies: [thread_dep, research_lib]
)

exerciser_exe = executable('harmony_exerciser', 'data_exerciser.cpp',
    include_directories: inc,
    dependencies: [thread_dep, research_lib]
)

value_printer_exe = executable('printValues', 'value_printer.cpp',
    include_directories: inc,
    dependencies: [eigen_dep, research_lib])

controllers_printer_exe = executable('printControllers', 'controller_printer.cpp',
    include_directories: inc,
    dependencies: [eigen_dep, research_lib])

set_controller_exe = executable('setController', 'set_controller.cpp',
    include_directories: inc,
    dependencies: research_lib)

stub_harmony_exe = executable('stubHarmony', 'stub_harmony.cpp',
    include_directories: inc,
    dependencies: research_lib)
