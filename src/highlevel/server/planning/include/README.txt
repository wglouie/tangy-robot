In order to use header the files in other packages, the files must be in the folder include/<package_name>/<hearder_file>.h

In the CMakeList of the package that will use the header files include the following
find_package(catkin REQUIRED COMPONENTS <package_name> ...)

