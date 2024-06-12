# Scan Context
This library provides a simple and light weight implementation of the ScanContext descriptor and place recognition database originally presented in [1]. Specifically, this library provides functionality to 1) Compute ScanContext descriptors from LiDAR Scans and 2) maintain and query a database of descriptors for LiDAR place recognition.

[1] G. Kim and A. Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition Within 3D Point Cloud Map," 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018, pp. 4802-4809,

![Example ScanContext 1](./media/newer_college_example_1.png)
![Example ScanContext 2](./media/newer_college_example_2.png)
![Example ScanContext 3](./media/newer_college_example_3.png)
*1) LiDAR Scans overlaid with descriptor in polar space, 2)ScanContext Descriptor, 3) RingKey*

# Documentation

#### Project Structure
* `include/scan_context/` - Contains the all implementation
  * `scan_context*` - Provides implementation to compute and store ScanContext descriptors.
  * `database*` - Provides implementation of incremental database for LiDAR place recognition with ScanContext.
* `python/` - Contains definitions for python wrappers of the modules above.
* `tests/` - Provides unit tests to validate the implementation.

Code is currently documented with in-line comments (doxygen coming soon!). For example usage please see the unit tests in `tests/`

[TODO] Document Install

[TODO] Document independence to other libraries

