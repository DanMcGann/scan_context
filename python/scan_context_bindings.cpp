#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include "scan_context/scan_context.h"

// Use short form as it appears alot
namespace py = pybind11;

// Pybind will automatically convert between Eigen <-> Numpy to support numpy format points use use the following type
using ScanContextPybind = ScanContext<Eigen::VectorXd>;

PYBIND11_MODULE(scan_context_python, m) {
  py::module gtsam = py::module::import("numpy");

  py::class_<ScanContextPybind>(m, "ScanContext")
      .def(py::init<const std::vector<Eigen::VectorXd> &, const ScanContextPybind::Params &>(),  // Constructor
           py::arg("lidar_scan"), py::arg("params"))
      .def("descriptor", &ScanContextPybind::descriptor)  // const descriptor access. outputs numpy matrix
      .def("ringKey", &ScanContextPybind::ringKey)        // const ringkey access. outputs numpy vector
      .def("distance", &ScanContextPybind::distance,      // distance computation
           py::arg("other"))
      .def("ringKeyDistance", &ScanContextPybind::ringKeyDistance,  // ring key distance
           py::arg("other"));

  py::class_<ScanContextPybind::Params>(m, "ScanContextParams")
      .def(py::init<>())
      .def_readwrite("number_sectors", &ScanContextPybind::Params::number_sectors)
      .def_readwrite("ringKey", &ScanContextPybind::Params::number_rings)
      .def_readwrite("max_range", &ScanContextPybind::Params::max_range)
      .def("equals", &ScanContextPybind::Params::equals, py::arg("other"));
}