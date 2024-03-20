#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include "scan_context/database.h"
#include "scan_context/scan_context.h"

// Use short form as it appears alot
namespace py = pybind11;

// Pybind will automatically convert between Eigen <-> Numpy to support numpy format points use use the following type
using ScanContextPybind = ScanContext<Eigen::VectorXd>;
using ScanContextDatabasePybind = ScanContextDatabase<Eigen::VectorXd>;

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
      .def_readwrite("number_rings", &ScanContextPybind::Params::number_rings)
      .def_readwrite("max_range", &ScanContextPybind::Params::max_range)
      .def("equals", &ScanContextPybind::Params::equals, py::arg("other"));

  py::class_<ScanContextDatabasePybind>(m, "ScanContextDatabase")
      .def(py::init<const ScanContextDatabasePybind::Params &>(), py::arg("params"))
      .def("insert", &ScanContextDatabasePybind::insert, py::arg("key"), py::arg("scan_context"))
      .def("remove", &ScanContextDatabasePybind::remove, py::arg("key"))
      .def("query", &ScanContextDatabasePybind::query, py::arg("query"))
      .def("queryK", &ScanContextDatabasePybind::queryK, py::arg("query"), py::arg("k"));

  py::class_<ScanContextDatabasePybind::Params>(m, "ScanContextDatabaseParams")
      .def(py::init<>())
      .def_readwrite("sc_params", &ScanContextDatabasePybind::Params::sc_params)
      .def_readwrite("number_ring_key_nn", &ScanContextDatabasePybind::Params::number_ring_key_nn)
      .def_readwrite("query_distance_threshold", &ScanContextDatabasePybind::Params::query_distance_threshold)
      .def_readwrite("kdtree_rebuild_threshold", &ScanContextDatabasePybind::Params::kdtree_rebuild_threshold);
}