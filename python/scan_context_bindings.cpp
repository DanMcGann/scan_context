#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include "scan_context/database.h"
#include "scan_context/scan_context.h"

// Use short form as it appears alot
namespace py = pybind11;

PYBIND11_MODULE(scan_context_python, m) {
  py::module gtsam = py::module::import("numpy");

  py::class_<ScanContext>(m, "ScanContext")
      .def(py::init<const ScanContext::Params &>(), py::arg("params"))  // Empty Constructor
      .def_static("fromScan", &ScanContext::fromScan<ParenAccessor, Eigen::Vector3d, std::allocator>,
                  py::arg("lidar_scan"), py::arg("params"))  // Constructor from scan
      .def("descriptor", &ScanContext::descriptor)           // const descriptor access. outputs numpy matrix
      .def("ringKey", &ScanContext::ringKey)                 // const ringkey access. outputs numpy vector
      .def("distance", &ScanContext::distance,               // distance computation
           py::arg("other"))
      .def("ringKeyDistance", &ScanContext::ringKeyDistance,  // ring key distance
           py::arg("other"));

  py::class_<ScanContext::Params>(m, "ScanContextParams")
      .def(py::init<>())
      .def_readwrite("number_sectors", &ScanContext::Params::number_sectors)
      .def_readwrite("number_rings", &ScanContext::Params::number_rings)
      .def_readwrite("max_range", &ScanContext::Params::max_range)
      .def("equals", &ScanContext::Params::equals, py::arg("other"));

  py::class_<ScanContextDatabase>(m, "ScanContextDatabase")
      .def(py::init<const ScanContextDatabase::Params &>(), py::arg("params"))
      .def("insert", &ScanContextDatabase::insert, py::arg("key"), py::arg("scan_context"))
      .def("remove", &ScanContextDatabase::remove, py::arg("key"))
      .def("query", &ScanContextDatabase::query, py::arg("query"))
      .def("queryK", &ScanContextDatabase::queryK, py::arg("query"), py::arg("k"));

  py::class_<ScanContextDatabase::Params>(m, "ScanContextDatabaseParams")
      .def(py::init<>())
      .def_readwrite("sc_params", &ScanContextDatabase::Params::sc_params)
      .def_readwrite("number_ring_key_nn", &ScanContextDatabase::Params::number_ring_key_nn)
      .def_readwrite("query_distance_threshold", &ScanContextDatabase::Params::query_distance_threshold)
      .def_readwrite("kdtree_rebuild_threshold", &ScanContextDatabase::Params::kdtree_rebuild_threshold);
}