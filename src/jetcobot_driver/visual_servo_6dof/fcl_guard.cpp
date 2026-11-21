// 只读FCL几何检查：不包含ROS、串口或运动命令。
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <fcl/fcl.h>
#include <memory>
#include <vector>

using Geometry = std::shared_ptr<fcl::CollisionGeometryd>;
static const char* capsule_name = "jetcobot.fcl_geometry";
static void destroy_geometry(PyObject* capsule) {
  delete static_cast<Geometry*>(PyCapsule_GetPointer(capsule, capsule_name));
}
static PyObject* capsule(Geometry geometry) {
  return PyCapsule_New(new Geometry(std::move(geometry)), capsule_name, destroy_geometry);
}
static PyObject* make_mesh(PyObject*, PyObject* args) {
  Py_buffer vertices{}, faces{};
  if (!PyArg_ParseTuple(args, "y*y*", &vertices, &faces)) return nullptr;
  if (vertices.len % (3 * sizeof(double)) || faces.len % (3 * sizeof(int))) {
    PyBuffer_Release(&vertices); PyBuffer_Release(&faces);
    return PyErr_Format(PyExc_ValueError, "Invalid packed mesh arrays");
  }
  const auto* v = static_cast<const double*>(vertices.buf);
  const auto* f = static_cast<const int*>(faces.buf);
  auto points = std::make_shared<std::vector<fcl::Vector3d>>();
  auto polygons = std::make_shared<std::vector<int>>();
  for (Py_ssize_t i = 0; i < vertices.len / static_cast<Py_ssize_t>(sizeof(double)); i += 3)
    points->emplace_back(v[i], v[i+1], v[i+2]);
  for (Py_ssize_t i = 0; i < faces.len / static_cast<Py_ssize_t>(sizeof(int)); i += 3) {
    if (f[i] < 0 || f[i+1] < 0 || f[i+2] < 0 ||
        static_cast<size_t>(std::max(f[i], std::max(f[i+1], f[i+2]))) >= points->size()) {
      PyBuffer_Release(&vertices); PyBuffer_Release(&faces);
      return PyErr_Format(PyExc_ValueError, "Invalid triangle index");
    }
    polygons->insert(polygons->end(), {3, f[i], f[i+1], f[i+2]});
  }
  PyBuffer_Release(&vertices); PyBuffer_Release(&faces);
  if (points->empty() || polygons->empty())
    return PyErr_Format(PyExc_ValueError, "Empty collision mesh");
  try {
    return capsule(std::make_shared<fcl::Convexd>(points, polygons->size()/4, polygons, true));
  } catch (const std::exception& e) { return PyErr_Format(PyExc_ValueError, "%s", e.what()); }
}
static PyObject* make_box(PyObject*, PyObject* args) {
  double x, y, z;
  if (!PyArg_ParseTuple(args, "ddd", &x, &y, &z)) return nullptr;
  if (x <= 0 || y <= 0 || z <= 0) return PyErr_Format(PyExc_ValueError, "Invalid box size");
  return capsule(std::make_shared<fcl::Boxd>(x, y, z));
}
static PyObject* distances(PyObject*, PyObject* args) {
  PyObject *geometry, *pairs;
  Py_buffer matrices{};
  if (!PyArg_ParseTuple(args, "Oy*O", &geometry, &matrices, &pairs)) return nullptr;
  if (!PyList_Check(geometry) || !PyList_Check(pairs) ||
      matrices.len != PyList_Size(geometry) * 16 * static_cast<Py_ssize_t>(sizeof(double))) {
    PyBuffer_Release(&matrices);
    return PyErr_Format(PyExc_ValueError, "Geometry/transform size mismatch");
  }
  const double* data = static_cast<const double*>(matrices.buf);
  std::vector<std::unique_ptr<fcl::CollisionObjectd>> objects;
  for (Py_ssize_t i = 0; i < PyList_Size(geometry); ++i) {
    auto* g = static_cast<Geometry*>(PyCapsule_GetPointer(PyList_GetItem(geometry, i), capsule_name));
    if (!g) { PyBuffer_Release(&matrices); return nullptr; }
    fcl::Transform3d tf = fcl::Transform3d::Identity();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) tf.linear()(r, c) = data[i*16+r*4+c];
      tf.translation()(r) = data[i*16+r*4+3];
    }
    objects.emplace_back(new fcl::CollisionObjectd(*g, tf));
  }
  PyBuffer_Release(&matrices);
  PyObject* output = PyList_New(PyList_Size(pairs));
  for (Py_ssize_t k = 0; k < PyList_Size(pairs); ++k) {
    int i, j;
    if (!PyArg_ParseTuple(PyList_GetItem(pairs, k), "ii", &i, &j) ||
        i < 0 || j < 0 || i >= static_cast<int>(objects.size()) || j >= static_cast<int>(objects.size())) {
      Py_DECREF(output);
      if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "Invalid collision pair");
      return nullptr;
    }
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;
    double distance;
    try { distance = fcl::distance(objects[i].get(), objects[j].get(), request, result); }
    catch (const std::exception& e) {
      Py_DECREF(output); return PyErr_Format(PyExc_RuntimeError, "%s", e.what());
    }
    PyList_SET_ITEM(output, k, PyFloat_FromDouble(distance));
  }
  return output;
}
static PyMethodDef methods[] = {
  {"make_mesh", make_mesh, METH_VARARGS, "Build a convex hull enclosing an original mesh."},
  {"make_box", make_box, METH_VARARGS, "Build a box."},
  {"distances", distances, METH_VARARGS, "Compute configured pair distances."},
  {nullptr, nullptr, 0, nullptr}
};
static PyModuleDef module = {PyModuleDef_HEAD_INIT, "_fcl_guard", nullptr, -1, methods};
PyMODINIT_FUNC PyInit__fcl_guard(void) { return PyModule_Create(&module); }
