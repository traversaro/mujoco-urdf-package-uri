#include <pybind11/pybind11.h>

#include <mujoco_urdf_package_uri/mujoco_urdf_package_uri.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(mujoco_urdf_package_uri_python, m) {
    m.doc() = R"pbdoc(
        mujoco_urdf_package_uri_python
        -----------------------

        .. currentmodule:: mujoco_urdf_package_uri_python

        .. autosummary::
           :toctree: _generate

           register_resource_provider
    )pbdoc";
    m.def("register_resource_provider", &mjp_registerURDFPackageURIResourceProvider, R"pbdoc(
        Register the mujoco_urdf_package_uri ResourceProvider.
    )pbdoc");


    m.attr("__version__") = MACRO_STRINGIFY(MUPU_VERSION_INFO);
}
