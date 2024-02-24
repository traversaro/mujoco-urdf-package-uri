#ifndef MUJOCO_URDF_PACKAGE_URI_H_
#define MUJOCO_URDF_PACKAGE_URI_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Method to registed the Mujoco Resource provider to handle package:// URIs.
 */
int mjp_registerURDFPackageURIResourceProvider();

#ifdef __cplusplus
}
#endif


#endif
