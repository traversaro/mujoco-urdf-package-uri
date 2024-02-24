#include "mujoco_urdf_package_uri.h"

#include <mujoco/mujoco.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <unordered_set>

extern "C" int mupu_open_callback(mjResource* resource);
extern "C" int mupu_read_callback(mjResource* resource, const void** buffer);
extern "C" void mupu_close_callback(mjResource* resource);

mjpResourceProvider g_mujocoURDFPackageURIResourceProvider = {
    .prefix = "package",
    .open = mupu_open_callback,
    .read = mupu_read_callback,
    .close = mupu_close_callback,
    .getdir = NULL};

extern "C" int mjp_registerURDFPackageURIResourceProvider() {
  return mjp_registerResourceProvider(&g_mujocoURDFPackageURIResourceProvider);
}

struct mupu_openedFileData
{
  size_t size;
  void* data;
};

std::string cleanPathSeparator(const std::string& filename, const bool isWindows)
{
  std::string output = filename;
  char pathSeparator = isWindows ? '\\' : '/';
  char wrongPathSeparator = isWindows ? '/' : '\\';
  for (size_t i = 0; i < output.size(); ++i) {
    if (output[i] == wrongPathSeparator) {
      output[i] = pathSeparator;
    }
  }
  return output;
}

bool isFileExisting(const std::string& filename) {
  if (FILE* file = fopen(filename.c_str(), "r")) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

bool getFilePath(const std::string& filename,
                 const std::string& prefixToRemove,
                 const std::unordered_set<std::string>& paths,
                 const bool isWindows,
                       std::string& outputFileName) {
  if (isFileExisting(filename)) {
    outputFileName = filename;
    return true;
  }
  if (filename.substr(0, prefixToRemove.size()) == prefixToRemove) {
    std::string filename_noprefix = filename;
    filename_noprefix.erase(0, prefixToRemove.size());
    for (const std::string& path : paths) {
      const std::string testPath =
          cleanPathSeparator(path + filename_noprefix, isWindows);
      if (isFileExisting(testPath)) {
        outputFileName = testPath;
        return true;
      } else {
        return false;
      }
    }
  }
  outputFileName = filename;  // By default return the input;
  return false;
}

int mupu_open_callback(mjResource* resource) {
  fprintf(stderr, "mupu_open_callback  with %s \n", resource->name);

  if (!resource || !resource->name) {
    return 0;
  }

  std::string resourceName = resource->name;

  bool isWindows = false;
#ifdef _WIN32
  isWindows = true;
#endif

  std::unordered_set<std::string> pathList;

  // List of variables that contain <prefix>/share paths
  std::vector<std::string> envListShare = {"ROS_PACKAGE_PATH"};
  // List of variables that contains <prefix> paths (to which /share needs
  // to be added)
  std::vector<std::string> envListPrefix = {"AMENT_PREFIX_PATH"};  
  for (size_t i = 0; i < envListShare.size(); ++i) {
    const char* env_var_value = std::getenv(envListShare[i].c_str());  
    if (env_var_value) {
      std::stringstream env_var_string(env_var_value);  
      std::string individualPath;  
      while (std::getline(env_var_string, individualPath,
                          isWindows ? ';' : ':')) {
        pathList.insert(individualPath);
      }
    }
  }  

  for (size_t i = 0; i < envListPrefix.size(); ++i) {
    const char* env_var_value = std::getenv(envListPrefix[i].c_str());

    if (env_var_value) {
      std::stringstream env_var_string(env_var_value);

      std::string individualPath;

      while (std::getline(env_var_string, individualPath,
                          isWindows ? ';' : ':')) {
        pathList.insert(individualPath + "/share");
      }
    }
  }

  std::string realAbsoluteFileName;
  bool ok = getFilePath(resourceName, "package:/", pathList, isWindows, realAbsoluteFileName);

  if (!ok) {
    fprintf(stderr, "Failing in opening package:// URI file %s as it was not found on the system.\n", resource->name);
    return 0;
  } else {
    // At this point, we can read the file and add it in the buffer
    fprintf(stderr, "The real path of %s is %s.", resource->name, realAbsoluteFileName.c_str());

    resource->data = calloc(1, sizeof(mupu_openedFileData));

    // Open the file
    std::ifstream file(realAbsoluteFileName, std::ios::binary);
    
    // Check if the file is opened successfully
    if (!file.is_open()) {
        fprintf(stderr, "Error opening file: %s.\n", realAbsoluteFileName.c_str());
        return 0; // Return an error code
    }

    // Get the file size
    file.seekg(0, std::ios::end);
    std::streampos fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    mupu_openedFileData* dataAndSize = (mupu_openedFileData*)resource->data; 
    dataAndSize->size = fileSize;
    dataAndSize->data = calloc(1, fileSize);

    // Read the file content into the vector
    file.read((char*)dataAndSize->data, dataAndSize->size);

    // Close the file
    file.close();

    // Successful return
    return 1;
  }
}

int mupu_read_callback(mjResource * resource, const void** buffer) {
  fprintf(stderr, "mupu_read_callback  with %s \n", resource->name);

  if (!resource || !resource->name || !resource->data) {
    *buffer = NULL;
    return -1;
  }

  mupu_openedFileData* dataAndSize = (mupu_openedFileData*)resource->data; 

  *buffer = dataAndSize->data;

  return dataAndSize->size;
}

void mupu_close_callback(mjResource * resource) {
  fprintf(stderr, "mupu_close_callback  with %s \n", resource->name);

  if (resource->data) {
    mupu_openedFileData* dataAndSize = (mupu_openedFileData*)resource->data; 

    if (dataAndSize->data) {
      free(dataAndSize->data);
      dataAndSize->data = NULL;
    }

    free(resource->data);
    resource->data = NULL;
  }

  return;
}