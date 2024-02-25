import sys
import time

import resolve_robotics_uri_py

import mujoco
import mujoco.viewer

import mujoco_urdf_package_uri_python

import xml.etree.ElementTree as ET

def get_mujoco_ready_string_from_package_uri(package_uri_file):
  absolute_file_name = resolve_robotics_uri_py.resolve_robotics_uri(package_uri_model)
  # Open file, strippath set to false, and return the string
  # Inspired from https://github.com/ami-iit/comodo/blob/8cbd5f5bc6f65a1385a646188abf999844244d30/src/comodo/robotModel/robotModel.py#L191,
  # thanks Carlotta Sartore
  tree = ET.parse(absolute_file_name)
  root = tree.getroot()
  robot_el = None
  for elem in root.iter():
      if elem.tag == "robot":
          robot_el = elem
          break

  ## Adding compiler discard visaul false for mujoco rendering
  mujoco_el = ET.Element("mujoco")
  compiler_el = ET.Element("compiler")
  compiler_el.set("strippath", "false")
  mujoco_el.append(compiler_el)
  robot_el.append(mujoco_el)
  # Convert the XML tree to a string
  return ET.tostring(root)

# This example is based on the code in https://mujoco.readthedocs.io/en/3.1.2/python.html#passive-viewer

if len(sys.argv) < 2:
    # fallback
    package_uri_model = "package://iCub/robots/iCubGazeboV2_7/model.urdf"
else:
    package_uri_model = sys.argv[1]

# Before parsing the model, we register the urdf package:// URI resource provider
mujoco_urdf_package_uri_python.register_resource_provider()

# The ResourceProvider provided by this package does not work in from_xml_path calls, so
# to get the absolute location of the model we get resolve-robotics-uri-py
m = mujoco.MjModel.from_xml_string(get_mujoco_ready_string_from_package_uri(package_uri_model))
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
