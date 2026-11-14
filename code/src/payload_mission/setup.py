from setuptools import setup
import glob
pkg = "payload_mission"
setup(name=pkg, version="0.2.0", packages=[pkg, pkg + ".urc"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + pkg]),
                  ("share/" + pkg, ["package.xml"]),
                  ("share/" + pkg + "/missions", glob.glob("missions/*.json")),
                  ("share/" + pkg + "/config", glob.glob("config/*.urdf"))],
      install_requires=["setuptools"], zip_safe=True,
      entry_points={"console_scripts": ["mission_node_urc = payload_mission.mission_node_urc:main"]})
