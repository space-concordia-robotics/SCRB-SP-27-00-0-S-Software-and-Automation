"""
mission_node_urc.py -- PARTIAL URC mission node (the parts that are settled).

WRITTEN:   mission lookup, running a step file in a worker thread, abort, progress reporting.
NOT WRITTEN (left as TODO on purpose, pending review with the lead):
    * the GUI action server / goal handling      -> TODO(GUI)
    * the operator-prompt service                -> TODO(prompt)
    * the telemetry registry for the TUI         -> TODO(telemetry)
    * Analyze Site and every other URC mission   -> TODO(missions)
    * all ERC modules                            -> TODO(erc)
"""
import threading

from .context import Ctx
from .hw import HW
from .runner import StepRunner
from .units import Units
from .urc.deep_sample import REGISTRY as DEEP_SAMPLE

# mission name (what the GUI will ask for) -> (step file, functions it may call)
MISSIONS = {
    "urc_take_deep_sample": ("urc_take_deep_sample.json", DEEP_SAMPLE),
    # TODO(missions): urc_analyze_site, urc_...  (not written yet)
}


class UrcMission:
    """Owns one mission run.  The ROS node (below, stubbed) creates one of these per GUI goal."""

    def __init__(self, backend, urdf_path, missions_dir, on_progress=None, ask_operator=None):
        self.abort = threading.Event()
        self.hw = HW(backend, self.abort)
        self.ctx = Ctx(self.hw, Units(urdf_path), competition="urc")
        self.dir, self.progress, self.ask = missions_dir, on_progress, ask_operator
        self.result = None

    def run(self, mission):
        if mission not in MISSIONS:
            raise KeyError(f"unknown URC mission '{mission}'")
        fname, registry = MISSIONS[mission]
        self.runner = StepRunner.from_file(f"{self.dir}/{fname}", self.ctx, registry, ask_operator=self.ask, on_progress=self.progress)
        self.result = self.runner.run()
        return self.result

    def request_abort(self):
        self.abort.set()


# ---------------------------------------------------------------------------------------------
# ROS node: intentionally a stub.  The real one needs the GUI interface decided first.
# ---------------------------------------------------------------------------------------------
def main():
    # TODO(GUI): create rclpy node, RosBackend(node), action server for "run mission",
    #            service for operator prompts, publisher for progress + telemetry registry.
    raise SystemExit("mission_node_urc: ROS wiring not written yet (see TODO markers in this file)")
