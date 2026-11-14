"""
runner.py -- runs a step file.  Knows nothing about CAN or ROS; it only calls registered functions.

Flow per step:  mark RUNNING -> (ask operator if the step has a prompt) -> run jetson_function.
On ABORT or FAULT: stop outputs (safe state), then undo completed *reversible* steps in reverse
order using each step's antithesis_function.  Irreversible steps are left as they are.
"""
import json
import re

from .hw import Aborted, HardwareFault


def func_name(call):
    """'URCDrillMove(-100 mm)' -> 'URCDrillMove'.  The text in brackets is documentation; real values live in params."""
    m = re.match(r"\s*([A-Za-z_]\w*)", call or "")
    return m.group(1) if m else None


class StepRunner:
    def __init__(self, steps, ctx, registry, ask_operator=None, on_progress=None):
        self.steps, self.ctx, self.registry = steps, ctx, registry
        self.ask = ask_operator or (lambda prompt: None)
        self.progress = on_progress or (lambda step_id, status: None)
        self.done = []                                   # completed steps, newest last, for rollback

    @classmethod
    def from_file(cls, path, ctx, registry, **kw):
        with open(path) as f:
            return cls(json.load(f), ctx, registry, **kw)

    def _set(self, step, status):
        step["status"] = status
        self.progress(step["step_id"], status)

    def run(self):
        """Returns 'COMPLETE', 'ABORTED' or 'FAULT'."""
        step = None
        try:
            for step in self.steps:
                self.ctx.hw._check_abort()
                self._set(step, "RUNNING")
                answer = self.ask(step["operator_prompt"]) if step.get("operator_prompt") else None
                self.registry[func_name(step["jetson_function"])](self.ctx, step.get("params", {}), answer)
                self._set(step, "COMPLETE")
                self.done.append(step)
            return "COMPLETE"
        except Aborted:
            self._fail(step, "ABORTED"); return "ABORTED"
        except (HardwareFault, ValueError, KeyError) as e:      # ValueError = out-of-range request, KeyError = unknown function
            self.error = str(e)
            self._fail(step, "FAULT"); return "FAULT"

    def _fail(self, step, status):
        if step is not None:
            self._set(step, status)
        self.ctx.safe_state()                             # 1. stop everything that is moving
        self.ctx.hw.abort.clear()                         # 2. rollback must be allowed to run
        for s in reversed(self.done):                     # 3. undo completed reversible steps, newest first
            anti = s.get("antithesis_function")
            if s.get("reversible") and anti:
                try:
                    self.registry[func_name(anti)](self.ctx, s.get("antithesis_params", s.get("params", {})), None)
                    self._set(s, "REVERSED")
                except (HardwareFault, Aborted, ValueError):
                    self.ctx.safe_state()                 # rollback itself failed: stop, leave the rest to the operator
                    break
