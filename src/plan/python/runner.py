"""Run one piece of plan code: stdin {code, context} -> stdout {ok, delta, printed, error}.

Started as `python3 -I runner.py` from a temp dir that also holds pilot.py.
The plan code gets the pilot vocabulary and a small set of builtins (no
import, open, eval or exec). That keeps honest mistakes contained; it is
not a security sandbox.
"""

import io
import json
import os
import sys
import traceback
import types

SAFE_BUILTINS = (
    "abs", "all", "any", "bool", "dict", "enumerate", "filter", "float", "int", "isinstance",
    "len", "list", "map", "max", "min", "range", "repr", "reversed", "round", "set", "sorted",
    "str", "sum", "tuple", "zip", "True", "False", "None", "ValueError", "Exception",
)


def load_pilot():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pilot.py")
    mod = types.ModuleType("pilot")
    mod.__file__ = path
    with open(path) as f:
        code = compile(f.read(), path, "exec")
    exec(code, mod.__dict__)
    return mod


def main():
    req = json.load(sys.stdin)
    pilot = load_pilot()
    out = io.StringIO()

    def _print(*args, **kwargs):
        kwargs.pop("file", None)
        print(*args, file=out, **kwargs)

    result = {"ok": False, "delta": None, "printed": "", "error": None}
    try:
        pilot._reset(req.get("context") or {})
        ns = pilot.namespace()
        b = __builtins__ if isinstance(__builtins__, dict) else __builtins__.__dict__
        safe = dict((k, b[k]) for k in SAFE_BUILTINS if k in b)
        safe["print"] = _print
        ns["__builtins__"] = safe
        exec(compile(req.get("code", ""), "<plan>", "exec"), ns)
        result["delta"] = pilot._finish()
        result["ok"] = True
    except pilot.PlanError as e:
        result["error"] = "plan error: %s%s" % (e, _line_of(e))
    except SyntaxError as e:
        result["error"] = "syntax error on line %s: %s" % (e.lineno, e.msg)
    except NameError as e:
        result["error"] = "%s%s (the vocabulary has no such name; imports are not available)" % (e, _line_of(e))
    except Exception as e:  # noqa: BLE001 - report anything to the agent
        result["error"] = "%s: %s%s" % (type(e).__name__, e, _line_of(e))
    result["printed"] = out.getvalue()
    sys.stdout.write(json.dumps(result))


def _line_of(exc):
    for frame in reversed(traceback.extract_tb(exc.__traceback__)):
        if frame.filename == "<plan>":
            return " (line %d)" % frame.lineno
    return ""


if __name__ == "__main__":
    main()
