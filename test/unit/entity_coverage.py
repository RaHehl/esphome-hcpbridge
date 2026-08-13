"""Every entity this component offers has to appear in the build example.

ESPHome copies a platform's sources only when a configuration actually uses
that platform, so an entity nobody wrote into the example is an entity no build
ever compiles. That is not a theory: the button platform went on naming a class
that had been renamed away, and it linked everywhere, because the one example
that used a button was the one nobody had built.

Reads the types out of the platform schemas rather than a list kept here, so
adding a type without adding it to the example is what fails.
"""

import ast
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = ROOT / ".github/example_build_hcpbridge.yaml"


def types_of(module):
    tree = ast.parse(module.read_text())
    consts, types = {}, None
    for node in tree.body:
        if not isinstance(node, ast.Assign) or not isinstance(
            node.targets[0], ast.Name
        ):
            continue
        name, value = node.targets[0].id, node.value
        if isinstance(value, ast.Constant) and isinstance(value.value, str):
            consts[name] = value.value
        elif name == "TYPES":
            types = value
    if types is None:
        return []
    if isinstance(types, ast.Dict):
        return [k.value for k in types.keys if isinstance(k, ast.Constant)]
    if isinstance(types, (ast.List, ast.Tuple)):
        out = []
        for e in types.elts:
            if isinstance(e, ast.Constant):
                out.append(e.value)
            elif isinstance(e, ast.Name) and e.id in consts:
                out.append(consts[e.id])
        return out
    return []


def main():
    yaml = EXAMPLE.read_text()
    missing = []
    for module in sorted(ROOT.glob("components/hcpbridge/*/__init__.py")):
        platform = module.parent.name
        if not re.search(rf"^{platform}:", yaml, re.MULTILINE):
            missing.append(f"{platform} (the whole platform)")
            continue
        # Named, never left to the schema default: an example whose job is
        # to be compiled should say what it is asking for.
        missing.extend(
            f"{platform} type {t}"
            for t in types_of(module)
            if not re.search(rf"^\s*type:\s*{t}\s*$", yaml, re.MULTILINE)
        )
    if missing:
        print("  FAILED: nothing in the build matrix ever compiles these")
        for m in missing:
            print(f"    {m}")
        return 1
    print("  ok    every platform and type is in the build example")
    return 0


if __name__ == "__main__":
    sys.exit(main())
