"""Print deterministic C1-2 and C2 host-only scenario summaries."""

from __future__ import annotations

import json

from sailing.scenarios import run_c1_scenario, run_c2_scenario


def main() -> None:
    print(json.dumps({"c1": run_c1_scenario(), "c2": run_c2_scenario()}, indent=2))


if __name__ == "__main__":
    main()
