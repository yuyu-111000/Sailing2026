# Upper Computer Framework

Python 3.12 standard-library framework for C1-2 mission orchestration and the
C2 extension.

The package deliberately contains no camera, serial-port, ROS, AI-model, or
vehicle-driver dependency. `sailing/interfaces.py` is the adapter boundary;
`sailing/simulation.py` provides deterministic host-only fakes.

Main modules:

- `mission.py`: global safety lifecycle plus C1/C2 substates.
- `protocol.py`: the independently implemented protocol-v1 codec.
- `safety.py`: neutralization and command limiting before transmission.
- `runtime.py`: a small dependency-injected composition root.

Run tests from this directory with:

```powershell
python -m unittest discover -s tests -v
```

Run both fake, deterministic mission demonstrations with:

```powershell
python -m examples.run_scenarios
```
