# Reference trees: NOT loadable, never launched

These are kept because they record what the team meant the mission to be.
They are excluded from the A-3 load test (`test/test_tree_loading.cpp` only
loads `behavior_trees/*.xml` at the top level).

- **`comp_tree.xml`** could never load. It references `BarrelRoll`,
  `DriveForward`, `GoToTorpedoTarget` and `GoToOctagon`, which were never
  registered with the factory.
- **`qualification_tree.xml`** is last year's tree. Its
  `GoToStart`/`GoToGate` assume the vehicle can move holonomically to a pose,
  which this vehicle cannot do.

The leaves this vehicle has are listed in `../../PLAN.md` (section 3).
