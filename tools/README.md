# tools/

Hardware probes, and two modules the ground station actually depends on.

## Not all of this is diagnostic

`i2c_bitbang_probe.py` and `i2c_bitbang_read.py` are imported at RUNTIME by
`ground_station/inputs.py`, which falls back to a userspace bit-banged I2C bus
when the kernel one is unavailable. `inputs.py` reaches them by putting this
directory on `sys.path`, so **moving or renaming either file breaks the ADC
fallback**, and it breaks it quietly — the failure only shows up on a rig where
the kernel bus is missing.

Everything else here is a probe, safe to run and safe to ignore.

## They contend for the same pins as the running system

A probe that requests a GPIO line the ground station already holds will fail
with `EBUSY`, and the reverse is also true: `inputs.py` reports "switch pins
busy" when a probe has them. Stop the ground station before running one.

## Fixed 2026-08-29

Each of these used `sys.path.insert(0, "/home/arnobot")` to find a sibling —
correct when the tree lived directly in the home directory, and broken from the
moment it moved into `DuctCleaning/`. They now resolve relative to their own
location. `gpio_watch.py` and `live_watch.py` additionally carried a UTF-8 BOM,
which makes a file unparseable by Python: neither had been runnable at all.
