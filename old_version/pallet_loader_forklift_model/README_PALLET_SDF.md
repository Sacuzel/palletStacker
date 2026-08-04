# Simplified EUR pallet SDF

## Geometry

The model is intentionally simpler than a certified EPAL pallet. It follows the
requested five-member construction while retaining the EUR pallet envelope:

- footprint: 800 x 1200 mm
- total height: 144 mm
- bottom slab: 800 x 1200 x 22 mm
- top slab: 800 x 1200 x 22 mm
- three longitudinal stringers: 50 x 1200 x 100 mm each
- clear width of each fork channel: 325 mm
- mass: 25 kg

The stringers run in the Y direction. Forks therefore enter from either end of
the 1200 mm dimension and travel through the two open channels.

## Files

- `models/euro_pallet/model.sdf`: standalone dynamic model
- `models/euro_pallet/model.config`: Gazebo model metadata
- `config.py`: enables the model and defines its project-relative path
- `gazebo_exporter.py`: loads the standalone SDF, applies export-time mass and
  contact parameters, renames and positions each instance, and inlines it into
  the generated world
- `examples/pallet_only_world.sdf`: generated one-pallet smoke-test world

## Running

Normal project execution is unchanged:

```bash
python main.py
gz sim gazebo_runs/latest/pallet_towers.sdf
```

The output world is self-contained because the pallet `<model>` is inlined.
No `GZ_SIM_RESOURCE_PATH`, Fuel download, or mesh dependency is required.

To temporarily restore the old solid cuboid, set:

```python
USE_EURO_PALLET_MODEL = False
```

## Validation performed

- all Python modules compile
- `model.sdf`, `model.config`, and the generated example world are well-formed XML
- the pallet is dynamic and has a 25 kg inertial block
- the model contains five primitive box collisions and five matching visuals
- the generated exporter world contains the inlined dynamic pallet model
- pallet deck and exported box reference heights include the same spawn clearance

Gazebo / sdformat binaries were not available in the build environment, so the
model was not runtime-simulated here. Run the example world in the target Gazebo
installation before proceeding to the forklift stage.
