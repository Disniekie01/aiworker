# Self-contained pick/place scene

Isaac Sim stage for FFW SG2 shirt pick/place. The USD layer (`newscene.usda`) is in git; **binary meshes are not** (~1 GB total).

See the main [README](../../README.md#pick--place-simulation) for full setup on a new machine.

## Quick commands

```bash
cd scenes/newscene

# On a machine that already has source assets (Downloads, etc.)
./collect_assets.sh copy
./verify_assets.sh

# Package for transfer
./package_assets.sh    # writes dist/newscene-assets-YYYYMMDD.tar.gz

# On the new machine (after git clone)
./install_assets.sh dist/newscene-assets-YYYYMMDD.tar.gz
./verify_assets.sh
```

## Scene entry

```text
scenes/newscene/newscene.usda
```

## Layout

```text
newscene/
  newscene.usda              # main scene (in git)
  Scene.usda                 # robot articulation (binary, not in git)
  assets/
    environment/Austria/     # warehouse / SceneRobot.usd
    crate/                   # KB3D crate FBX
    shirt/                   # folded shirt USDZ contents
```

Asset requirements and env overrides: `manifest.json`.
