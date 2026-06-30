# Self-contained pick/place scene

Isaac Sim stage for FFW SG2 shirt pick/place.

## In git (LFS)

Binary meshes live here and are tracked with **Git LFS**:

```bash
git lfs install
git lfs pull
./verify_assets.sh
```

## Layout

```text
newscene/
  newscene.usda              # main scene (git)
  Scene.usda                 # robot articulation (LFS, ~744 MB)
  assets/
    environment/Austria/     # warehouse / SceneRobot.usd (LFS)
    crate/                   # KB3D crate FBX
    shirt/                   # folded shirt USDZ contents
```

## Fallback: tarball (no LFS)

```bash
./package_assets.sh    # create dist/newscene-assets-*.tar.gz
./install_assets.sh dist/newscene-assets-*.tar.gz
```

`collect_assets.sh copy` — gather from local Downloads paths (`manifest.json`).
