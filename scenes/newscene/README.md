# Self-contained pick/place scene

Isaac Sim stage for FFW SG2 shirt pick/place. **Assets are in git via Git LFS.**

```bash
git lfs install
git lfs pull
./verify_assets.sh
```

## Layout

```text
newscene/
  newscene.usda              # main scene
  Scene.usda                 # robot articulation (~744 MB, LFS)
  assets/environment/Austria/
  assets/crate/
  assets/shirt/
```

`collect_assets.sh` — optional maintainer script to rebuild assets from local source files (`manifest.json`).

See the main [README](../../README.md).
