# Self-contained pick/place scene

Isaac stage: `newscene.usda` (robot + warehouse + crate + shirt).

## After clone (any machine)

```bash
git lfs install
git lfs pull
./verify_assets.sh
```

All asset paths in `newscene.usda` are **relative to this folder** — no `~/Downloads` references.

## Layout

```text
scenes/newscene/
  newscene.usda
  Scene.usda                 # LFS
  assets/environment/Austria/
  assets/crate/
  assets/shirt/
```

## Maintainers

To rebuild meshes from external source files (not needed for normal clones), set `NEWSCENE_*` env vars listed in `manifest.json` and run `./collect_assets.sh copy`.
