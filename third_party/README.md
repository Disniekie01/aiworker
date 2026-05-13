# Third-party Python: `robotis_dds_python`

The relay (`ffw_vuer_dds_relay`) needs **ROBOTIS** [`robotis_dds_python`](https://github.com/ROBOTIS-GIT/robotis_dds_python) on `PYTHONPATH`. It is **not** included in this repository.

## Option A — Clone into this repo (recommended for a standalone `aiworker` clone)

From the repository root:

```bash
cd "${ROBOTIS_VR_ROOT:-.}"
git clone https://github.com/ROBOTIS-GIT/robotis_dds_python.git third_party/robotis_dds_python
```

Then follow `INSTALL.txt` to install it into your venv with Cyclone DDS.

## Option B — Use `robotis_lab` next to this repo

If you already have `robotis_lab` checked out **next to** this repository:

```text
parent/
  aiworker/          # this repo
  robotis_lab/
    third_party/
      robotis_dds_python/
```

`env_local.bash` will pick that path automatically when `third_party/robotis_dds_python` is not present here.
