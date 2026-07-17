# Mapping

The planned unified map output root remains `data/maps/warehouse_v1`. Phase 6 now implements and validates the nvblox portion:

```text
data/maps/warehouse_v1/nvblox/
├── warehouse.nvblx
├── warehouse.ply
├── nvblox_rates.txt
├── nvblox_timings.txt
└── save_report.json
```

Save the current map with:

```bash
./scripts/save_nvblox_map.sh data/maps/warehouse_v1/nvblox warehouse
```

Native Isaac Sim 32FC1 metric depth is the nvblox depth source; FoundationStereo and ESS are not part of the baseline. cuVSLAM supplies the pose through TF. The automated closed-loop collection of cuVSLAM, nvblox, mesh, occupancy and cuVGL artifacts will be added in Phase 7.

See [nvblox configuration](nvblox_configuration.md) for map parameters and persistence, and [Phase 6 validation](phase6_validation.md) for the tested reconstruction baseline.
