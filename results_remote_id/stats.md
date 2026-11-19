# Remote ID inference results

| Condition | Flight error (m) | Flights within hit radius (%) | Addresses recovered (%) | Flights linkable by ID (%) | Broadcast vs true position (m) |
|---|---|---|---|---|---|
| none | 2.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100 ± 0 | 0.2 ± 0.0 |
| id_rotation | 2.0 ± 0.0 | 100.0 ± 0.0 | 100.0 ± 0.0 | 0 ± 0 | 0.2 ± 0.0 |
| noise_25 | 7.1 ± 1.0 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100 ± 0 | 31.4 ± 0.6 |
| noise_100 | 30.4 ± 3.9 | 53.3 ± 9.3 | 82.5 ± 20.8 | 100 ± 0 | 125.2 ± 3.3 |
| grid_100 | 31.9 ± 5.2 | 40.0 ± 17.0 | 40.0 ± 17.0 | 100 ± 0 | 43.1 ± 3.0 |
| gps_spoof | 2.0 ± 0.1 | 100.0 ± 0.0 | 100.0 ± 0.0 | 100 ± 0 | 13.1 ± 1.1 |

Mean ± 95 % CI (t distribution) over 5 seeds. Hit radius 30 m, cluster radius 40 m.
