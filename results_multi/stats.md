# Results over repeated runs

| Scenario | Outcomes | Max est. error (m) | Landing error (m) | Detect latency (s) | Min sep (m) |
|---|---|---|---|---|---|
| baseline | PASS 5/5 | 0.65 ± 0.23 | 1.27 ± 0.21 |  |  |
| wind_gust | PASS 5/5 | 0.65 ± 0.26 | 1.31 ± 0.25 |  |  |
| gps_dropout_short | PASS 5/5 | 0.65 ± 0.23 | 1.27 ± 0.21 |  |  |
| gps_dropout_long | SAFE_ABORT 4/5, DEGRADED 1/5 | 3.60 ± 3.23 | 38.30 ± 2.43 | 0.82 ± 0.00 |  |
| gps_step_jump | SAFE_ABORT 4/5, DEGRADED 1/5 | 3.15 ± 2.49 | 37.94 ± 2.10 | 1.00 ± 0.00 |  |
| gps_drift_slow | SAFE_ABORT 4/5, DEGRADED 1/5 | 5.56 ± 3.31 | 29.99 ± 3.53 | 10.20 ± 1.42 |  |
| gps_drift_slow_no_detector | DEGRADED 5/5 | 13.72 ± 0.74 | 12.10 ± 0.85 |  |  |
| gps_drift_consistent | DEGRADED 5/5 | 14.49 ± 0.74 | 13.52 ± 0.93 |  |  |
| gps_feed_walkoff | DEGRADED 5/5 | 11.21 ± 0.76 | 9.89 ± 0.98 |  |  |
| gps_feed_circle | SAFE_ABORT 4/5, DEGRADED 1/5 | 4.36 ± 3.50 | 39.38 ± 2.63 | 3.76 ± 0.87 |  |
| gps_freeze | DEGRADED 4/5, SAFE_ABORT 1/5 | 9.19 ± 6.20 | 35.32 ± 4.61 | 2.28 ± 0.38 |  |
| gps_noise_high | SAFE_ABORT 3/5, DEGRADED 2/5 | 4.96 ± 4.27 | 35.99 ± 2.23 | 1.64 ± 0.62 |  |
| baro_stuck_climb | PASS 5/5 | 2.83 ± 0.55 | 1.25 ± 0.24 |  |  |
| baro_bias | PASS 5/5 | 0.67 ± 0.23 | 1.29 ± 0.20 |  |  |
| imu_bias_then_gps_loss | FAIL 5/5 | 59.02 ± 12.66 |  | 5.62 ± 0.57 |  |
| battery_false_low | SAFE_ABORT 5/5 | 0.63 ± 0.24 | 33.36 ± 0.80 | 0.00 ± 0.00 |  |
| battery_false_high | FAIL 5/5 | 8.81 ± 1.57 |  |  |  |
| cmd_latency_0p4s | PASS 5/5 | 1.43 ± 0.41 | 1.41 ± 0.38 |  |  |
| cmd_latency_1s | HUNG 5/5 | 0.74 ± 0.21 |  |  |  |
| two_drones_baseline | PASS 5/5 | 0.64 ± 0.23 | 1.12 ± 0.20 |  | 4.98 ± 1.41 |
| two_drones_no_avoidance | FAIL 4/5, PASS 1/5 | 10.92 ± 6.92 | 1.33 (1 run) |  | 0.71 ± 0.71 |
| two_drones_v2v_freeze | FAIL 4/5, PASS 1/5 | 10.92 ± 6.92 | 1.33 (1 run) |  | 0.71 ± 0.71 |
| two_drones_v2v_drop | FAIL 4/5, PASS 1/5 | 10.92 ± 6.92 | 1.33 (1 run) |  | 0.71 ± 0.71 |
| two_drones_v2v_ghost | HUNG 5/5 | 0.66 ± 0.22 | 1.04 ± 0.13 |  | 6.95 ± 0.89 |
| two_drones_gps_spoof_d1 | DEGRADED 5/5 | 24.30 ± 0.95 | 22.78 ± 0.87 |  | 9.39 ± 0.74 |

Runs per scenario: [5]. Values are mean ± 95 % CI (t distribution, n-1 df) over the runs in which the metric exists; a metric that exists in only one run is shown as '(1 run)'. Max est. error is the maximum over every 0.02 s step.
