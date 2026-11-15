# Scenario summary

| Scenario | Outcome | Max est. error (m) | Landing error (m) | Detect latency (s) | Min sep (m) | First failsafe |
|---|---|---|---|---|---|---|
| baseline | PASS | 0.63 | 1.18 |  |  |  |
| wind_gust | PASS | 0.6 | 1.06 |  |  |  |
| gps_dropout_short | PASS | 0.63 | 1.18 |  |  |  |
| gps_dropout_long | SAFE_ABORT | 2.89 | 35.67 | 0.82 |  | GPS data lost for 1.0s |
| gps_step_jump | SAFE_ABORT | 2.5 | 35.55 | 1.0 |  | GPS spoof detector: position change disagrees with GPS veloc |
| gps_drift_slow | SAFE_ABORT | 6.31 | 26.7 | 10.8 |  | GPS spoof detector: position change disagrees with GPS veloc |
| gps_drift_slow_no_detector | DEGRADED | 13.89 | 12.47 |  |  |  |
| gps_drift_consistent | DEGRADED | 14.68 | 13.99 |  |  |  |
| gps_feed_walkoff | DEGRADED | 11.46 | 10.38 |  |  |  |
| gps_feed_circle | SAFE_ABORT | 4.95 | 36.72 | 3.4 |  | GPS spoof detector: position change disagrees with GPS veloc |
| gps_freeze | DEGRADED | 6.25 | 30.07 | 2.4 |  | GPS innovation test failing for 2.0s (inconsistent with IMU/ |
| gps_noise_high | SAFE_ABORT | 3.09 | 34.69 | 1.2 |  | GPS spoof detector: position change disagrees with GPS veloc |
| baro_stuck_climb | PASS | 2.33 | 1.23 |  |  |  |
| baro_bias | PASS | 0.66 | 1.21 |  |  |  |
| imu_bias_then_gps_loss | FAIL | 65.27 |  | 5.82 |  | GPS data lost for 1.0s |
| battery_false_low | SAFE_ABORT | 0.57 | 32.69 | 0.0 |  | MISSION -> LAND: battery critical (9%) |
| battery_false_high | FAIL | 9.45 |  |  |  |  |
| cmd_latency_0p4s | PASS | 0.79 | 1.85 |  |  |  |
| cmd_latency_1s | HUNG | 0.68 |  |  |  |  |
| two_drones_baseline | PASS | 0.58 | 0.89 |  | 6.06 |  |
| two_drones_no_avoidance | FAIL | 13.08 |  |  | 0.78 |  |
| two_drones_v2v_freeze | FAIL | 13.08 |  |  | 0.78 |  |
| two_drones_v2v_drop | FAIL | 13.08 |  |  | 0.78 |  |
| two_drones_v2v_ghost | HUNG | 0.58 | 0.88 |  | 6.06 |  |
| two_drones_gps_spoof_d1 | DEGRADED | 23.67 | 22.17 |  | 8.94 |  |

1 run(s) per scenario. Max/min are worst case over runs; latency is the mean.
