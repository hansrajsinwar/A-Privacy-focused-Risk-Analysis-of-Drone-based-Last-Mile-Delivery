"""Tests for remote_id.py: python -m pytest -q test_remote_id.py"""
import numpy as np

from remote_id import RID, cluster, fly, mitigate, observer_estimate, place_customers


def test_customers_are_separated_and_reproducible():
    cfg = RID()
    a, b = place_customers(cfg, 3), place_customers(cfg, 3)
    assert np.allclose(a, b)
    d = np.linalg.norm(a[:, None] - a[None], axis=2) + np.eye(len(a)) * 1e9
    assert d.min() >= cfg.min_sep


def test_grid_snap_and_noise():
    rng = np.random.default_rng(1)
    p = np.array([[10.0, 130.0], [149.0, -20.0]])
    g = mitigate("grid_100", p, rng)
    assert np.allclose(g, [[50, 150], [150, -50]])
    n = mitigate("noise_25", np.zeros((5000, 2)), rng)
    assert abs(n.std() - 25) < 1.5
    assert np.allclose(mitigate("none", p, rng), p)


def test_cluster_merges_close_points():
    c = cluster([np.array([0.0, 0.0]), np.array([10.0, 0.0]), np.array([200.0, 0.0])], 40)
    assert len(c) == 2


def test_unmitigated_broadcast_reveals_address():
    cfg = RID()
    addr = np.array([120.0, 80.0])
    f = fly(cfg, addr, seed=5)
    est = observer_estimate(cfg, f["t"], f["est"])
    assert np.linalg.norm(est - addr) < cfg.hit_radius


def test_gps_spoof_moves_true_position_away_from_broadcast():
    cfg = RID()
    f = fly(cfg, np.array([120.0, 80.0]), seed=5, spoof=True)
    gap = np.linalg.norm(f["est"] - f["true"], axis=1).max()
    assert gap > 5.0
