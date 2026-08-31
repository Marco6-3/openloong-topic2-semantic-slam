from __future__ import annotations

import math

import numpy as np

from scripts.export_rtk_enu import (
    Fix,
    WGS84_A,
    WGS84_E2,
    ecef_to_enu,
    enu_to_geodetic,
    geodetic_to_ecef,
    rejection_reason,
)


def make_fix(**changes: float | int) -> Fix:
    values = {
        "bag_time_ns": 1,
        "header_time_ns": 1,
        "latitude": 31.0,
        "longitude": 121.0,
        "altitude": 10.0,
        "status": 2,
        "covariance_type": 2,
        "covariance_e": 0.0004,
        "covariance_n": 0.0004,
        "covariance_u": 0.0016,
    }
    values.update(changes)
    return Fix(**values)


def test_geodetic_to_ecef_at_equator_and_pole() -> None:
    np.testing.assert_allclose(geodetic_to_ecef(0.0, 0.0, 0.0), [WGS84_A, 0.0, 0.0])
    semi_minor = WGS84_A * math.sqrt(1.0 - WGS84_E2)
    np.testing.assert_allclose(
        geodetic_to_ecef(90.0, 0.0, 0.0), [0.0, 0.0, semi_minor], atol=1e-8
    )


def test_origin_maps_to_zero_enu() -> None:
    origin = geodetic_to_ecef(31.2, 121.5, 8.0)
    np.testing.assert_allclose(ecef_to_enu(origin, origin, 31.2, 121.5), [0.0, 0.0, 0.0])


def test_one_meter_altitude_change_is_up() -> None:
    origin = geodetic_to_ecef(31.2, 121.5, 8.0)
    point = geodetic_to_ecef(31.2, 121.5, 9.0)
    np.testing.assert_allclose(
        ecef_to_enu(point, origin, 31.2, 121.5), [0.0, 0.0, 1.0], atol=1e-8
    )


def test_enu_geodetic_round_trip() -> None:
    origin = (31.12115284, 121.60327063, 6.2988)
    point = (31.12345678, 121.60654321, 8.75)
    origin_ecef = geodetic_to_ecef(*origin)
    enu = ecef_to_enu(geodetic_to_ecef(*point), origin_ecef, origin[0], origin[1])
    recovered = enu_to_geodetic(enu, *origin)
    np.testing.assert_allclose(recovered, point, atol=1e-8)


def test_quality_filter_rejects_invalid_and_uncertain_fixes() -> None:
    assert rejection_reason(make_fix(), 0, 0.5) is None
    assert rejection_reason(make_fix(status=-1), 0, 0.5) == "status_below_minimum"
    assert rejection_reason(make_fix(covariance_type=0), 0, 0.5) == "unknown_covariance"
    assert (
        rejection_reason(make_fix(covariance_e=9.0, covariance_n=9.0), 0, 0.5)
        == "horizontal_sigma_too_large"
    )
