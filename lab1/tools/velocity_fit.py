#!/usr/bin/env python3
"""
Voltage -> speed relation of the FOC motor: capture, plot and linear fit.

The Arduino sketch velocity_calibration.ino prints CSV lines
    point,voltage_V,speed_rad_s,speed_rpm,std_rad_s
measured with the shaft free, for positive and negative voltages.

For each direction (moving points only, see --min-speed) two models are fitted
on the absolute values:

    linear      |speed| = a*|V| + b                 ->  Ke = 1/a,  V_f = -b/a
                fitted ONLY on the low-speed points (|speed| <= --max-speed-linear,
                default 30 rad/s)
    quadratic   |speed| = c0 + c1*|V| + c2*|V|^2    ->  curvature c2, low-speed Ke
                fitted on all the moving points

The linear fit on low-speed points is intended to estimate the back-EMF constant
with less influence from high-speed effects. The quadratic fit describes curvature
across the measured moving-speed range. This version uses np.linalg.lstsq for both
fits and does not calculate Student-t statistics or confidence intervals.

Usage
    # capture from the Arduino (saves the CSV), then plot and fit
    python velocity_fit.py --port COM3
    python velocity_fit.py --port /dev/ttyUSB0 --out speed_data.csv

    # or use a CSV file that was already saved
    python velocity_fit.py --csv speed_data.csv

    # linear fit on speeds up to 50 rad/s instead of the default 30 rad/s
    # (use 'inf' to fit all the moving points)
    python velocity_fit.py --csv speed_data.csv --max-speed-linear 50

    # optionally also estimate Kt and phase resistance, with the torque constant
    # (N*m/V) measured with torque_fit.py
    python velocity_fit.py --csv speed_data.csv --torque-const 0.0123

    # voltage needed for given speeds (rpm), from the fits
    python velocity_fit.py --csv speed_data.csv --speed 100 200 300

Requirements: pip install numpy matplotlib pyserial (pyserial only for --port)
"""

import argparse
import os
import sys
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

HEADER = "point,voltage_V,speed_rad_s,speed_rpm,std_rad_s"
RPM_PER_RAD_S = 60.0 / (2.0 * np.pi)
DEFAULT_MAX_SPEED_LINEAR = 30.0  # rad/s


def signed(x, decimals):
    """Return '+ 0.52' or '- 0.52' for equations in plot legends."""
    return f"{'+' if x >= 0 else '−'} {abs(x):.{decimals}f}"


def num(x, decimals):
    """Format a number with a proper minus sign, for equations in legends."""
    return f"{x:.{decimals}f}".replace("-", "−")


# ---------------------------------------------------------------- data in / out

def parse_row(line):
    """Return a list of 5 floats if line is a CSV data row, otherwise None."""
    parts = line.strip().split(",")
    if len(parts) != 5:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def capture(port, baud):
    """Read CSV rows from the Arduino. Send 's' when the user is ready."""
    import serial  # pyserial

    rows = []
    started = False

    with serial.Serial(port, baud, timeout=1) as ser:
        print(f"Listening on {port} at {baud} baud (Ctrl+C to stop)...")
        try:
            while True:
                line = ser.readline().decode(errors="replace").strip()
                if not line:
                    continue

                if line.startswith("#READY") and not started:
                    print(line)
                    input(">>> Rod removed, shaft free to spin? Press Enter to start... ")
                    ser.write(b"s")
                    started = True
                    continue

                row = parse_row(line)
                if row is not None:
                    rows.append(row)
                    print(line)
                    continue

                print(line)  # header and messages from the Arduino
                if line.startswith("#DONE") or line.startswith("#ABORTED"):
                    break
        except KeyboardInterrupt:
            print("\nStopped by the user.")
            ser.write(b"x")  # motor to 0 V

    return np.asarray(rows, dtype=float)


def load_csv(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                continue
            row = parse_row(line)
            if row is not None:
                rows.append(row)
    return np.asarray(rows, dtype=float)


def save_csv(path, data):
    with open(path, "w", encoding="utf-8") as f:
        f.write(HEADER + "\n")
        for r in data:
            f.write(f"{int(r[0])},{r[1]:.3f},{r[2]:.3f},{r[3]:.2f},{r[4]:.3f}\n")
    print(f"Data saved to {path}")


# ---------------------------------------------------------------- fits

def linear_fit(x, y):
    """Least-squares fit y = a*x + b using np.linalg.lstsq."""
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()

    if x.size != y.size:
        raise ValueError("x and y must have the same number of values")
    n = x.size
    if n < 3:
        raise ValueError("at least 3 points are needed for a linear fit")

    # Model: y = b + a*x. lstsq minimizes ||X @ [b, a] - y||_2.
    X = np.column_stack((np.ones_like(x), x))
    beta, _, rank, _ = np.linalg.lstsq(X, y, rcond=None)
    if rank < 2:
        raise ValueError("voltage values do not vary enough for a unique linear fit")

    b, a = beta
    res = y - (a * x + b)
    sse = float(np.sum(res**2))
    sst = float(np.sum((y - y.mean())**2))

    return dict(
        a=float(a),
        b=float(b),
        res=res,
        dof=n - 2,
        n=n,
        r2=1.0 - sse / sst if sst > 0 else float("nan"),
        rmse=float(np.sqrt(sse / n)),
    )


def quadratic_fit(x, y):
    """Least-squares fit y = c0 + c1*x + c2*x^2 using np.linalg.lstsq."""
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()

    if x.size != y.size:
        raise ValueError("x and y must have the same number of values")
    n = x.size
    if n < 4:
        raise ValueError("at least 4 points are needed for a quadratic fit")

    X = np.column_stack((np.ones_like(x), x, x**2))
    beta, _, rank, _ = np.linalg.lstsq(X, y, rcond=None)
    if rank < 3:
        raise ValueError("voltage values do not span enough distinct values for a unique quadratic fit")

    res = y - X @ beta
    sse = float(np.sum(res**2))
    sst = float(np.sum((y - y.mean())**2))

    return dict(
        c=beta,
        res=res,
        dof=n - 3,
        n=n,
        r2=1.0 - sse / sst if sst > 0 else float("nan"),
        rmse=float(np.sqrt(sse / n)),
    )


def analyse_quadratic(x, y):
    """Quadratic fit plus derived low-speed slope and top-speed deficit."""
    f = quadratic_fit(x, y)
    c0, c1, c2 = f["c"]
    v_top = float(np.max(x))
    v_low = float(np.min(x))

    # Find a real zero-speed root inside the fitted voltage range, if one exists.
    if abs(c2) > np.finfo(float).eps:
        roots = np.roots([c2, c1, c0])
    elif abs(c1) > np.finfo(float).eps:
        roots = np.array([-c0 / c1])
    else:
        roots = np.array([])

    real_roots = [
        float(r.real)
        for r in np.atleast_1d(roots)
        if abs(getattr(r, "imag", 0.0)) < 1e-9 and 0.0 <= float(r.real) < v_top
    ]
    root = min(real_roots) if real_roots else None
    v0 = root if root is not None else v_low

    def q(v):
        return c0 + c1 * v + c2 * v**2

    # Tangent slope at V0, used as a low-speed slope estimate.
    slope0 = float(c1 + 2.0 * c2 * v0)
    out = dict(
        f,
        c0=float(c0),
        c1=float(c1),
        c2=float(c2),
        root=root,
        v0=float(v0),
        v_top=v_top,
        q=q,
        slope0=slope0,
        q_top=float(q(v_top)),
        ke_low=None,
        deficit=float("nan"),
        deficit_pct=float("nan"),
    )

    if slope0 > 0:
        out["ke_low"] = 1.0 / slope0
        # Difference between the quadratic curve and its tangent at V0.
        tangent_top = float(q(v0) + slope0 * (v_top - v0))
        out["deficit"] = float(c2 * (v_top - v0) ** 2)
        out["deficit_pct"] = (
            100.0 * out["deficit"] / tangent_top if tangent_top > 0 else float("nan")
        )

    return out


def analyse_direction(data, sign, vmin, vmax, min_speed, max_speed_linear):
    """Fit one direction (sign = +1 or -1); return None if there is no data.

    The quadratic model uses all the moving points. The linear model uses only
    moving points with |speed| <= max_speed_linear.
    """
    v_all = data[:, 1]
    sel = (np.sign(v_all) == sign) & (np.abs(v_all) >= vmin) & (np.abs(v_all) <= vmax)
    d = data[sel]
    if len(d) == 0:
        return None

    v = np.abs(d[:, 1])
    w = np.abs(d[:, 2])
    thr = min_speed if min_speed is not None else 0.1 * w.max()
    moving = w >= thr
    lin = moving & (w <= max_speed_linear)

    # A speed sign opposite to voltage is a possible wiring/sign issue.
    wrong = np.sign(d[:, 2]) == -sign
    if np.any(wrong & moving):
        print(
            f"WARNING: for {'positive' if sign > 0 else 'negative'} voltages the shaft turned in the "
            "opposite direction at some points."
        )

    out = dict(
        sign=sign,
        v=v,
        w=w,
        std=d[:, 4],
        moving=moving,
        lin=lin,
        thr=thr,
        max_speed_linear=max_speed_linear,
        fit=None,
        quad=None,
    )

    if lin.sum() < 3:
        return out

    try:
        f = linear_fit(v[lin], w[lin])
    except ValueError as exc:
        print(f"WARNING: could not fit {'positive' if sign > 0 else 'negative'} direction line: {exc}")
        return out

    a, b = f["a"], f["b"]
    if a <= 0:
        return out

    ke = 1.0 / a
    vf = -b / a
    out["fit"] = dict(
        f,
        ke=ke,
        vf=vf,
        kv_rpm=a * RPM_PER_RAD_S,
        top=a * v[moving].max() + b,
    )

    if moving.sum() >= 4:
        try:
            out["quad"] = analyse_quadratic(v[moving], w[moving])
        except ValueError as exc:
            print(
                f"WARNING: could not fit {'positive' if sign > 0 else 'negative'} direction quadratic: {exc}"
            )
    return out


# ---------------------------------------------------------------- report

def report(dirs, torque_const):
    names = {1: "POSITIVE voltage", -1: "NEGATIVE voltage"}
    ok = [d for d in dirs if d and d["fit"]]

    print("\n================ Voltage -> speed fit ================")
    print("Linear model:     |speed| = a*|V| + b            ->  Ke = 1/a,  V_f = -b/a")
    print("                  (fitted only on low-speed points, see --max-speed-linear)")
    print("Quadratic model:  |speed| = c0 + c1*|V| + c2*|V|^2   (speed in rad/s, all moving points)")

    for d in dirs:
        if d is None:
            continue

        print(f"\n--- {names[d['sign']]} ---")
        print(
            f"Points: {len(d['v'])}, moving: {int(d['moving'].sum())} "
            f"(speed >= {d['thr']:.2f} rad/s; the slower ones are the dead zone)"
        )
        print(
            f"Linear fit uses {int(d['lin'].sum())} points with speed <= {d['max_speed_linear']:g} rad/s; "
            f"quadratic fit uses all {int(d['moving'].sum())} moving points"
        )

        f = d["fit"]
        if f is None:
            print(
                "Not enough low-speed points with a positive slope for the linear fit "
                "(need 3 moving points with speed <= --max-speed-linear; try a higher --max-speed-linear, "
                "a lower --min-speed or a higher voltage limit)."
            )
            continue

        print("Linear fit:")
        print(
            f"  slope a  = {f['a']:.3f} rad/s per V "
            f"= {f['kv_rpm']:.1f} rpm/V"
        )
        print(f"  Ke       = {f['ke']:.5f} V*s/rad")
        print(f"  V_f      = {f['vf']:.4f} V (friction / cogging voltage estimate)")
        print(f"  R^2 = {f['r2']:.5f}    RMS residual = {f['rmse']:.3f} rad/s")

        q = d["quad"]
        if q is None:
            print("Quadratic fit: needs at least 4 distinct moving voltage points, skipped.")
            continue

        print("Quadratic fit:")
        print(f"  c2 = {q['c2']:.3f} rad/s per V^2 (curvature)")
        print(f"  c1 = {q['c1']:.3f} rad/s per V")
        print(
            f"  c0 = {q['c0']:.3f} rad/s "
            "(fit coefficient; not necessarily a physical zero-voltage speed)"
        )
        print(
            f"  R^2 = {q['r2']:.5f}    RMS residual = {q['rmse']:.3f} rad/s "
            f"(linear, low-speed points only: {f['rmse']:.3f} rad/s)"
        )
        print(
            f"  Speed at |V| = {q['v_top']:.2f} V: linear (extrapolated) {f['top']:.2f} rad/s, "
            f"quadratic {q['q_top']:.2f} rad/s "
            f"(quadratic minus linear: {q['q_top'] - f['top']:+.2f} rad/s)"
        )

        if q["ke_low"] is not None:
            ref = (
                "where the curve crosses zero speed"
                if q["root"] is not None
                else "at the lowest fitted voltage (no zero-speed crossing in the fitted range)"
            )
            print(f"  Low-speed slope {q['slope0']:.3f} rad/s per V at |V| = {q['v0']:.3f} V ({ref})")
            print(f"  Ke(low speed) = {q['ke_low']:.5f} V*s/rad")
            print(
                f"  At |V| = {q['v_top']:.2f} V the speed is {-q['deficit']:.2f} rad/s "
                f"({-q['deficit_pct']:.1f} %) below the low-speed tangent"
            )
        else:
            print("  The low-speed slope is not positive: the quadratic model may not describe this range well.")

    if len(ok) == 2:
        p, n = (ok[0], ok[1]) if ok[0]["sign"] > 0 else (ok[1], ok[0])
        fp, fn = p["fit"], n["fit"]
        diff = fp["ke"] - fn["ke"]
        mean_ke = 0.5 * (fp["ke"] + fn["ke"])
        print("\n--- Direction comparison ---")
        ratio = fp["ke"] / fn["ke"] if fn["ke"] != 0 else float("nan")
        pct_diff = 100.0 * diff / mean_ke if mean_ke != 0 else float("nan")
        print(f"  Linear Ke(+) / Ke(-) = {ratio:.4f}   (difference {pct_diff:+.2f} %)")
        print(
            f"  V_f(+) = {fp['vf']:.4f} V,  V_f(-) = {fn['vf']:.4f} V "
            f"(difference {fp['vf'] - fn['vf']:+.4f} V)"
        )
        qp, qn = p["quad"], n["quad"]
        if qp and qn and qp["ke_low"] is not None and qn["ke_low"] is not None:
            print(f"  Ke(low speed)(+) / Ke(low speed)(-) = {qp['ke_low'] / qn['ke_low']:.4f}")
        print(
            "  Compare the offsets and slopes to assess directional asymmetry. A larger speed in one direction\n"
            "  at the same |V| can reflect a different effective dead-zone voltage, motor/sensor asymmetry,\n"
            "  or measurement variation. This comparison does not perform a statistical significance test."
        )

    if ok:
        ke_lin = float(np.mean([d["fit"]["ke"] for d in ok]))
        ke_low_list = [
            d["quad"]["ke_low"]
            for d in ok
            if d["quad"] is not None and d["quad"]["ke_low"] is not None
        ]
        ke_low = float(np.mean(ke_low_list)) if ke_low_list else None

        print("\n--- Summary ---")
        print(
            f"  Mean Ke (linear fit, low speeds) = {ke_lin:.5f} V*s/rad "
            f"(speed constant {RPM_PER_RAD_S / ke_lin:.1f} rpm/V of commanded voltage)"
        )
        if ke_low is not None:
            print(
                f"  Mean Ke (quadratic, low speed)   = {ke_low:.5f} V*s/rad "
                f"(speed constant {RPM_PER_RAD_S / ke_low:.1f} rpm/V)"
            )

        c2_values = [d["quad"]["c2"] for d in ok if d["quad"] is not None]
        if any(c2 < 0 for c2 in c2_values):
            print(
                "  At least one quadratic fit bends downward (c2 < 0) over the fitted range. The low-speed\n"
                "  straight-line fit and the quadratic fit serve different purposes: use the low-speed fit\n"
                "  to estimate Ke and the quadratic fit to describe the measured speed curve."
            )
        elif any(c2 > 0 for c2 in c2_values):
            print(
                "  At least one quadratic fit bends upward (c2 > 0) over the fitted range. Review the data\n"
                "  and residuals before assigning a physical interpretation to the curvature."
            )
        else:
            print("  The fitted quadratic coefficients show no measurable curvature in the fitted data.")
        print("  No curvature significance test or coefficient confidence intervals are calculated in this version.")

        if torque_const:
            for label, ke in (("linear", ke_lin), ("low speed", ke_low)):
                if ke is None:
                    continue
                kt = 1.5 * ke
                print(
                    f"  Ke ({label}): Kt = 1.5*Ke = {kt:.5f} N*m/A, "
                    f"phase resistance R = Kt / k = {kt / torque_const:.3f} ohm "
                    f"(k = {torque_const} N*m/V)"
                )
            print(
                "  Compare R with a multimeter (phase-to-phase resistance / 2 for a star-wired motor).\n"
                "  Kt = 1.5*Ke assumes SimpleFOC's amplitude-invariant convention."
            )
    print("======================================================\n")


# ---------------------------------------------------------------- inverse relation (voltage for a speed)

def quad_inverse_formula(c0, c1, c2, v_top):
    """Display the inverse formula for |w| = c0 + c1*|V| + c2*|V|^2."""
    if abs(c2) * max(abs(v_top), 1.0) < 1e-6 * max(abs(c1), 1e-12):
        if abs(c1) > 1e-12:
            return f"|V| = (|ω| {signed(-c0, 2)}) / {c1:.2f}"
        return "quadratic inverse unavailable (both linear and quadratic terms are near zero)"
    d0 = c1**2 - 4.0 * c2 * c0
    return (
        f"|V| = ({num(-c1, 2)} + √({num(d0, 1)} "
        f"{signed(4.0 * c2, 2)}·|ω|)) / {num(2.0 * c2, 2)}"
    )


def build_inverse(d):
    """Build inverse relations: voltage required for a given |speed|."""
    f, q = d["fit"], d["quad"]
    mv = d["moving"]
    lin = d["lin"]
    inv = dict(
        sign=d["sign"],
        a=f["a"],
        b=f["b"],
        vf=f["vf"],
        ke=f["ke"],
        w_min=float(d["w"][mv].min()),
        w_max=float(d["w"][mv].max()),
        lin_w_min=float(d["w"][lin].min()),
        lin_w_max=float(d["w"][lin].max()),
        quad=None,
    )
    if q is not None:
        vg = np.linspace(q["v0"], q["v_top"], 400)
        wg = q["q"](vg)
        if np.all(np.diff(wg) > 0):  # invertible only while the curve is increasing
            inv["quad"] = (vg, wg)
            inv["qc"] = (q["c0"], q["c1"], q["c2"])
            inv["v_top"] = q["v_top"]
    return inv


def inv_linear(inv, w):
    return (w - inv["b"]) / inv["a"]


def inv_quad(inv, w):
    if inv["quad"] is None:
        return float("nan")
    vg, wg = inv["quad"]
    if w < wg[0] or w > wg[-1]:
        return float("nan")
    return float(np.interp(w, wg, vg))


def print_inverse_table(dirs, queries_rpm):
    """Print voltage estimates for requested speeds or a default speed table."""
    invs = [build_inverse(d) for d in dirs if d and d["fit"]]
    if not invs:
        return

    if queries_rpm:
        speeds = list(queries_rpm)
    else:
        top = min(i["w_max"] for i in invs) * RPM_PER_RAD_S
        speeds = list(np.linspace(top / 8.0, top, 8))

    names = {1: "V(+)", -1: "V(-)"}

    def cell(value, flagged):
        if np.isnan(value):
            return f"{'n/a':>10} "
        suffix = "*" if flagged else " "
        return f"{value:10.3f}{suffix}"

    print("================ Voltage needed for a given speed ================")
    head = "  speed (rpm) | speed (rad/s)"
    for i in invs:
        head += f" | {names[i['sign']]} linear | {names[i['sign']]} quadr."
    print(head)

    for rpm in speeds:
        w = rpm / RPM_PER_RAD_S
        row = f"  {rpm:11.1f} | {w:13.2f}"
        for i in invs:
            flag_lin = w < i["lin_w_min"] * 0.9999 or w > i["lin_w_max"] * 1.0001
            flag_quad = w < i["w_min"] * 0.9999 or w > i["w_max"] * 1.0001
            row += " | " + cell(inv_linear(i, w), flag_lin) + " | " + cell(inv_quad(i, w), flag_quad)
        print(row)

    print("  * outside the speed range the model was fitted on (extrapolation); the linear model is only")
    print("    fitted on the low speeds (--max-speed-linear), so it is flagged above that speed")
    print("  n/a : outside the range described by the quadratic curve (not extrapolated above top speed)")
    print("  Relations (speed |ω| in rad/s, voltage |V| in volts):")
    for i in invs:
        name = names[i["sign"]]
        print(
            f"    {name} linear   : |V| = {i['vf']:.4f} {signed(i['ke'], 5)}·|ω| "
            f"(fitted for |ω| from {i['lin_w_min']:.1f} to {i['lin_w_max']:.1f} rad/s)"
        )
        if i["quad"] is not None:
            c0, c1, c2 = i["qc"]
            print(
                f"    {name} quadratic: {quad_inverse_formula(c0, c1, c2, i['v_top'])} "
                f"(valid for |ω| from {max(i['quad'][1][0], 0.0):.1f} to {i['quad'][1][-1]:.1f} rad/s)"
            )
    print("  V(+), V(-) : commanded voltage for the positive and negative direction.")
    print("  Free-shaft values: with a load, the same voltage gives less speed.")
    print("  Below the dead-zone voltage V_f the motor may not start, so very low speeds are not reachable")
    print("  in open loop.")
    print("===================================================================\n")


def plot_inverse(dirs, save_fig):
    """Second figure: voltage needed for a given speed."""
    fig, ax = plt.subplots(figsize=(9, 6))
    colors = {1: "tab:blue", -1: "tab:red"}
    labels = {1: "positive", -1: "negative"}

    for d in dirs:
        if d is None or d["fit"] is None:
            continue
        s, color = d["sign"], colors[d["sign"]]
        inv = build_inverse(d)
        mv = d["moving"]
        lin = d["lin"]

        ax.plot(s * d["w"][lin], s * d["v"][lin], "o", color=color,
                label=f"{labels[s]}: measured (linear fit)")
        high = mv & ~lin
        if np.any(high):
            ax.plot(s * d["w"][high], s * d["v"][high], "s", color=color, alpha=0.7,
                    label=f"{labels[s]}: measured (above {d['max_speed_linear']:g} rad/s, not in linear fit)")
        if np.any(~mv):
            ax.plot(s * d["w"][~mv], s * d["v"][~mv], "o", mfc="none", color=color, alpha=0.6,
                    label=f"{labels[s]}: dead zone (not fitted)")

        # Solid over the fitted speed range, dotted beyond it (extrapolation).
        ww = np.linspace(0.0, inv["lin_w_max"], 100)
        v_lin = inv_linear(inv, ww)
        keep = v_lin >= 0
        ax.plot(
            s * ww[keep], s * v_lin[keep], "-", color=color,
            label=(f"{labels[s]} linear: |V| = {inv['vf']:.3f} "
                   f"{'+' if inv['ke'] >= 0 else '−'} {abs(inv['ke']):.4f}·|ω|\n"
                   "|V| in volts, |ω| in rad/s"),
        )
        if inv["w_max"] > inv["lin_w_max"]:
            ww2 = np.linspace(inv["lin_w_max"], inv["w_max"], 50)
            ax.plot(s * ww2, s * inv_linear(inv, ww2), ":", color=color, linewidth=1.2)

        if inv["quad"] is not None:
            vg, wg = inv["quad"]
            c0, c1, c2 = inv["qc"]
            ax.plot(s * wg, s * vg, "--", color=color, linewidth=2.0,
                    label=f"{labels[s]} quadratic:\n{quad_inverse_formula(c0, c1, c2, inv['v_top'])}")

    ax.axhline(0.0, color="k", linewidth=0.8)
    ax.axvline(0.0, color="k", linewidth=0.8)
    ax.set_xlabel("Speed (rad/s)")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Voltage needed for a given speed (free shaft)")
    ax.minorticks_on()
    ax.grid(True, which="major", alpha=0.4)
    ax.grid(True, which="minor", alpha=0.15)
    ax.legend(fontsize=8)
    sec = ax.secondary_xaxis("top", functions=(lambda w: w * RPM_PER_RAD_S,
                                                lambda r: r / RPM_PER_RAD_S))
    sec.set_xlabel("Speed (rpm)")
    fig.tight_layout()

    if save_fig:
        base, ext = os.path.splitext(save_fig)
        path = f"{base}_inverse{ext or '.png'}"
        fig.savefig(path, dpi=150)
        print(f"Figure saved to {path}")


# ---------------------------------------------------------------- plot

def plot(data, dirs, save_fig):
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(9, 8.5), sharex=True,
        gridspec_kw={"height_ratios": [3, 1]},
    )
    colors = {1: "tab:blue", -1: "tab:red"}
    labels = {1: "positive", -1: "negative"}

    for d in dirs:
        if d is None:
            continue
        s, color = d["sign"], colors[d["sign"]]
        v_signed = s * d["v"]
        w_signed = s * d["w"]
        mv = d["moving"]
        lin = d["lin"]

        ax1.errorbar(v_signed[mv], w_signed[mv], yerr=d["std"][mv], fmt="o", color=color,
                     capsize=2, label=f"{labels[s]}: measured")
        if np.any(~mv):
            ax1.plot(v_signed[~mv], w_signed[~mv], "o", mfc="none", color=color, alpha=0.6,
                     label=f"{labels[s]}: dead zone (not fitted)")

        f = d["fit"]
        if f is not None:
            # Solid over the fit range, dotted beyond it (extrapolation).
            v_lo = max(f["vf"], 0.0)
            v_hi = d["v"][lin].max()
            vv = np.linspace(v_lo, v_hi, 50)
            ax1.plot(
                s * vv, s * (f["a"] * vv + f["b"]), "-", color=color,
                label=(f"{labels[s]} linear (|ω| ≤ {d['max_speed_linear']:g} rad/s): "
                       f"|ω| = {f['a']:.2f}·|V| {signed(f['b'], 2)} rad/s\n"
                       f"Ke = {f['ke']:.4f} V·s/rad, V_f = {f['vf']:.3f} V, R² = {f['r2']:.4f}"),
            )
            if d["v"].max() > v_hi:
                vv2 = np.linspace(v_hi, d["v"].max(), 30)
                ax1.plot(s * vv2, s * (f["a"] * vv2 + f["b"]), ":", color=color, linewidth=1.2)
            ax2.plot(s * d["v"][lin], f["res"], "o", color=color, label=f"{labels[s]} linear")

        q = d["quad"]
        if q is not None:
            vq_min = q["v0"] if q["root"] is not None else d["v"][mv].min()
            vq = np.linspace(vq_min, q["v_top"], 80)
            lab = (
                f"{labels[s]} quadratic: |ω| = {num(q['c2'], 2)}·|V|² "
                f"{signed(q['c1'], 2)}·|V| {signed(q['c0'], 2)} rad/s\nR² = {q['r2']:.4f}"
            )
            ax1.plot(s * vq, s * q["q"](vq), "--", color=color, linewidth=2.0, label=lab)
            ax2.plot(s * d["v"][mv], q["res"], "x", color=color, markersize=8,
                     label=f"{labels[s]} quadratic")

    ax1.axhline(0.0, color="k", linewidth=0.8)
    ax1.axvline(0.0, color="k", linewidth=0.8)
    ax1.set_ylabel("Speed (rad/s)")
    ax1.set_title("Steady-state speed vs voltage (free shaft)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=7)
    sec = ax1.secondary_yaxis("right", functions=(lambda w: w * RPM_PER_RAD_S,
                                                  lambda r: r / RPM_PER_RAD_S))
    sec.set_ylabel("Speed (rpm)")

    ax2.axhline(0.0, color="k", linewidth=0.8)
    ax2.set_xlabel("Voltage (V)")
    ax2.set_ylabel("Residual (rad/s)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=7, ncol=2)

    fig.tight_layout()
    if save_fig:
        fig.savefig(save_fig, dpi=150)
        print(f"Figure saved to {save_fig}")


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(
        description="Capture, plot and fit the voltage -> speed relation of the FOC motor."
    )
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the Arduino (e.g. COM3, /dev/ttyUSB0)")
    src.add_argument("--csv", help="CSV file already saved")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", help="CSV file to write when capturing (default: speed_data_<date>.csv)")
    ap.add_argument(
        "--min-speed", type=float, default=None,
        help="points slower than this (rad/s) are treated as dead zone and not fitted "
             "(default: 10 percent of the maximum speed of each direction)",
    )
    ap.add_argument(
        "--max-speed-linear", type=float, default=DEFAULT_MAX_SPEED_LINEAR,
        help="linear fit uses moving points with |speed| up to this value (rad/s); "
             f"quadratic fit uses all moving points (default: {DEFAULT_MAX_SPEED_LINEAR:g}; "
             "use 'inf' to fit the linear model on all moving points)",
    )
    ap.add_argument("--vmin", type=float, default=0.0, help="ignore points below this |voltage| (V)")
    ap.add_argument("--vmax", type=float, default=float("inf"), help="ignore points above this |voltage| (V)")
    ap.add_argument(
        "--torque-const", type=float, default=None,
        help="torque constant k (N*m/V) from torque_fit.py: also estimates Kt and phase resistance",
    )
    ap.add_argument(
        "--speed", type=float, nargs="+", default=None, metavar="RPM",
        help="speeds (rpm) for which the needed voltage is printed (default: 8 values up to measured maximum)",
    )
    ap.add_argument(
        "--save-fig", help="save the figure to this file (png, pdf...); inverse plot saved as <name>_inverse.<ext>"
    )
    ap.add_argument("--no-show", action="store_true", help="do not open the plot window")
    args = ap.parse_args()

    if args.port:
        data = capture(args.port, args.baud)
        if data.size == 0:
            sys.exit("No data received.")
        out = args.out or datetime.now().strftime("speed_data_%Y%m%d_%H%M%S.csv")
        save_csv(out, data)
    else:
        data = load_csv(args.csv)
        if data.size == 0:
            sys.exit(f"No valid data rows in {args.csv}")

    if data.ndim != 2 or data.shape[1] != 5:
        sys.exit("ERROR: data must contain five columns: point, voltage_V, speed_rad_s, speed_rpm, std_rad_s")

    dirs = [
        analyse_direction(data, +1, args.vmin, args.vmax, args.min_speed, args.max_speed_linear),
        analyse_direction(data, -1, args.vmin, args.vmax, args.min_speed, args.max_speed_linear),
    ]
    if not any(d and d["fit"] for d in dirs):
        print(
            "WARNING: no direction has enough low-speed moving points for the linear fit "
            "(try a higher --max-speed-linear)."
        )

    report(dirs, args.torque_const)
    print_inverse_table(dirs, args.speed)
    plot(data, dirs, args.save_fig)
    if any(d and d["fit"] for d in dirs):
        plot_inverse(dirs, args.save_fig)
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
