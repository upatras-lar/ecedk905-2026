#!/usr/bin/env python3
"""
Torque-versus-commanded-voltage calibration for an FOC motor.

Expected Arduino CSV columns:
    point,voltage_V,mass_g,force_N,torque_Nm,speed_rpm

Fits torque = k*|V| + b and torque = k0*|V|, after excluding samples where
|speed| exceeds --max-stall-rpm. This is a torque-per-commanded-voltage slope
(N*m/V), not the motor's physical torque constant Kt (N*m/A).

Usage:
    python torque_fit.py --port COM3
    python torque_fit.py --csv torque_data.csv
    python torque_fit.py --csv torque_data.csv --max-stall-rpm 3
    python torque_fit.py --csv torque_data.csv --torque 0.010 0.015

Requirements: pip install numpy matplotlib pyserial (pyserial only for --port)
"""

import argparse
import os
import sys
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

HEADER = "point,voltage_V,mass_g,force_N,torque_Nm,speed_rpm"
DEFAULT_MAX_STALL_RPM = 10.0


def parse_row(line):
    parts = line.strip().split(",")
    if len(parts) != 6:
        return None
    try:
        row = [float(p) for p in parts]
    except ValueError:
        return None
    return row if np.all(np.isfinite(row)) else None


def capture(port, baud):
    """Read CSV rows from Arduino; send 's' when the user is ready."""
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
                    input(">>> Rod against the load cell with no preload? Press Enter to start... ")
                    ser.write(b"s")
                    started = True
                    continue
                row = parse_row(line)
                if row is not None:
                    rows.append(row)
                    print(line)
                    continue
                print(line)
                if line.startswith("#DONE") or line.startswith("#ABORTED"):
                    break
        except KeyboardInterrupt:
            print("\nStopped by the user.")
            ser.write(b"x")
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
            f.write(f"{int(r[0])},{r[1]:.3f},{r[2]:.2f},{r[3]:.5f},{r[4]:.6f},{r[5]:.2f}\n")
    print(f"Data saved to {path}")


def fit_with_intercept(x, y):
    """Least-squares fit y = k*x + b."""
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    if x.size != y.size:
        raise ValueError("x and y must contain the same number of values")
    if x.size < 3:
        raise ValueError("at least 3 points are needed for a fit with an intercept")
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("x and y must contain only finite values")

    A = np.column_stack((np.ones_like(x), x))
    coefficients, _, rank, _ = np.linalg.lstsq(A, y, rcond=None)
    if rank < 2:
        raise ValueError("voltage values must vary enough to determine slope and intercept")
    b, k = coefficients
    res = y - (k * x + b)
    sse = float(np.sum(res**2))
    sst = float(np.sum((y - y.mean())**2))
    return dict(
        k=float(k), b=float(b), r2=1.0 - sse / sst if sst > 0 else float("nan"),
        rmse=float(np.sqrt(sse / x.size)), res=res,
    )


def fit_through_origin(x, y):
    """Least-squares fit y = k0*x with intercept fixed at zero."""
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    if x.size != y.size:
        raise ValueError("x and y must contain the same number of values")
    if x.size < 2:
        raise ValueError("at least 2 points are needed for a fit through the origin")
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("x and y must contain only finite values")
    A = x[:, None]
    coefficients, _, rank, _ = np.linalg.lstsq(A, y, rcond=None)
    if rank < 1:
        raise ValueError("at least one nonzero voltage is required")
    k0 = float(coefficients[0])
    res = y - k0 * x
    return dict(k=k0, rmse=float(np.sqrt(np.mean(res**2))), res=res)


def signed(x, decimals):
    return f"{'+' if x >= 0 else '−'} {abs(x):.{decimals}f}"


def voltage_for_torque(fit, fit0, tau):
    return (tau - fit["b"]) / fit["k"], tau / fit0["k"], tau / fit["k"]


def print_voltage_table(fit, fit0, tau_max, queries):
    taus = list(queries) if queries else list(np.linspace(tau_max / 8.0, tau_max, 8))
    print("================ Voltage needed for a given torque ================")
    print("  torque (N*m) | torque (mN*m) | V = (T-b)/k | V = T/k0 | V = T/k (no offset)")
    for tau in taus:
        v_free, v_origin, v_fw = voltage_for_torque(fit, fit0, tau)
        flag = "*" if (tau > tau_max * 1.0001 or tau < 0) else " "
        neg = " (<0: below the fitted offset b)" if v_free < 0 else ""
        print(f"  {tau:12.5f} | {tau * 1e3:13.3f} | {v_free:11.3f} | {v_origin:8.3f} | {v_fw:10.3f}{flag}{neg}")
    print("  * outside the measured torque range (extrapolation)")
    print("  k and k0 are torque per commanded volt (N*m/V), not Kt (N*m/A).")
    print("  The first column inverts the fitted model including its offset; the last omits it.")
    print("====================================================================\n")


def plot_inverse(v, torque, fit, fit0, save_fig):
    fig, ax = plt.subplots(figsize=(8, 5.5))
    tt = np.linspace(0.0, max(float(np.max(torque)), 0.0) * 1.05, 200)
    ax.plot(torque * 1e3, v, "o", label="measurements used in fit")
    v_free = (tt - fit["b"]) / fit["k"]
    keep = v_free >= 0
    ax.plot(tt[keep] * 1e3, v_free[keep], "-",
            label=f"linear fit with offset: V = (T {('−' if fit['b'] >= 0 else '+')} {abs(fit['b']) * 1e3:.2f}) / {fit['k'] * 1e3:.3f}\nT in mN·m, V in volts")
    if fit0["k"] != 0:
        ax.plot(tt * 1e3, tt / fit0["k"], "--",
                label=f"through origin: V = T / {fit0['k'] * 1e3:.3f}")
    ax.set_xlabel("Torque (mN·m)")
    ax.set_ylabel("Commanded voltage (V)")
    ax.set_title("Commanded voltage needed for a given torque (stall calibration)")
    ax.minorticks_on()
    ax.grid(True, which="major", alpha=0.4)
    ax.grid(True, which="minor", alpha=0.15)
    ax.legend()
    fig.tight_layout()
    if save_fig:
        base, ext = os.path.splitext(save_fig)
        path = f"{base}_inverse{ext or '.png'}"
        fig.savefig(path, dpi=150)
        print(f"Figure saved to {path}")
    plt.close(fig)


def report_and_plot(data, vmin, vmax, save_fig, show, torque_queries=None,
                    max_stall_rpm=DEFAULT_MAX_STALL_RPM):
    # Filter malformed/non-finite rows and requested voltage range first.
    finite = np.all(np.isfinite(data), axis=1)
    data = data[finite]
    v_abs = np.abs(data[:, 1])
    keep_v = (v_abs >= vmin) & (v_abs <= vmax)
    data = data[keep_v]
    if len(data) < 3:
        sys.exit("ERROR: fewer than 3 finite data points remain after voltage filtering.")

    moving = np.abs(data[:, 5]) > max_stall_rpm
    n_moving = int(np.count_nonzero(moving))
    if n_moving:
        print(f"Excluding {n_moving} point(s) with |speed| > {max_stall_rpm:g} RPM from the stall-torque fit.")
    data_fit = data[~moving]
    if len(data_fit) < 3:
        sys.exit("ERROR: fewer than 3 stationary points remain. Repeat the test with the shaft restrained.")

    v = np.abs(data_fit[:, 1])
    torque = data_fit[:, 4]
    speed = data_fit[:, 5]
    if np.ptp(v) <= 1e-12:
        sys.exit("ERROR: voltage values do not vary enough for a meaningful regression.")

    try:
        fit = fit_with_intercept(v, torque)
        fit0 = fit_through_origin(v, torque)
    except ValueError as exc:
        sys.exit(f"ERROR: could not calculate the fit: {exc}")

    if fit["k"] <= 0:
        print("WARNING: negative or zero slope. Check voltage sign, load-cell sign, and test setup.")

    print("\n================ Linear regression ================")
    print(f"Points used: {len(v)} (excluded moving: {n_moving}); |V| from {v.min():.3f} to {v.max():.3f} V")
    print("\nModel  torque = k*|V| + b")
    print(f"  k = {fit['k']:.6f} N*m/V = {fit['k'] * 1e3:.3f} mN*m/V")
    print(f"  b = {fit['b'] * 1e3:.4f} mN*m")
    print(f"  R^2 = {fit['r2']:.5f}    RMS residual = {fit['rmse'] * 1e3:.4f} mN*m")
    print("\nModel  torque = k0*|V| (through the origin)")
    print(f"  k0 = {fit0['k']:.6f} N*m/V = {fit0['k'] * 1e3:.3f} mN*m/V")
    print(f"  RMS residual = {fit0['rmse'] * 1e3:.4f} mN*m")
    print("If b is compatible with zero, the two slope estimates should be similar.")
    print("A nonzero b can indicate friction, cogging, load-cell tare/preload, or other offsets.")
    print("\nFor firmware that expects torque per commanded volt:")
    print(f"    foc.setTorqueConstant({fit['k']:.6f}f);")
    print("Verify the firmware API's units before using this value; it is not Kt in N*m/A.")
    print("===================================================\n")

    if fit["k"] > 0 and fit0["k"] > 0:
        print_voltage_table(fit, fit0, float(np.max(torque)), torque_queries)
        plot_inverse(v, torque, fit, fit0, save_fig)
    else:
        print("Inverse voltage-for-torque tables skipped because a fitted slope is not positive.")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 1]})
    vv = np.linspace(0.0, v.max() * 1.05, 100)
    ax1.plot(v, torque * 1e3, "o", label="stationary measurements used in fit")
    ax1.plot(vv, (fit["k"] * vv + fit["b"]) * 1e3, "-",
             label=f"linear fit: T = {fit['k'] * 1e3:.3f}·V {signed(fit['b'] * 1e3, 2)}  (R² = {fit['r2']:.4f})\nT in mN·m, V in volts")
    ax1.plot(vv, fit0["k"] * vv * 1e3, "--", label=f"through origin: T = {fit0['k'] * 1e3:.3f}·V")
    ax1.set_ylabel("Torque (mN·m)")
    ax1.set_title("Torque vs commanded voltage (stall calibration)")
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax2.axhline(0.0, color="k", linewidth=0.8)
    ax2.plot(v, fit["res"] * 1e3, "o")
    ax2.set_xlabel("Absolute commanded voltage (V)")
    ax2.set_ylabel("Residual (mN·m)")
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    if save_fig:
        fig.savefig(save_fig, dpi=150)
        print(f"Figure saved to {save_fig}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Torque-versus-commanded-voltage calibration.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the Arduino (e.g. COM3, /dev/ttyUSB0)")
    src.add_argument("--csv", help="CSV file already saved")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", help="CSV file to write when capturing")
    ap.add_argument("--vmin", type=float, default=0.0, help="ignore points below this |voltage| (V)")
    ap.add_argument("--vmax", type=float, default=float("inf"), help="ignore points above this |voltage| (V)")
    ap.add_argument("--max-stall-rpm", type=float, default=DEFAULT_MAX_STALL_RPM,
                    help=f"exclude samples with |speed| above this threshold (default {DEFAULT_MAX_STALL_RPM:g} RPM)")
    ap.add_argument("--torque", type=float, nargs="+", default=None, metavar="T",
                    help="torques (N*m) for which needed voltage is printed")
    ap.add_argument("--save-fig", help="save plot to this file; inverse plot saved as <name>_inverse.<ext>")
    ap.add_argument("--no-show", action="store_true", help="do not open plot windows")
    args = ap.parse_args()

    if args.vmin < 0 or args.vmax < args.vmin:
        ap.error("require 0 <= --vmin <= --vmax")
    if args.max_stall_rpm < 0:
        ap.error("--max-stall-rpm must be nonnegative")
    if args.torque and any(t < 0 for t in args.torque):
        ap.error("--torque values must be nonnegative")

    if args.port:
        data = capture(args.port, args.baud)
        if data.size == 0:
            sys.exit("No data received.")
        out = args.out or datetime.now().strftime("torque_data_%Y%m%d_%H%M%S.csv")
        save_csv(out, data)
    else:
        data = load_csv(args.csv)
        if data.size == 0:
            sys.exit(f"No valid data rows in {args.csv}")
    if data.ndim != 2 or data.shape[1] != 6:
        sys.exit("ERROR: data must contain six columns: point, voltage_V, mass_g, force_N, torque_Nm, speed_rpm")

    report_and_plot(data, args.vmin, args.vmax, args.save_fig, not args.no_show,
                    args.torque, args.max_stall_rpm)


if __name__ == "__main__":
    main()
