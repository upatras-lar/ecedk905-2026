#!/usr/bin/env python3
"""
Torque constant of the FOC motor: capture, plot and least-squares fit (no confidence intervals).

The Arduino sketch torque_calibration.ino prints CSV lines
    point,voltage_V,mass_g,force_N,torque_Nm,speed_rpm
This script fits   torque = k * |voltage| (+ offset)   and plots the result.

Usage
    # capture from the Arduino (saves the CSV), then plot and fit
    python torque_fit_lstsq.py --port COM3
    python torque_fit_lstsq.py --port /dev/ttyUSB0 --out data.csv

    # or use a CSV file that was already saved
    python torque_fit_lstsq.py --csv data.csv

    # voltage needed for given torques (N*m), from the fits (also a table and a second plot)
    python torque_fit_lstsq.py --csv data.csv --torque 0.010 0.015 0.020

Requirements:  pip install numpy matplotlib pyserial   (pyserial only for --port)
"""

import argparse
import os
import sys
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

HEADER = "point,voltage_V,mass_g,force_N,torque_Nm,speed_rpm"


STALL_WARN_RPM = 10.0   # warn if the rotor moved faster than this during a step


# ---------------------------------------------------------------- data in / out

def parse_row(line):
    """Returns a list of 6 floats if 'line' is a CSV data row, otherwise None."""
    parts = line.strip().split(",")
    if len(parts) != 6:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def capture(port, baud):
    """Reads the CSV rows from the Arduino. Sends 's' (start) when the user is ready."""
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

                print(line)   # header and messages from the Arduino

                if line.startswith("#DONE") or line.startswith("#ABORTED"):
                    break
        except KeyboardInterrupt:
            print("\nStopped by the user.")
            ser.write(b"x")   # motor to 0 V

    return np.array(rows)


def load_csv(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            row = parse_row(line)
            if row is not None:
                rows.append(row)
    return np.array(rows)


def save_csv(path, data):
    with open(path, "w") as f:
        f.write(HEADER + "\n")
        for r in data:
            f.write(f"{int(r[0])},{r[1]:.3f},{r[2]:.2f},{r[3]:.5f},{r[4]:.6f},{r[5]:.2f}\n")
    print(f"Data saved to {path}")


# ---------------------------------------------------------------- linear fits

def fit_with_intercept(x, y):
    """Least-squares fit y = k*x + b using np.linalg.lstsq."""
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    if x.size != y.size:
        raise ValueError("x and y must contain the same number of values")

    n = x.size
    if n < 3:
        raise ValueError("at least 3 data points are needed for a fit with an intercept")

    # Degree-one polynomial design matrix: columns are [1, x].
    # lstsq minimizes ||A @ coefficients - y||_2 without forming A.T @ A.
    A = np.column_stack((np.ones_like(x), x))
    coefficients, _, rank, _ = np.linalg.lstsq(A, y, rcond=None)
    if rank < 2:
        raise ValueError(
            "the intercept fit is not identifiable: voltage values must vary "
            "enough to determine both slope and intercept"
        )

    b, k = coefficients
    res = y - (k * x + b)
    sse = np.sum(res ** 2)

    ym = y.mean()
    sst = np.sum((y - ym) ** 2)
    r2 = 1.0 - sse / sst if sst > 0 else float("nan")

    return dict(
        k=k,
        b=b,
        r2=r2,
        rmse=np.sqrt(sse / n),
        res=res,
    )


def fit_through_origin(x, y):
    """Least-squares fit y = k*x (intercept fixed at zero) using np.linalg.lstsq."""
    x = np.asarray(x, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    if x.size != y.size:
        raise ValueError("x and y must contain the same number of values")

    n = x.size
    if n < 2:
        raise ValueError("at least 2 data points are needed for a fit through the origin")

    # Single-column design matrix for the constrained model y = k*x.
    A = x[:, None]
    coefficients, _, rank, _ = np.linalg.lstsq(A, y, rcond=None)
    if rank < 1:
        raise ValueError(
            "the through-origin fit is not identifiable: at least one voltage "
            "value must be nonzero"
        )

    k = coefficients[0]
    res = y - k * x
    sse = np.sum(res ** 2)

    return dict(
        k=k,
        rmse=np.sqrt(sse / n),
        res=res,
    )


# ---------------------------------------------------------------- report and plot

def signed(x, decimals):
    """'+ 0.52' or '- 0.52' (for equations in legends)."""
    return f"{'+' if x >= 0 else '−'} {abs(x):.{decimals}f}"


def voltage_for_torque(fit, fit0, tau):
    """Voltage needed for a torque tau (N*m), from the inverse of the fits.

    free      : inverse of  torque = k*V + b      ->  V = (tau - b) / k
    origin    : inverse of  torque = k0*V         ->  V = tau / k0
    firmware  : what foc.setTargetTorque() applies with the constant k pasted
                into the sketch (no offset)       ->  V = tau / k
    """
    return (tau - fit["b"]) / fit["k"], tau / fit0["k"], tau / fit["k"]


def print_voltage_table(fit, fit0, tau_max, queries):
    """Prints the voltage needed for some torques (evenly spaced, or the requested ones)."""
    taus = list(queries) if queries else list(np.linspace(tau_max / 8.0, tau_max, 8))

    print("================ Voltage needed for a given torque ================")
    print("  torque (N*m) | torque (mN*m) | V = (T-b)/k | V = T/k0 | V = T/k (firmware)")
    for tau in taus:
        v_free, v_origin, v_fw = voltage_for_torque(fit, fit0, tau)
        flag = "*" if (tau > tau_max * 1.0001 or tau < 0) else " "
        neg = " (<0: below the fitted offset b)" if v_free < 0 else ""
        print(f"  {tau:12.5f} | {tau * 1e3:13.3f} | {v_free:11.3f} | {v_origin:8.3f} | {v_fw:10.3f}{flag}{neg}")
    print("  * outside the measured range (extrapolation)")
    print("  V = (T-b)/k  : inverse of the fit with offset (best description of the data)")
    print("  V = T/k0     : inverse of the fit through the origin")
    print("  V = T/k      : what foc.setTargetTorque() applies with foc.setTorqueConstant(k): no offset,")
    print("                 so it differs from the first column by the constant b/k volts (relatively larger at low torque)")
    print("====================================================================\n")


def plot_inverse(v, torque, fit, fit0, save_fig):
    """Second figure: voltage needed for a given torque."""
    fig, ax = plt.subplots(figsize=(8, 5.5))

    tt = np.linspace(0.0, torque.max() * 1.05, 200)
    ax.plot(torque * 1e3, v, "o", label="measurements")

    v_free = (tt - fit["b"]) / fit["k"]
    keep = v_free >= 0
    ax.plot(tt[keep] * 1e3, v_free[keep], "-",
            label=f"linear fit: V = (T {'−' if fit['b'] >= 0 else '+'} {abs(fit['b']) * 1e3:.2f}) / "
                  f"{fit['k'] * 1e3:.3f}\nT in mN·m, V in volts")
    ax.plot(tt * 1e3, tt / fit0["k"], "--", label=f"through origin: V = T / {fit0['k'] * 1e3:.3f}")

    ax.set_xlabel("Torque (mN·m)")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Voltage needed for a given torque (stall)")
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


def report_and_plot(data, vmin, vmax, save_fig, show, torque_queries=None):
    v = np.abs(data[:, 1])
    torque = data[:, 4]
    speed = data[:, 5]

    keep = (v >= vmin) & (v <= vmax)
    v, torque, speed = v[keep], torque[keep], speed[keep]

    if len(v) < 3:
        sys.exit("ERROR: at least 3 data points are needed for the fit.")

    if np.max(np.abs(speed)) > STALL_WARN_RPM:
        print(f"WARNING: the rotor moved up to {np.max(np.abs(speed)):.1f} rpm during the test. "
              "The rod was not blocking the shaft, so those points are not valid stall torques.")

    try:
        fit = fit_with_intercept(v, torque)
        fit0 = fit_through_origin(v, torque)
    except ValueError as exc:
        sys.exit(f"ERROR: could not calculate the fit: {exc}")

    if fit["k"] <= 0:
        print("WARNING: negative slope. Check VOLTAGE_SIGN and LOADCELL_REVERSE in the sketch.")

    print("\n================ Linear regression ================")
    print(f"Points used: {len(v)}  (|V| from {v.min():.3f} to {v.max():.3f} V)")
    print("\nModel  torque = k * V + b")
    print(f"  k = {fit['k']:.6f} N*m/V = {fit['k'] * 1e3:.3f} mN*m/V")
    print(f"  b = {fit['b'] * 1e3:.4f} mN*m")
    print(f"  R^2 = {fit['r2']:.5f}    RMS residual = {fit['rmse'] * 1e3:.4f} mN*m")
    print("\nModel  torque = k0 * V  (through the origin)")
    print(f"  k0 = {fit0['k']:.6f} N*m/V = {fit0['k'] * 1e3:.3f} mN*m/V")
    print(f"  RMS residual = {fit0['rmse'] * 1e3:.4f} mN*m")
    print("\nIf the offset b is compatible with 0, both constants agree. A clearly non-zero")
    print("b means friction, cogging or a tare/preload error: check the setup.")
    print("\nFor FOC_lar (torque = k * voltage):")
    print(f"    foc.setTorqueConstant({fit['k']:.6f}f);")
    print("===================================================\n")

    inverse_ok = fit["k"] > 0 and fit0["k"] > 0
    if inverse_ok:
        print_voltage_table(fit, fit0, torque.max(), torque_queries)
        plot_inverse(v, torque, fit, fit0, save_fig)
    else:
        print("The inverse relation (voltage for a given torque) is not shown because the slope is not positive.")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 1]})

    vv = np.linspace(0.0, v.max() * 1.05, 100)
    ax1.plot(v, torque * 1e3, "o", label="measurements")
    ax1.plot(vv, (fit["k"] * vv + fit["b"]) * 1e3, "-",
             label=f"linear fit: T = {fit['k'] * 1e3:.3f}·V {signed(fit['b'] * 1e3, 2)}  "
                   f"(R² = {fit['r2']:.4f})\nT in mN·m, V in volts")
    ax1.plot(vv, fit0["k"] * vv * 1e3, "--",
             label=f"through origin: T = {fit0['k'] * 1e3:.3f}·V")
    ax1.set_ylabel("Torque (mN·m)")
    ax1.set_title("Motor torque vs voltage (stall)")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2.axhline(0.0, color="k", linewidth=0.8)
    ax2.plot(v, fit["res"] * 1e3, "o")
    ax2.set_xlabel("Voltage (V)")
    ax2.set_ylabel("Residual (mN·m)")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()

    if save_fig:
        fig.savefig(save_fig, dpi=150)
        print(f"Figure saved to {save_fig}")
    if show:
        plt.show()


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="Capture, plot and fit the torque constant of the FOC motor.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the Arduino (e.g. COM3, /dev/ttyUSB0)")
    src.add_argument("--csv", help="CSV file already saved")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", help="CSV file to write when capturing (default: torque_data_<date>.csv)")
    ap.add_argument("--vmin", type=float, default=0.0, help="ignore points below this |voltage| (V)")
    ap.add_argument("--vmax", type=float, default=float("inf"), help="ignore points above this |voltage| (V)")
    ap.add_argument("--torque", type=float, nargs="+", default=None, metavar="T",
                    help="torques (N*m) for which the needed voltage is printed (default: 8 values up to the measured maximum)")
    ap.add_argument("--save-fig", help="save the figure to this file (png, pdf...); the inverse plot is saved "
                                       "as <name>_inverse.<ext>")
    ap.add_argument("--no-show", action="store_true", help="do not open the plot window")
    args = ap.parse_args()

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

    report_and_plot(data, args.vmin, args.vmax, args.save_fig, not args.no_show, args.torque)


if __name__ == "__main__":
    main()
