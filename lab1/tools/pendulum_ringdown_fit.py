#!/usr/bin/env python3
"""
Pendulum ring-down: capture, fit and plot. Identifies shaft friction.

The sketch pendulum_ringdown.ino logs the angle of the freely swinging pendulum:
    t_s,theta_rad           (theta = 0 at the hanging rest position)

The nonlinear pendulum model is fitted to the decaying oscillation:
    theta'' = -w02 * sin(theta - off) - c1 * theta' - c0 * tanh(theta' / 0.05)

    w02 = G/J        (1/s^2)   gravity torque / inertia: sets the period
    c1  = b/J        (1/s)     viscous friction
    c0  = tau_c/J    (rad/s^2) Coulomb friction
    off              (rad)     equilibrium offset

With inertia J (from --inertia, or masses and length), friction torques are:
    b = c1 * J (N*m*s/rad), tau_c = c0 * J (N*m), G_fit = w02 * J.

The outer fit is nonlinear, so np.linalg.lstsq is used to solve each damped,
linearized least-squares step inside the Levenberg-Marquardt optimizer. No
standard errors or confidence intervals are calculated or reported.

Usage
    # capture from Arduino, then fit
    python pendulum_ringdown_fit_lstsq_no_ci.py --port COM3 --rod-mass 0.005 --point-mass 0.020 --length 0.100

    # or use a saved CSV file
    python pendulum_ringdown_fit_lstsq_no_ci.py --csv free.csv --inertia 2.167e-4

    # compare free and braked runs
    python pendulum_ringdown_fit_lstsq_no_ci.py --csv free.csv --braked braked.csv --inertia 2.167e-4 \
        --torque-const 0.0123 --ke 0.035

Requirements: pip install numpy matplotlib pyserial (pyserial only for --port)
"""

import argparse
import math
import sys
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

HEADER = "t_s,theta_rad"
GRAVITY = 9.80665
COULOMB_EPS = 0.05          # rad/s, smoothing of sign(velocity) for Coulomb friction
STEP_MAX = 0.005            # s, largest integration step


# ---------------------------------------------------------------- data in / out

def parse_row(line):
    """Parse a CSV data row, returning two floats or None."""
    parts = line.strip().split(",")
    if len(parts) != 2:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def capture(port, baud):
    """Read CSV rows from Arduino. Sends 's' when the user is ready."""
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
                    input(">>> Pendulum hanging at rest? Press Enter to start (then do not touch it for 1 s)... ")
                    ser.write(b"s")
                    started = True
                    continue

                row = parse_row(line)
                if row is not None:
                    rows.append(row)
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
            f.write(f"{r[0]:.4f},{r[1]:.5f}\n")
    print(f"Data saved to {path}")


# ---------------------------------------------------------------- model and nonlinear fit

def simulate(p, t, coulomb=True):
    """Simulate theta at sample times t for p=[w02,c1,c0,off,theta0,omega0] (RK4)."""
    w02, c1, c0, off, th, om = p
    if not coulomb:
        c0 = 0.0

    sin, tanh = math.sin, math.tanh
    inv_eps = 1.0 / COULOMB_EPS
    out = [th]
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        n = int(dt / STEP_MAX) + 1
        h = dt / n
        for _ in range(n):
            a1 = -w02 * sin(th - off) - c1 * om - c0 * tanh(om * inv_eps)
            t2, o2 = th + 0.5 * h * om, om + 0.5 * h * a1
            a2 = -w02 * sin(t2 - off) - c1 * o2 - c0 * tanh(o2 * inv_eps)
            t3, o3 = th + 0.5 * h * o2, om + 0.5 * h * a2
            a3 = -w02 * sin(t3 - off) - c1 * o3 - c0 * tanh(o3 * inv_eps)
            t4, o4 = th + h * o3, om + h * a3
            a4 = -w02 * sin(t4 - off) - c1 * o4 - c0 * tanh(o4 * inv_eps)
            th += h / 6.0 * (om + 2.0 * o2 + 2.0 * o3 + o4)
            om += h / 6.0 * (a1 + 2.0 * a2 + 2.0 * a3 + a4)
        out.append(th)
    return np.asarray(out)


LOWER = np.array([1e-3, 0.0, 0.0, -0.5, -10.0, -50.0])


def levenberg_marquardt(theta, t, p0, free, coulomb, max_iter=80):
    """Nonlinear least squares using Levenberg-Marquardt.

    np.linalg.lstsq solves the augmented linearized least-squares step. This
    avoids explicitly forming J.T @ J for the parameter update, reducing the
    numerical-conditioning penalty associated with normal equations.

    Returns fitted parameters, the final residual Jacobian, residuals, and the
    indices of the fitted parameters.
    """
    p = np.array(p0, dtype=float)
    idx = [i for i in range(6) if free[i]]

    def residual(q):
        return simulate(q, t, coulomb) - theta

    def jacobian(q, r0):
        J = np.zeros((len(t), len(idx)))
        scale = np.array([100.0, 1.0, 0.5, 0.01, 1.0, 0.1])
        for k, i in enumerate(idx):
            delta = 1e-5 * (abs(q[i]) + scale[i])
            q2 = q.copy()
            q2[i] += delta
            J[:, k] = (residual(q2) - r0) / delta
        return J

    r = residual(p)
    cost = float(r @ r)
    lam = 1e-3

    for _ in range(max_iter):
        J = jacobian(p, r)
        # Equivalent damped least-squares step to solving
        # (J.T @ J + lam * diag(diag(J.T @ J) + eps)) dp = -J.T @ r,
        # but using an augmented system avoids explicitly constructing J.T @ J.
        diag_h = np.sum(J * J, axis=0)
        damping = np.diag(np.sqrt(diag_h + 1e-12))
        J_aug = np.vstack((J, math.sqrt(lam) * damping))
        rhs_aug = np.concatenate((-r, np.zeros(len(idx))))

        improved = False
        for _ in range(12):
            dp, _, _, _ = np.linalg.lstsq(J_aug, rhs_aug, rcond=None)
            q = p.copy()
            q[idx] = np.maximum(p[idx] + dp, LOWER[idx])
            rq = residual(q)
            cq = float(rq @ rq)
            if cq < cost:
                rel = (cost - cq) / max(cost, 1e-30)
                p, r, cost = q, rq, cq
                lam = max(lam / 3.0, 1e-9)
                improved = True
                break
            lam *= 4.0
            J_aug[-len(idx):, :] = math.sqrt(lam) * damping

        if not improved or rel < 1e-10:
            break

    return p, jacobian(p, r), r, idx


# ---------------------------------------------------------------- data preparation

def find_release(t, th):
    """Index at the end of the first plateau at the largest absolute angle."""
    a = np.abs(th)
    peak = a.max()
    near = np.flatnonzero(a >= peak - 0.02)
    if len(near) == 0:
        return 0
    end = near[0]
    for i in near[1:]:
        if i - end > 10:
            break
        end = i
    return int(end)


def extrema(t, y, min_amp, min_gap=None):
    """Times and values of local extrema above min_amp on a lightly smoothed signal."""
    k = 5
    if len(y) <= 2 * k + 1:
        return np.array([]), np.array([])
    ys = np.convolve(y, np.ones(k) / k, mode="same")
    d = np.diff(ys)
    idx = np.flatnonzero(((d[:-1] > 0) & (d[1:] <= 0)) |
                         ((d[:-1] < 0) & (d[1:] >= 0))) + 1
    idx = idx[(idx > k) & (idx < len(y) - k)]
    kept = []
    for i in idx:
        if abs(ys[i]) <= min_amp:
            continue
        if min_gap and kept and t[i] - t[kept[-1]] < min_gap:
            if abs(ys[i]) > abs(ys[kept[-1]]):
                kept[-1] = i
            continue
        kept.append(i)
    return np.array([t[i] for i in kept]), np.array([ys[i] for i in kept])


def initial_guess(t, th):
    """Rough starting values: period from peaks and decay from their amplitudes."""
    off = float(np.median(th[-max(5, len(th) // 10):]))
    tp, yp = extrema(t, th - off, 0.03)
    w02, c1 = 100.0, 0.5
    if len(tp) >= 4:
        half_periods = np.diff(tp)
        T = 2.0 * float(np.median(half_periods))
        amp = float(np.max(np.abs(th - off)))
        w0 = 2.0 * math.pi / T * (1.0 + amp ** 2 / 16.0)
        w02 = w0 ** 2
        mask = np.abs(yp) > 0.05
        if mask.sum() >= 3:
            slope = np.polyfit(tp[mask], np.log(np.abs(yp[mask])), 1)[0]
            c1 = max(-2.0 * slope, 0.05)
    return np.array([w02, c1, 0.05, off, float(th[0]), 0.0])


def prepare(data, t_start, t_end):
    t, th = data[:, 0], data[:, 1]
    order = np.argsort(t)
    t, th = t[order], th[order]

    i0 = find_release(t, th) if t_start is None else int(np.searchsorted(t, t_start))
    i0 = min(max(i0, 0), len(t) - 1)
    t0 = t[i0]

    rest = float(np.median(th[-max(5, len(th) // 10):]))
    moving = np.flatnonzero(np.abs(th - rest) > 0.01)
    dt_median = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
    extra = int(0.5 / dt_median) if dt_median > 0 else 0
    i1 = len(t) if t_end is not None else int(min(len(t), (moving[-1] if len(moving) else len(t)) + extra))
    if t_end is not None:
        i1 = int(np.searchsorted(t, t_end))
    i1 = max(i0, min(i1, len(t)))

    tt, yy = t[i0:i1] - t0, th[i0:i1]
    if len(tt) > 3000:
        step = int(math.ceil(len(tt) / 3000.0))
        tt, yy = tt[::step], yy[::step]
    return tt, yy, t0


# ---------------------------------------------------------------- analysis

def analyse(data, args, label):
    tt, yy, t0 = prepare(data, args.t_start, args.t_end)
    if len(tt) < 50:
        sys.exit(f"{label}: not enough data after the release ({len(tt)} samples).")
    if np.any(np.diff(tt) <= 0):
        sys.exit(f"{label}: timestamps must be strictly increasing after sorting.")

    print(f"\n--- {label} ---")
    print(f"Release at t = {t0:.2f} s, starting angle {math.degrees(yy[0]):.1f} deg; "
          f"fitting {len(tt)} samples over {tt[-1]:.1f} s")

    p0 = initial_guess(tt, yy)
    free = [True, True, False, True, True, True]
    pv, Jv, rv, iv = levenberg_marquardt(
        yy, tt, np.r_[p0[:2], 0.0, p0[3:]], free, coulomb=False
    )
    rms_v = math.sqrt(float(rv @ rv) / len(tt))

    p_start = pv.copy()
    p_start[2] = max(0.05, 0.01 * pv[0])
    pf, Jf, rf, iff = levenberg_marquardt(yy, tt, p_start, [True] * 6, coulomb=True)
    rms_f = math.sqrt(float(rf @ rf) / len(tt))

    res = dict(t=tt, y=yy, pv=pv, pf=pf, rv=rv, rf=rf,
               rms_v=rms_v, rms_f=rms_f, label=label)
    w0 = math.sqrt(pf[0])
    print(f"Natural frequency (small amplitude)  w0 = {w0:.3f} rad/s   "
          f"(period {2 * math.pi / w0:.4f} s)")
    print("Viscous + Coulomb fit:")
    print(f"  w02 = G/J      = {pf[0]:.3f} 1/s^2")
    print(f"  c1  = b/J      = {pf[1]:.4f} 1/s")
    print(f"  c0  = tau_c/J  = {pf[2]:.4f} rad/s^2" +
          ("   (at the lower bound: no Coulomb friction detected)" if pf[2] <= 1e-6 else ""))
    print(f"  offset         = {math.degrees(pf[3]):.3f} deg")
    print(f"  RMS residual   = {math.degrees(rms_f):.4f} deg")
    print("Viscous-only fit (Coulomb friction forced to 0):")
    print(f"  w02 = {pv[0]:.3f},  c1 = b/J = {pv[1]:.4f} 1/s, "
          f"RMS residual = {math.degrees(rms_v):.4f} deg")
    gain = 100.0 * (rms_v - rms_f) / rms_v if rms_v > 0 else 0.0
    print(f"  Adding Coulomb friction lowers the RMS residual by {gain:.0f} %: "
          + ("it appears to be a substantial part of the damping." if gain > 15
             else "the damping appears mostly viscous by this comparison."))
    return res


def physical(res, args):
    J = None
    if args.inertia:
        J = args.inertia
    elif args.rod_mass is not None and args.point_mass is not None and args.length:
        J = (args.rod_mass / 3.0 + args.point_mass) * args.length ** 2
    if J is None:
        print("\nNo inertia given (--inertia, or --rod-mass --point-mass --length): the results above are per unit "
              "of inertia.\nMultiply c1 and c0 by J to get b (N*m*s/rad) and tau_c (N*m).")
        return None

    pf = res["pf"]
    b = pf[1] * J
    print(f"\nWith J = {J:.4e} kg*m^2:")
    print(f"  b     = {b:.3e} N*m*s/rad   (viscous friction; Coulomb friction fitted separately)")
    print(f"  tau_c = {pf[2] * J * 1e3:.3f} mN*m   (Coulomb friction)")
    print(f"  b (viscous-only fit) = {res['pv'][1] * J:.3e} N*m*s/rad "
          "(effective damping if Coulomb friction is ignored)")
    print(f"  G = w02 * J = {pf[0] * J * 1e3:.2f} mN*m")
    if args.rod_mass is not None and args.point_mass is not None and args.length:
        g_model = (args.rod_mass / 2.0 + args.point_mass) * args.length * args.gravity
        err = 100.0 * (pf[0] * J - g_model) / g_model
        print(f"  G from the masses and the length = {g_model * 1e3:.2f} mN*m "
              f"(the fit differs by {err:+.1f} %: "
              + ("consistent)." if abs(err) < 5
                 else "check the masses, the length and the center of mass)."))
    print("\nFor the pendulum sketches (mechanical friction):")
    print(f"    #define VISCOUS_FRICTION {b:.2e}f")
    print(f"    #define COULOMB_FRICTION {pf[2] * J:.2e}f")
    return J


def compare_braked(free, braked, J, args):
    d_c1 = braked["pf"][1] - free["pf"][1]
    print("\n================ Electrical damping ================")
    print(f"c1 free = {free['pf'][1]:.4f} 1/s,  c1 braked (driver at 0 V) = "
          f"{braked['pf'][1]:.4f} 1/s,  difference = {d_c1:.4f} 1/s")
    if J is None:
        print("Give the inertia to get the electrical damping in N*m*s/rad.")
        return
    b_el = d_c1 * J
    print(f"Electrical damping = {b_el:.3e} N*m*s/rad")
    if args.torque_const and args.ke:
        expected = args.torque_const * args.ke
        print(f"Expected k*Ke = {args.torque_const} * {args.ke} = {expected:.3e} N*m*s/rad  "
              f"(measured / expected = {b_el / expected:.2f})")
        print("A ratio far from 1 points to an error in the torque constant or in Ke (or in the inertia).")
    print("=====================================================")


# ---------------------------------------------------------------- plot

def plot(res_list, save_fig, show):
    fig, axes = plt.subplots(3, 1, figsize=(10, 9))
    ax1, ax2, ax3 = axes
    colors = ["tab:blue", "tab:red"]

    for res, c in zip(res_list, colors):
        t, y = res["t"], res["y"]
        lab = res["label"]
        ax1.plot(t, np.degrees(y), ".", ms=2, color=c, alpha=0.5, label=f"{lab}: data")
        ax1.plot(t, np.degrees(res["rf"] + y), "-", color=c, lw=1.2,
                 label=f"{lab}: viscous + Coulomb fit")
        ax1.plot(t, np.degrees(res["rv"] + y), "--", color="k", lw=0.8, alpha=0.7,
                 label=f"{lab}: viscous-only fit")
        ax2.plot(t, np.degrees(res["rf"]), "-", color=c, lw=0.8,
                 label=f"{lab}: viscous + Coulomb")
        ax2.plot(t, np.degrees(res["rv"]), "--", color="k", lw=0.6, alpha=0.6)

        off = res["pf"][3]
        gap = 0.6 * math.pi / math.sqrt(res["pf"][0])
        te, ye = extrema(t, y - off, 0.03, gap)
        tm, ym = extrema(t, (res["rf"] + y) - off, 0.03, gap)
        if len(te):
            ax3.semilogy(te, np.abs(ye), "o", color=c, ms=4, label=f"{lab}: peaks")
        if len(tm):
            ax3.semilogy(tm, np.abs(ym), "-", color=c, lw=1.0)

    ax1.set_ylabel("angle (deg)")
    ax1.set_title("Pendulum ring-down")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=7)
    ax2.set_ylabel("residual (deg)")
    ax2.grid(True, alpha=0.3)
    ax3.set_ylabel("|peak angle - offset| (rad)")
    ax3.set_xlabel("time since the release (s)")
    ax3.set_title("Peak amplitudes: a straight line = viscous damping, a concave curve = Coulomb friction",
                  fontsize=9)
    ax3.grid(True, which="both", alpha=0.3)
    ax3.legend(fontsize=7)
    fig.tight_layout()

    if save_fig:
        fig.savefig(save_fig, dpi=150)
        print(f"Figure saved to {save_fig}")
    if show:
        plt.show()
    else:
        plt.close(fig)


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="Pendulum ring-down: friction identification.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the Arduino (e.g. COM3, /dev/ttyUSB0)")
    src.add_argument("--csv", help="CSV file already saved")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", help="CSV file to write when capturing (default: ringdown_<date>.csv)")
    ap.add_argument("--braked", help="CSV of a run with BRAKED_MODE 1: also estimates electrical damping")
    ap.add_argument("--inertia", type=float, help="pendulum inertia J (kg*m^2)")
    ap.add_argument("--rod-mass", type=float, help="rod mass (kg), with --point-mass and --length")
    ap.add_argument("--point-mass", type=float, help="point mass (kg)")
    ap.add_argument("--length", type=float, help="length from shaft axis to point mass (m)")
    ap.add_argument("--gravity", type=float, default=GRAVITY)
    ap.add_argument("--torque-const", type=float, help="torque constant k (N*m/V), to check electrical damping")
    ap.add_argument("--ke", type=float, help="back-EMF constant Ke (V*s/rad), to check electrical damping")
    ap.add_argument("--t-start", type=float, help="time of release (s); default: detected")
    ap.add_argument("--t-end", type=float, help="end of fitted data (s); default: until pendulum rests")
    ap.add_argument("--save-fig", help="save the figure to this file (png, pdf...) ")
    ap.add_argument("--no-show", action="store_true", help="do not open the plot window")
    args = ap.parse_args()

    if args.port:
        data = capture(args.port, args.baud)
        if data.size == 0:
            sys.exit("No data received.")
        save_csv(args.out or datetime.now().strftime("ringdown_%Y%m%d_%H%M%S.csv"), data)
    else:
        data = load_csv(args.csv)
        if data.size == 0:
            sys.exit(f"No valid data rows in {args.csv}")

    if data.ndim != 2 or data.shape[1] != 2 or len(data) < 2:
        sys.exit("Input data must contain at least two valid time,angle rows.")

    free = analyse(data, args, "free" if args.braked else "run")
    J = physical(free, args)
    results = [free]

    if args.braked:
        braked_data = load_csv(args.braked)
        if braked_data.size == 0:
            sys.exit(f"No valid data rows in {args.braked}")
        braked = analyse(braked_data, args, "braked")
        results.append(braked)
        compare_braked(free, braked, J, args)

    print("\nThe fit is nonlinear and may depend on the starting guess and chosen time window. "
          "Compare repeat runs and inspect the residual plot.")
    plot(results, args.save_fig, not args.no_show)


if __name__ == "__main__":
    main()
