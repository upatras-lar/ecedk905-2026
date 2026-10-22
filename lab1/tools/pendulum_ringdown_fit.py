#!/usr/bin/env python3
"""
Pendulum ring-down fit for mechanical friction; optionally compare a free run with
an electrically braked run.

CSV columns:
    t_s,theta_rad

Model:
    theta'' = -w02*sin(theta - off) - c1*theta' - c0*tanh(theta'/COULOMB_EPS)

Parameters:
    w02 = G/J       (1/s^2)
    c1  = b/J       (1/s), viscous damping per unit inertia
    c0  = tau_c/J   (rad/s^2), smoothed Coulomb-friction term per unit inertia
    off             (rad), equilibrium-angle offset

The fit is nonlinear. A bounded Levenberg-Marquardt-style least-squares optimizer
solves each linearized step with np.linalg.lstsq. No confidence intervals are
calculated. The input angle must be continuous/unwrapped (no +/-pi jumps).

Usage:
    python pendulum_ringdown_fit.py --port COM3 --rod-mass 0.005 --point-mass 0.020 --length 0.100
    python pendulum_ringdown_fit.py --csv free.csv --inertia 2.167e-4
    python pendulum_ringdown_fit.py --csv free.csv --braked braked.csv --inertia 2.167e-4 \
        --torque-per-volt 0.0123 --ke 0.035

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
COULOMB_EPS = 0.05  # rad/s, tanh smoothing scale for Coulomb friction
STEP_MAX = 0.005    # seconds, maximum RK4 integration step

# Broad physical bounds prevent numerically nonsensical parameter excursions while
# allowing normal pendulum setups and unwrapped sensor angles.
LOWER = np.array([1e-3, 0.0, 0.0, -math.pi, -40.0 * math.pi, -200.0])
UPPER = np.array([1e5, 100.0, 100.0, math.pi, 40.0 * math.pi, 200.0])


def parse_row(line):
    parts = line.strip().split(",")
    if len(parts) != 2:
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
                    input(">>> Pendulum hanging at rest? Press Enter to start; then do not touch it... ")
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
            f.write(f"{r[0]:.6f},{r[1]:.7f}\n")
    print(f"Data saved to {path}")


def clean_data(data, label):
    """Drop invalid rows, sort by time, and average duplicate timestamps."""
    data = np.asarray(data, dtype=float)
    if data.ndim != 2 or data.shape[1] != 2:
        raise ValueError(f"{label}: data must have columns t_s,theta_rad")
    good = np.all(np.isfinite(data), axis=1)
    dropped = int(len(data) - np.count_nonzero(good))
    if dropped:
        print(f"{label}: dropping {dropped} non-finite row(s).")
    data = data[good]
    if len(data) < 2:
        raise ValueError(f"{label}: fewer than two finite samples remain")
    data = data[np.argsort(data[:, 0])]
    unique_t, inv = np.unique(data[:, 0], return_inverse=True)
    if len(unique_t) != len(data):
        sums = np.zeros(len(unique_t), dtype=float)
        counts = np.zeros(len(unique_t), dtype=int)
        np.add.at(sums, inv, data[:, 1])
        np.add.at(counts, inv, 1)
        data = np.column_stack((unique_t, sums / counts))
        print(f"{label}: averaged duplicate timestamps; {len(data)} unique samples remain.")
    if len(data) < 2 or np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError(f"{label}: timestamps must be strictly increasing")
    return data


def simulate(p, t, coulomb=True):
    """Simulate theta at sample times t for p=[w02,c1,c0,off,theta0,omega0] via RK4."""
    w02, c1, c0, off, th, om = p
    if not coulomb:
        c0 = 0.0
    out = [th]
    inv_eps = 1.0 / COULOMB_EPS

    def accel(theta, omega):
        return (-w02 * math.sin(theta - off)
                - c1 * omega
                - c0 * math.tanh(omega * inv_eps))

    for i in range(1, len(t)):
        dt = float(t[i] - t[i - 1])
        if dt < 0:
            raise ValueError("simulation timestamps must be sorted")
        n = max(1, int(math.ceil(dt / STEP_MAX)))
        h = dt / n
        for _ in range(n):
            a1 = accel(th, om)
            t2, o2 = th + 0.5 * h * om, om + 0.5 * h * a1
            a2 = accel(t2, o2)
            t3, o3 = th + 0.5 * h * o2, om + 0.5 * h * a2
            a3 = accel(t3, o3)
            t4, o4 = th + h * o3, om + h * a3
            a4 = accel(t4, o4)
            th += h / 6.0 * (om + 2.0 * o2 + 2.0 * o3 + o4)
            om += h / 6.0 * (a1 + 2.0 * a2 + 2.0 * a3 + a4)
        out.append(th)
    return np.asarray(out)


def levenberg_marquardt(theta, t, p0, free, coulomb, max_iter=80):
    """Bounded nonlinear least-squares fit using damped, augmented lstsq steps."""
    p = np.clip(np.asarray(p0, dtype=float), LOWER, UPPER)
    idx = [i for i in range(6) if free[i]]
    if not idx:
        raise ValueError("at least one parameter must be free")

    def residual(q):
        return simulate(q, t, coulomb) - theta

    scale = np.array([100.0, 1.0, 0.5, 0.01, 1.0, 0.1])

    def jacobian(q, r0):
        jac = np.zeros((len(t), len(idx)), dtype=float)
        for k, i in enumerate(idx):
            delta = 1e-5 * (abs(q[i]) + scale[i])
            q2 = q.copy()
            if q[i] + delta <= UPPER[i]:
                q2[i] += delta
                denom = delta
            elif q[i] - delta >= LOWER[i]:
                q2[i] -= delta
                denom = -delta
            else:
                # Parameter is boxed in too tightly for a finite difference.
                continue
            jac[:, k] = (residual(q2) - r0) / denom
        return jac

    r = residual(p)
    cost = float(r @ r)
    lam = 1e-3
    rel = float("inf")

    for _ in range(max_iter):
        jac = jacobian(p, r)
        diag_h = np.sum(jac * jac, axis=0)
        damping = np.diag(np.sqrt(diag_h + 1e-12))
        rhs = np.concatenate((-r, np.zeros(len(idx))))
        improved = False

        for _ in range(12):
            jac_aug = np.vstack((jac, math.sqrt(lam) * damping))
            dp, _, _, _ = np.linalg.lstsq(jac_aug, rhs, rcond=None)
            candidate = p.copy()
            candidate[idx] = np.clip(p[idx] + dp, LOWER[idx], UPPER[idx])
            rc = residual(candidate)
            candidate_cost = float(rc @ rc)
            if candidate_cost < cost:
                rel = (cost - candidate_cost) / max(cost, 1e-30)
                p, r, cost = candidate, rc, candidate_cost
                lam = max(lam / 3.0, 1e-9)
                improved = True
                break
            lam = min(lam * 4.0, 1e12)

        if not improved or rel < 1e-10:
            break

    return p, jacobian(p, r), r, idx


def find_release(t, th):
    """Find the end of the first near-maximum-angle plateau, relative to final rest."""
    tail = th[-max(5, len(th) // 10):]
    rest = float(np.median(tail))
    amp = np.abs(th - rest)
    peak = float(np.max(amp))
    if not np.isfinite(peak) or peak < 1e-6:
        return 0
    near = np.flatnonzero(amp >= peak - max(0.02, 0.02 * peak))
    if len(near) == 0:
        return 0
    end = int(near[0])
    for idx in near[1:]:
        if idx - end > 10:
            break
        end = int(idx)
    return end


def extrema(t, y, min_amp, min_gap=None):
    """Times/values of local extrema in a lightly smoothed signal."""
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
                kept[-1] = int(i)
            continue
        kept.append(int(i))
    return np.array([t[i] for i in kept]), np.array([ys[i] for i in kept])


def initial_guess(t, th):
    """Estimate starting values from the late-time rest offset and peak spacing."""
    off = float(np.median(th[-max(5, len(th) // 10):]))
    tp, yp = extrema(t, th - off, 0.03)
    w02, c1 = 100.0, 0.5
    if len(tp) >= 4:
        half_periods = np.diff(tp)
        good = half_periods[half_periods > 0]
        if len(good):
            period = 2.0 * float(np.median(good))
            amplitude = float(np.max(np.abs(th - off)))
            w0 = 2.0 * math.pi / period * (1.0 + amplitude**2 / 16.0)
            w02 = w0**2
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
    t0 = float(t[i0])

    rest = float(np.median(th[-max(5, len(th) // 10):]))
    moving_idx = np.flatnonzero(np.abs(th - rest) > 0.01)
    dt_median = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
    extra = int(0.5 / dt_median) if dt_median > 0 else 0
    if t_end is not None:
        i1 = int(np.searchsorted(t, t_end))
    else:
        last_moving = int(moving_idx[-1]) if len(moving_idx) else len(t) - 1
        i1 = min(len(t), last_moving + extra + 1)
    i1 = max(i0, min(i1, len(t)))

    tt, yy = t[i0:i1] - t0, th[i0:i1]
    if len(tt) > 3000:
        step = int(math.ceil(len(tt) / 3000.0))
        tt, yy = tt[::step], yy[::step]
    return tt, yy, t0


def analyse(data, args, label):
    tt, yy, t0 = prepare(data, args.t_start, args.t_end)
    if len(tt) < 50:
        raise ValueError(f"{label}: not enough data after release ({len(tt)} samples; need at least 50).")
    if np.any(np.diff(tt) <= 0):
        raise ValueError(f"{label}: timestamps must be strictly increasing after preparation.")

    print(f"\n--- {label} ---")
    print(f"Release at t = {t0:.3f} s, starting angle {math.degrees(yy[0]):.1f} deg; "
          f"fitting {len(tt)} samples over {tt[-1]:.2f} s")

    p0 = initial_guess(tt, yy)
    free_viscous = [True, True, False, True, True, True]
    pv, _, rv, _ = levenberg_marquardt(
        yy, tt, np.r_[p0[:2], 0.0, p0[3:]], free_viscous, coulomb=False
    )
    rms_v = math.sqrt(float(rv @ rv) / len(tt))

    p_start = pv.copy()
    p_start[2] = max(0.01, min(1.0, 0.05 * math.sqrt(pv[0])))
    pf, _, rf, _ = levenberg_marquardt(yy, tt, p_start, [True] * 6, coulomb=True)
    rms_f = math.sqrt(float(rf @ rf) / len(tt))

    res = dict(t=tt, y=yy, pv=pv, pf=pf, rv=rv, rf=rf,
               rms_v=rms_v, rms_f=rms_f, label=label)
    w0 = math.sqrt(pf[0])
    print(f"Natural frequency (small amplitude)  w0 = {w0:.3f} rad/s   "
          f"(period {2 * math.pi / w0:.4f} s)")
    print("Viscous + Coulomb fit:")
    print(f"  w02 = G/J      = {pf[0]:.5f} 1/s^2")
    print(f"  c1  = b/J      = {pf[1]:.5f} 1/s")
    print(f"  c0  = tau_c/J  = {pf[2]:.5f} rad/s^2" +
          ("   (at lower bound: no Coulomb term resolved)" if pf[2] <= 1.01 * LOWER[2] else ""))
    print(f"  offset         = {math.degrees(pf[3]):.3f} deg")
    print(f"  RMS residual   = {math.degrees(rms_f):.4f} deg")
    print("Viscous-only fit (Coulomb friction fixed to zero):")
    print(f"  w02 = {pv[0]:.5f}, c1 = {pv[1]:.5f} 1/s, RMS residual = {math.degrees(rms_v):.4f} deg")
    gain = 100.0 * (rms_v - rms_f) / rms_v if rms_v > 0 else 0.0
    if gain >= 0:
        print(f"  Coulomb term reduces RMS residual by {gain:.1f} %. This alone does not prove Coulomb friction is present.")
    else:
        print(f"  Coulomb term increases RMS residual by {-gain:.1f} %; inspect fit convergence and the selected time window.")
    return res


def physical(res, args):
    if args.inertia is not None:
        inertia = args.inertia
    elif args.rod_mass is not None and args.point_mass is not None and args.length is not None:
        inertia = (args.rod_mass / 3.0 + args.point_mass) * args.length**2
    else:
        inertia = None

    if inertia is None:
        print("\nNo inertia given (--inertia, or --rod-mass --point-mass --length): results above are per unit inertia.")
        print("Multiply c1 and c0 by J to estimate b (N*m*s/rad) and tau_c (N*m).")
        return None

    pf = res["pf"]
    b = pf[1] * inertia
    tau_c = pf[2] * inertia
    print(f"\nWith J = {inertia:.4e} kg*m^2:")
    print(f"  b     = {b:.4e} N*m*s/rad (viscous friction coefficient)")
    print(f"  tau_c = {tau_c * 1e3:.4f} mN*m (Coulomb-friction torque parameter)")
    print(f"  b (viscous-only fit) = {res['pv'][1] * inertia:.4e} N*m*s/rad")
    print(f"  G = w02*J = {pf[0] * inertia * 1e3:.3f} mN*m")
    if args.rod_mass is not None and args.point_mass is not None and args.length is not None:
        g_model = (args.rod_mass / 2.0 + args.point_mass) * args.length * args.gravity
        err = 100.0 * (pf[0] * inertia - g_model) / g_model if g_model > 0 else float("nan")
        print(f"  G from masses and geometry = {g_model * 1e3:.3f} mN*m "
              f"(fit difference {err:+.1f} %: "
              + ("consistent within 5%)." if abs(err) < 5 else "check masses, length, geometry, and fit)."))
    print("\nFor the pendulum sketch (mechanical friction parameters):")
    print(f"    #define VISCOUS_FRICTION {b:.4e}f")
    print(f"    #define COULOMB_FRICTION {tau_c:.4e}f")
    return inertia


def compare_braked(free, braked, inertia, args):
    d_c1 = braked["pf"][1] - free["pf"][1]
    print("\n================ Electrical damping comparison ================")
    print(f"c1 free = {free['pf'][1]:.5f} 1/s; c1 braked (driver at 0 V) = {braked['pf'][1]:.5f} 1/s")
    print(f"Difference = {d_c1:+.5f} 1/s")
    if inertia is None:
        print("Supply --inertia (or the mass/length inputs) to express the difference in N*m*s/rad.")
        print("===============================================================")
        return
    b_el = d_c1 * inertia
    print(f"Apparent added damping = {b_el:+.4e} N*m*s/rad")
    if b_el < 0:
        print("WARNING: braked run has less fitted damping than free run; check run consistency and fit residuals.")
    if args.torque_per_volt is not None and args.ke is not None:
        expected = args.torque_per_volt * args.ke
        ratio = b_el / expected if expected != 0 else float("nan")
        print(f"Reference k*Ke,eff = {args.torque_per_volt:.6g} * {args.ke:.6g} = {expected:.4e} N*m*s/rad")
        print(f"Measured/reference ratio = {ratio:.3f}")
        print("This comparison is only meaningful if k and Ke use the same commanded-voltage convention.")
    print("The result depends on what '0 V' means electrically for your driver; it may not mean open-circuit.")
    print("===============================================================\n")


def plot(res_list, save_fig, show):
    fig, axes = plt.subplots(3, 1, figsize=(10, 9))
    colors = ["tab:blue", "tab:red"]
    ax1, ax2, ax3 = axes
    for res, color in zip(res_list, colors):
        t, y, label = res["t"], res["y"], res["label"]
        pred_f = y + res["rf"]
        pred_v = y + res["rv"]
        ax1.plot(t, np.degrees(y), ".", ms=2, color=color, alpha=0.5, label=f"{label}: data")
        ax1.plot(t, np.degrees(pred_f), "-", color=color, lw=1.2, label=f"{label}: viscous + Coulomb fit")
        ax1.plot(t, np.degrees(pred_v), "--", color="k", lw=0.8, alpha=0.7, label=f"{label}: viscous-only fit")
        ax2.plot(t, np.degrees(res["rf"]), "-", color=color, lw=0.8, label=f"{label}: viscous + Coulomb")
        ax2.plot(t, np.degrees(res["rv"]), "--", color="k", lw=0.6, alpha=0.6)

        offset = res["pf"][3]
        gap = 0.6 * math.pi / math.sqrt(res["pf"][0])
        te, ye = extrema(t, y - offset, 0.03, gap)
        tm, ym = extrema(t, pred_f - offset, 0.03, gap)
        if len(te):
            ax3.semilogy(te, np.abs(ye), "o", color=color, ms=4, label=f"{label}: measured peaks")
        if len(tm):
            ax3.semilogy(tm, np.abs(ym), "-", color=color, lw=1.0, label=f"{label}: model peaks")

    ax1.set_ylabel("Angle (deg)")
    ax1.set_title("Pendulum ring-down fit")
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=7)
    ax2.set_ylabel("Fit residual (deg)")
    ax2.grid(True, alpha=0.3)
    ax3.set_ylabel("Peak amplitude (rad)")
    ax3.set_xlabel("Time since fitted start (s)")
    ax3.set_title("Peak envelope: inspect shape and fit agreement", fontsize=9)
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


def main():
    ap = argparse.ArgumentParser(description="Pendulum ring-down: fit mechanical friction and compare electrical braking.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the Arduino (e.g. COM3, /dev/ttyUSB0)")
    src.add_argument("--csv", help="CSV file already saved")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", help="CSV output path when capturing")
    ap.add_argument("--braked", help="CSV file from a braked run (also estimates added damping)")
    ap.add_argument("--inertia", type=float, help="pendulum inertia J (kg*m^2)")
    ap.add_argument("--rod-mass", type=float, help="rod mass (kg), with --point-mass and --length")
    ap.add_argument("--point-mass", type=float, help="point mass (kg)")
    ap.add_argument("--length", type=float, help="shaft-to-point-mass distance (m)")
    ap.add_argument("--gravity", type=float, default=GRAVITY)
    ap.add_argument("--torque-per-volt", "--torque-const", dest="torque_per_volt", type=float,
                    help="torque-vs-commanded-voltage slope k (N*m/V), not Kt in N*m/A")
    ap.add_argument("--ke", type=float, help="effective commanded-voltage/speed constant (V*s/rad); must match k convention")
    ap.add_argument("--t-start", type=float, help="absolute time of fitted start (s); default: detect release")
    ap.add_argument("--t-end", type=float, help="absolute timestamp where fitting stops (s); default: near-rest end")
    ap.add_argument("--save-fig", help="save plot to this image file")
    ap.add_argument("--no-show", action="store_true", help="do not open plot window")
    args = ap.parse_args()

    if args.inertia is not None and args.inertia <= 0:
        ap.error("--inertia must be positive")
    mass_args = (args.rod_mass, args.point_mass, args.length)
    if any(x is not None for x in mass_args) and not all(x is not None for x in mass_args):
        ap.error("provide all of --rod-mass, --point-mass and --length together")
    if all(x is not None for x in mass_args):
        if args.rod_mass < 0 or args.point_mass < 0 or args.length <= 0:
            ap.error("masses must be nonnegative and --length must be positive")
        if (args.rod_mass / 3.0 + args.point_mass) * args.length**2 <= 0:
            ap.error("the supplied masses and length imply zero inertia")
    if args.gravity <= 0:
        ap.error("--gravity must be positive")
    if args.torque_per_volt is not None and args.torque_per_volt <= 0:
        ap.error("--torque-per-volt must be positive")
    if args.ke is not None and args.ke <= 0:
        ap.error("--ke must be positive")
    if (args.torque_per_volt is None) != (args.ke is None):
        ap.error("provide both --torque-per-volt and --ke, or neither")
    if args.t_start is not None and args.t_end is not None and args.t_end <= args.t_start:
        ap.error("--t-end must be greater than --t-start")

    if args.port:
        data = capture(args.port, args.baud)
        if data.size == 0:
            sys.exit("No data received.")
        save_csv(args.out or datetime.now().strftime("ringdown_%Y%m%d_%H%M%S.csv"), data)
    else:
        data = load_csv(args.csv)
        if data.size == 0:
            sys.exit(f"No valid data rows in {args.csv}")

    try:
        data = clean_data(data, "free/run" if args.braked else "run")
        fit_label = "free" if args.braked else "run"
        free = analyse(data, args, fit_label)
        inertia = physical(free, args)
        results = [free]

        if args.braked:
            braked_data = load_csv(args.braked)
            if braked_data.size == 0:
                sys.exit(f"No valid data rows in {args.braked}")
            braked_data = clean_data(braked_data, "braked")
            braked = analyse(braked_data, args, "braked")
            results.append(braked)
            compare_braked(free, braked, inertia, args)
    except ValueError as exc:
        sys.exit(f"ERROR: {exc}")

    print("\nThe fit can depend on initial guesses and time-window selection. Inspect residual plots and repeat runs.")
    plot(results, args.save_fig, not args.no_show)


if __name__ == "__main__":
    main()
