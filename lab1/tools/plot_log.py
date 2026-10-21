#!/usr/bin/env python3
"""
Plot and summarize a pendulum run: from the Arduino (pendulum_pid / pendulum_lqr / pendulum_mpc)
or from the simulation (pendulum_sim). Both print the same CSV:

    t_s,theta_rad,omega_rad_s,torque_Nm,voltage_V,mode        (mode 0 = swing-up, 1 = balance)

Usage
    # capture a run from the Arduino (saves the CSV), then plot it
    python plot_log.py --port COM3
    python plot_log.py --port /dev/ttyUSB0 --out run.csv

    # plot a CSV that was already saved (hardware or simulation)
    python plot_log.py --csv run.csv

Requirements:  pip install numpy matplotlib pyserial   (pyserial only for --port)
"""

import argparse
import sys
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt

HEADER = "t_s,theta_rad,omega_rad_s,torque_Nm,voltage_V,mode"


def parse_row(line):
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
        print(f"Listening on {port} at {baud} baud (Ctrl+C stops the motor and ends the capture)...")
        try:
            while True:
                line = ser.readline().decode(errors="replace").strip()
                if not line:
                    continue

                if line.startswith("#READY") and not started:
                    print(line)
                    input(">>> Pendulum hanging at rest and the area clear? Press Enter to start... ")
                    ser.write(b"s")
                    started = True
                    continue

                row = parse_row(line)
                if row is not None:
                    rows.append(row)
                    continue

                print(line)   # header and messages from the Arduino
                if line.startswith("#DONE") or line.startswith("#ABORTED"):
                    # let the final messages arrive
                    for _ in range(3):
                        extra = ser.readline().decode(errors="replace").strip()
                        if extra:
                            print(extra)
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
            f.write(f"{r[0]:.3f},{r[1]:.4f},{r[2]:.3f},{r[3]:.5f},{r[4]:.3f},{int(r[5])}\n")
    print(f"Data saved to {path}")


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def summary(d):
    t, theta, omega, tau, volt, mode = d.T
    phi = wrap(theta - np.pi)
    balance = mode > 0.5

    print("\n================ Run summary ================")
    print(f"Duration: {t[-1] - t[0]:.2f} s, {len(t)} samples")
    if not balance.any():
        print("The pendulum was never captured (the balance controller never started).")
        print(f"Largest angle reached: {np.degrees(np.max(np.abs(theta))):.0f} deg "
              f"(upright is {np.degrees(np.pi):.0f} deg), energy pumping peak speed {np.max(np.abs(omega)):.1f} rad/s")
    else:
        i0 = int(np.argmax(balance))
        falls = int(np.sum((mode[1:] < 0.5) & (mode[:-1] > 0.5)))
        print(f"First capture at {t[i0]:.2f} s   (|phi| = {np.degrees(abs(phi[i0])):.1f} deg, |omega| = {abs(omega[i0]):.2f} rad/s)")
        print(f"Times the balance was lost and the swing-up restarted: {falls}")
        print(f"Time in balance mode: {np.sum(balance) / len(balance) * 100:.0f} % of the run")
        tail = balance & (t > t[-1] - 5.0)
        if tail.any():
            print(f"Last 5 s in balance mode: RMS angle error {np.degrees(np.sqrt(np.mean(phi[tail] ** 2))):.2f} deg, "
                  f"max {np.degrees(np.max(np.abs(phi[tail]))):.2f} deg, RMS torque {np.sqrt(np.mean(tau[tail] ** 2)) * 1e3:.2f} mN*m")
    print(f"Peak |torque| {np.max(np.abs(tau)) * 1e3:.2f} mN*m, peak |voltage| {np.max(np.abs(volt)):.2f} V")
    print("=============================================\n")


def plot(d, save_fig, show):
    t, theta, omega, tau, volt, mode = d.T
    phi = wrap(theta - np.pi)
    balance = mode > 0.5

    fig = plt.figure(figsize=(11, 8.5))
    gs = fig.add_gridspec(3, 2, width_ratios=[2.2, 1])
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[1, 0], sharex=ax1)
    ax3 = fig.add_subplot(gs[2, 0], sharex=ax1)
    ax4 = fig.add_subplot(gs[:, 1])

    def shade(ax):
        # shade the intervals in balance mode
        edges = np.flatnonzero(np.diff(np.concatenate(([0], balance.astype(int), [0]))))
        for a, b in zip(edges[::2], edges[1::2]):
            ax.axvspan(t[a], t[min(b, len(t) - 1)], color="tab:green", alpha=0.12, lw=0)

    ax1.plot(t, theta, color="tab:blue")
    # upright = every odd multiple of pi (the pendulum can go up either way)
    k_lo = int(np.floor((theta.min() - 0.5) / (2.0 * np.pi)))
    k_hi = int(np.ceil((theta.max() + 0.5) / (2.0 * np.pi)))
    for k in range(k_lo, k_hi + 1):
        ax1.axhline((2 * k + 1) * np.pi, color="k", ls="--", lw=0.8)
    ax1.set_ylabel("theta (rad)")
    ax1.set_title("Pendulum run (green = balance mode, dashed = upright positions)")
    ax1.grid(True, alpha=0.3)
    shade(ax1)

    ax2.plot(t, omega, color="tab:orange")
    ax2.set_ylabel("omega (rad/s)")
    ax2.grid(True, alpha=0.3)
    shade(ax2)

    ax3.plot(t, tau * 1e3, color="tab:red", label="torque (mN·m)")
    ax3.set_ylabel("torque (mN·m)")
    ax3.set_xlabel("time (s)")
    ax3.grid(True, alpha=0.3)
    shade(ax3)
    ax3b = ax3.twinx()
    ax3b.plot(t, volt, color="tab:purple", lw=0.8, alpha=0.7)
    ax3b.set_ylabel("voltage (V)", color="tab:purple")

    ax4.plot(phi[~balance], omega[~balance], ".", ms=2, color="tab:blue", label="swing-up")
    ax4.plot(phi[balance], omega[balance], ".", ms=2, color="tab:green", label="balance")
    ax4.plot([0], [0], "k+", ms=12)
    ax4.set_xlabel("phi = theta - pi (rad)")
    ax4.set_ylabel("omega (rad/s)")
    ax4.set_title("Phase plane around the top")
    ax4.grid(True, alpha=0.3)
    ax4.legend(markerscale=4)

    fig.tight_layout()
    if save_fig:
        fig.savefig(save_fig, dpi=150)
        print(f"Figure saved to {save_fig}")
    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description="Capture, summarize and plot a pendulum run.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="serial port of the Arduino (e.g. COM3, /dev/ttyUSB0)")
    src.add_argument("--csv", help="CSV file already saved (hardware or simulation)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", help="CSV file to write when capturing (default: pendulum_run_<date>.csv)")
    ap.add_argument("--save-fig", help="save the figure to this file (png, pdf...)")
    ap.add_argument("--no-show", action="store_true", help="do not open the plot window")
    args = ap.parse_args()

    if args.port:
        data = capture(args.port, args.baud)
        if data.size == 0:
            sys.exit("No data received.")
        save_csv(args.out or datetime.now().strftime("pendulum_run_%Y%m%d_%H%M%S.csv"), data)
    else:
        data = load_csv(args.csv)
        if data.size == 0:
            sys.exit(f"No valid data rows in {args.csv}")

    summary(data)
    plot(data, args.save_fig, not args.no_show)


if __name__ == "__main__":
    main()
