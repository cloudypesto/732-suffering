#!/usr/bin/env python3
import csv
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

INPUT_CSV = 'odom_square_t5.csv'
OUTPUT_PLOT = 'odom_square_t5.png'

def make_square(start_x, start_y, theta):
    """1m square starting at (start_x, start_y), rotated by theta."""
    
    # square in local frame (robot forward direction)
    square = [
        (0, 0),
        (1, 0),
        (1, -1),
        (0, -1),
        (0, 0)
    ]

    rx, ry = [], []

    for x, y in square:
        # rotate
        xr = x * math.cos(theta) - y * math.sin(theta)
        yr = x * math.sin(theta) + y * math.cos(theta)

        # translate to start position
        rx.append(xr + start_x)
        ry.append(yr + start_y)

    return rx, ry


# --- Load CSV ---
data = []
with open(INPUT_CSV, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        data.append({
            'time': float(row['time']),
            'x': float(row['x']),
            'y': float(row['y']),
            'yaw_deg': float(row['yaw_deg']),
            'vel_lin': float(row['vel_lin']),
            'vel_ang': float(row['vel_ang']),
        })

if not data:
    print("No data found.")
    exit()

# --- Extract ---
times   = [r['time'] for r in data]
xs      = [r['x'] for r in data]
ys      = [r['y'] for r in data]
yaws    = [r['yaw_deg'] for r in data]
vel_lin = [r['vel_lin'] for r in data]

# --- Start/end (RAW, no modification) ---
x_start, y_start = xs[0], ys[0]
x_end, y_end     = xs[-1], ys[-1]

# --- Closing error (RAW space) ---
closing = math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)

# --- Use initial heading ONLY for ideal square orientation ---
theta0 = math.radians(yaws[0])
square_x, square_y = make_square(x_start, y_start, theta0)

# --- Plot ---
fig = plt.figure(figsize=(16, 10))
fig.suptitle('Odometry — Square Path (Raw Data) | TurtleBot 4',
             fontsize=14, fontweight='bold')

# --- Trajectory ---
ax_traj = fig.add_subplot(1, 2, 1)
ax_traj.plot(xs, ys, color='#2980B9', linewidth=2, label='Estimated path')

# Start / End markers (RAW coordinates)
ax_traj.plot(x_start, y_start, 'go', markersize=12,
             label=f'Start ({x_start:.3f}, {y_start:.3f})', zorder=5)

ax_traj.plot(x_end, y_end, 'r^', markersize=12,
             label=f'End ({x_end:.3f}, {y_end:.3f})', zorder=5)

# Ideal square (rotated to match initial heading)
ax_traj.plot(square_x, square_y,
             linestyle='--',
             color='gray',
             linewidth=1.2,
             label='Ideal 1m square (aligned)')

# Closing error line
ax_traj.plot([x_end, x_start], [y_end, y_start],
             'r--', linewidth=1, alpha=0.7,
             label=f'Closing error: {closing:.4f}m')

ax_traj.set_title(f'XY Trajectory\nClosing error: {closing:.4f} m', fontsize=12)
ax_traj.set_xlabel('X position (metres)', fontsize=11)
ax_traj.set_ylabel('Y position (metres)', fontsize=11)
ax_traj.legend(fontsize=9)
ax_traj.grid(True, alpha=0.3)
ax_traj.set_aspect('equal')

# --- Bounds (raw data, no artificial padding from square) ---
margin = 0.3
ax_traj.set_xlim(min(xs + square_x) - margin, max(xs + square_x) + margin)
ax_traj.set_ylim(min(ys + square_y) - margin, max(ys + square_y) + margin)

# --- Velocity ---
ax_vel = fig.add_subplot(2, 2, 2)
ax_vel.plot(times, vel_lin, color='#E74C3C', linewidth=1.2)
ax_vel.axhline(y=0, color='gray', linestyle='--', linewidth=0.8)
ax_vel.set_ylabel('Linear Velocity (m/s)', fontsize=10)
ax_vel.set_title('Linear Velocity over Time', fontsize=11)
ax_vel.grid(True, alpha=0.3)

# --- Heading ---
ax_yaw = fig.add_subplot(2, 2, 4)
ax_yaw.plot(times, yaws, color='#27AE60', linewidth=1.2)
ax_yaw.axhline(y=0, color='gray', linestyle='--', linewidth=0.8)
ax_yaw.set_ylabel('Heading (degrees)', fontsize=10)
ax_yaw.set_xlabel('Time (seconds)', fontsize=10)
ax_yaw.set_title('Heading Angle over Time', fontsize=11)
ax_yaw.grid(True, alpha=0.3)

# --- Footer ---
fig.text(0.01, 0.01,
         f"Samples: {len(data)}  |  Duration: {times[-1]:.1f}s",
         fontsize=8, color='gray')

plt.tight_layout(rect=[0, 0.03, 1, 1])
plt.savefig(OUTPUT_PLOT, dpi=150, bbox_inches='tight')
plt.close()

print(f"Raw plot saved: {OUTPUT_PLOT}")