import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def plot():
    csv_path = Path("outputs/trajectory.csv")
    if not csv_path.exists():
        print("No trajectory data found.")
        return

    df = pd.DataFrame(pd.read_csv(csv_path))
    output_dir = Path("outputs")
    output_dir.mkdir(exist_ok=True)

    # 2D Path Plot (XY)
    plt.figure(figsize=(10, 8))
    plt.plot(df['x'], df['y'], label='Pen Tip Path')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Pen Tip 2D Trajectory (XY)')
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "pentip_xy.png")
    print(f"Saved XY plot to {output_dir / 'pentip_xy.png'}")

    # 3D Trajectory Plot (XYZ)
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(df['x'], df['y'], df['z'], label='Pen Tip 3D Path')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Pen Tip 3D Trajectory (XYZ)')
    plt.legend()
    plt.savefig(output_dir / "pentip_xyz.png")
    print(f"Saved XYZ plot to {output_dir / 'pentip_xyz.png'}")

    # Time series plot
    plt.figure(figsize=(12, 6))
    plt.plot(df['t'] - df['t'].iloc[0], df['x'], label='X')
    plt.plot(df['t'] - df['t'].iloc[0], df['y'], label='Y')
    plt.plot(df['t'] - df['t'].iloc[0], df['z'], label='Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Pen Tip Position vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(output_dir / "pentip_time.png")
    print(f"Saved time series plot to {output_dir / 'pentip_time.png'}")

if __name__ == "__main__":
    plot()
