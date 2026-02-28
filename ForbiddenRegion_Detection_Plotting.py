import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
import time

polygon_sizes = [10, 100, 1000, 5000, 10000]
trials_per_size = 100

average_times = []
all_times = []
labels = []


dummy_poly = Polygon([(np.cos(i * 2 * np.pi / 10), np.sin(i * 2 * np.pi / 10)) for i in range(10)])
dummy_point = Point(0, 0)
_ = dummy_poly.contains(dummy_point)


for n in polygon_sizes:
    detection_times = []
    for _ in range(trials_per_size):
        polygon = Polygon([(np.cos(i * 2 * np.pi / n), np.sin(i * 2 * np.pi / n)) for i in range(n)])
        point = Point(0, 0)  # Point at origin

        start_time = time.perf_counter()
        _ = polygon.contains(point)
        end_time = time.perf_counter()

        detection_times.append(end_time - start_time)

    avg_time = np.mean(detection_times)
    average_times.append(avg_time)
    all_times.append(detection_times)
    labels.append(f"{n} vertices")


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

ax1.plot(polygon_sizes, average_times, marker='o')
ax1.set_xscale('log')
ax1.set_xlabel("Number of Vertices (log scale)")
ax1.set_ylabel("Avg. Detection Time per Test (s)")
ax1.set_title("Average Detection Time vs Number of Vertices")

ax2.boxplot(all_times, labels=labels)
ax2.set_ylabel("Detection Time per Test (s)")
ax2.set_xlabel("Polygon Size")
ax2.set_title("Boxplot of Detection Time per Polygon Size")

plt.tight_layout()
plt.show()

