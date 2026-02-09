#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
df = pd.read_csv("vector_field_front_campus.csv")
# df = pd.read_csv("vector_field_utah.csv")


#2d scatter plot of elevation 


plt.figure(figsize=(6, 6))
# sc = plt.scatter(
#     df["lon"],
#     df["lat"],
#     c=df["elevation"],
#     cmap="terrain",
#     s=1
# )


# sc = plt.scatter(
#     df["lon"],
#     df["lat"],
#     c=df["elevation"],
#     cmap="terrain",
#     s=1,
#     vmin=df["elevation"].min(),
#     vmax=df["elevation"].max()
# )

min_elev = df["elevation"].min()
max_elev = df["elevation"].max()
print(f"Elevation range: {min_elev} - {max_elev} m")


# Use scatter plot with manual color scaling
sc = plt.scatter(
    df["lon"],
    df["lat"],
    c=df["elevation"],
    cmap="terrain",
    s=5,  # slightly bigger points for visibility
    vmin=min_elev,  # force the colormap to use the real min
    vmax=max_elev   # force the colormap to use the real max
)


print(len(df))
print(df["elevation"].min(), df["elevation"].max())
print(df.head())

plt.colorbar(sc, label="Elevation (m)")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Terrain Elevation Map")
plt.axis("equal")
plt.show()



