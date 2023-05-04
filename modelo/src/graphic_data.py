from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd
import json
import matplotlib.colors as mcolors
from matplotlib.patches import Rectangle
def plot_colortable(colors, sort_colors=True, emptycols=0):

    cell_width = 212
    cell_height = 22
    swatch_width = 48
    margin = 12

    # Sort colors by hue, saturation, value and name.
    if sort_colors is True:
        by_hsv = sorted((tuple(mcolors.rgb_to_hsv(mcolors.to_rgb(color))),
                        name)
                        for name, color in colors.items())
        names = [name for hsv, name in by_hsv]
    else:
        names = list(colors)

    n = len(names)
    ncols = 4 - emptycols
    nrows = n // ncols + int(n % ncols > 0)

    width = cell_width * 4 + 2 * margin
    height = cell_height * nrows + 2 * margin
    dpi = 72

    fig, ax = plt.subplots(figsize=(width / dpi, height / dpi), dpi=dpi)
    fig.subplots_adjust(margin/width, margin/height,
                        (width-margin)/width, (height-margin)/height)
    ax.set_xlim(0, cell_width * 4)
    ax.set_ylim(cell_height * (nrows-0.5), -cell_height/2.)
    ax.yaxis.set_visible(False)
    ax.xaxis.set_visible(False)
    ax.set_axis_off()

    for i, name in enumerate(names):
        row = i % nrows
        col = i // nrows
        y = row * cell_height

        swatch_start_x = cell_width * col
        text_pos_x = cell_width * col + swatch_width + 7

        ax.text(text_pos_x, y, name, fontsize=14,
                horizontalalignment='left',
                verticalalignment='center')

        ax.add_patch(
            Rectangle(xy=(swatch_start_x, y-9), width=swatch_width,
                      height=18, facecolor=colors[name], edgecolor='0.7')
        )

    return fig

d = {"sell": [
           {
               "Rate": 0.001425,
               "Quantity": 537.27713514
           },
           {
               "Rate": 0.00142853,
               "Quantity": 6.59174681
           }
]}

path = "./data/dataBestFitnessGen_(1_3).json"
#path = "./data/dataBestFitnessGen.json"

with open(path) as f:
    data1 = json.load(f)

path2 = "./data/dataMeanFitnessGen_(1_3).json"
#path2 = "./data/dataMeanFitnessGen2.json"

with open(path2) as f:
    data2 = json.load(f)

path_weight = "./data/dataWeight_(1_3).json"
with open(path_weight) as f:
    data_weight = json.load(f)

plot_colortable(mcolors.CSS4_COLORS)
plt.show()
print(mcolors.CSS4_COLORS)
dict_data_l = {}
dict_data_a = {}
generations = list(data_weight.keys())
import random
colors = ["brown", "lightcoral", "turquoise", "blue", "plum", "violet"]

for i, sensor in enumerate(["SFL1", "SFL2", "SFC1", "SFC2", "SFR1", "SFR2"]):
    dict_data_l[sensor] = [data_weight[i][sensor][0] for i in list(data_weight.keys())]
    dict_data_a[sensor] = [data_weight[i][sensor][1] for i in list(data_weight.keys())]
    idx_color_random = random.randint(0, len(list(mcolors.CSS4_COLORS.keys())))
    color = list(mcolors.CSS4_COLORS.keys())[idx_color_random]
    plt.plot(generations, dict_data_a[sensor], color=colors[i], label=f'Sensor: {sensor}')


plt.xlabel("Xeracións")
plt.ylabel("Pesos")
plt.title(f"Gráfica pesos para velocidad angular en {int(generations[-1])} xeracións")

plt.grid()
plt.legend()
plt.show()

"""
df = pd.DataFrame(d['sell'])
print (df)
df.plot(x='Quantity', y='Rate')


generations = list(data1.keys())
best_list = list(data1.values())
mean_list = list(data2.values())

plt.plot(generations, best_list, color='r', label=f'best: {best_list[-1]}')
plt.plot(generations, mean_list, color='g', label=f'mean: {mean_list[-1]}')

# aming the x-axis, y-axis and the whole graph
plt.xlabel("Xeracións")
plt.ylabel("Fitness")
plt.title(f"Gráfica fitness x xeracións({int(generations[-1])})")

plt.grid()

# Adding legend, which helps us recognize the curve according to it's color
plt.legend()

# To load the display window
plt.show()

"""