import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import Image
import numpy as np


def make_task_instance():
    metals = ['Iron', 'Gold', 'Lithium', 'Cobalt',
              'Zinc']  # , 'zinc', 'niobium', 'molybdenum', 'lanthanum', 'europium', 'tungsten', 'gold']
    colors = [[183, 65, 14, 255], [212, 175, 55, 255], [192, 192, 192, 255], [0, 71, 171, 255], [52, 85, 70, 255]]
    means = {m: [] for m in metals}

    img = 'mars_map.png'
    img = Image.open(img)
    img = np.asarray(img)
    img = img[20:, :, :]

    maxy, maxx, _ = img.shape

    for i in range(10):
        for c, m in zip(colors, metals):
            scale = np.random.randint(5, 20)
            size = np.random.randint(25, 500)
            x, y = np.random.randint(scale, maxx - scale), np.random.randint(scale, maxy - scale)
            xs, ys = np.random.normal(loc=x, scale=scale, size=size), np.random.normal(loc=y, scale=scale, size=size)

            for y, x in zip(ys, xs):
                if 0 < x < maxx and 0 < y < maxy:
                    y, x = int(y), int(x)
                    img[y, x] = c

            means[m].append((y, x))

    fig, ax = plt.subplots(figsize=(25, 15))
    patches = [mpatches.Patch(color=np.asarray(colors[i]) / 255., label=metals[i]) for i in range(len(colors))]
    # put those patched as legend-handles into the legend
    plt.legend(handles=patches, bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0., prop={'size': 20})

    plt.axis('off')
    plt.imshow(img)
    plt.tight_layout()
    plt.savefig('concurrent_task.png')
    plt.show()
    return img, means


if __name__ == '__main__':
    save_path = ""
    image, mineral_means = make_task_instance()
