import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import Image
import numpy as np
import yaml

# , 'zinc', 'niobium', 'molybdenum', 'lanthanum', 'europium', 'tungsten', 'gold']
metals = ['Iron', 'Gold', 'Lithium', 'Cobalt', 'Zinc']
colors = [[183, 65, 14], [212, 175, 55], [192, 192, 192], [0, 71, 171], [52, 85, 70]]


def get_image(path):
    img = Image.open(path)
    return np.asarray(img)


class MarsMap:
    def __init__(self, imgpath):
        img = get_image(imgpath)
        self.base_img = img
        self.maxy, self.maxx, _ = img.shape
        self.mapped_x, self.old_x_ticks, self.new_x_ticks = self.remap_x()
        self.mapped_y, self.old_y_ticks, self.new_y_ticks = self.remap_y()

    def find_nearest(self, array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return idx

    def remap_y(self):
        yf = np.arange(0, self.maxy)
        yi = np.arange(0, self.maxy / 2)

        upper = np.linspace(90, 0, len(yi))
        lower = np.linspace(-1, -90, len(yi))
        newy = np.concatenate((upper, lower))

        old_ticks = []
        new_ticks = []

        for i in np.arange(0, 100, 10):
            idx = self.find_nearest(newy, i)
            old_ticks.append(yf[idx])
            new_ticks.append(int(np.around(newy[idx], -1)))

        for i in np.linspace(10, 90, 9):
            idx = self.find_nearest(newy, -i)
            old_ticks.append(yf[idx])
            new_ticks.append(int(np.around(newy[idx], -1)))

        mapped_y = {int(lat): x for lat, x in zip(newy, yf)}
        return mapped_y, old_ticks, new_ticks

    def remap_x(self):
        yf = np.arange(0, self.maxx)
        yi = np.arange(0, self.maxx / 2)

        upper = np.linspace(180, 360, len(yi))
        lower = np.linspace(0, 180, len(yi))
        newy = np.concatenate((upper, lower))

        old_ticks = []
        new_ticks = []

        for i in np.arange(0, 180, 10):
            idx = self.find_nearest(newy, i)
            old_ticks.append(yf[idx])
            new_ticks.append(int(np.around(newy[idx], -1)))

        for i in np.linspace(190, 350, 17):
            idx = self.find_nearest(newy, i)
            old_ticks.append(yf[idx])
            new_ticks.append(int(np.around(newy[idx], -1)))

        mapped_x = {int(lat): x for lat, x in zip(newy, yf)}
        return mapped_x, old_ticks, new_ticks

    def make_task_instance(self, include_minerals, save_file=None):
        fig, ax = plt.subplots(figsize=(25 * 0.65, 15 * 0.65))

        img = self.base_img.copy()
        means = {}

        if include_minerals:
            means = {m: [] for m in metals}
            for i in range(1):
                for c, m in zip(colors, metals):
                    scale = np.random.randint(5, 20)
                    size = np.random.randint(100, 500)
                    x, y = np.random.randint(scale, self.maxx - scale), np.random.randint(scale, self.maxy - scale)
                    xs, ys = np.random.normal(loc=x, scale=scale, size=size), np.random.normal(loc=y, scale=scale,
                                                                                               size=size)
                    for y, x in zip(ys, xs):
                        if 0 < x < self.maxx and 0 < y < self.maxy:
                            y, x = int(y), int(x)
                            img[y, x] = c

                    means[m].append((y, x, scale))

            patches = [mpatches.Patch(color=np.asarray(colors[i]) / 255., label=metals[i]) for i in range(len(colors))]
            # put those patched as legend-handles into the legend
            # plt.legend(handles=patches, bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0., prop={'size': 20})

        plt.grid(color='black', linestyle='-', linewidth=1)
        plt.imshow(img)
        plt.ylabel('Latitude')
        plt.xlabel('Longitude')
        plt.yticks(self.old_y_ticks, self.new_y_ticks)
        plt.xticks(self.old_x_ticks, self.new_x_ticks)
        ax.tick_params(labeltop=True, labelright=True)
        plt.tight_layout()

        canvas = plt.gca().figure.canvas
        canvas.draw()
        data = np.frombuffer(canvas.tostring_rgb(), dtype='uint8')
        img = data.reshape(canvas.get_width_height()[::-1] + (3,))
        height, width, channel = img.shape
        img = Image.fromarray(img)
        img = img.crop((10, 60, width-10, height-60))
        img.save(save_file)

        plt.close(fig)

        return img, means


def read_next(data_path, img_path):
    with open(data_path) as f:
        data = yaml.safe_load(f)
        img = Image.open(img_path)
        # 1621, 831
        img = img.resize((1621, 831))
        img = np.array(img)[:, :, :3]
        if 'minerals' in data:
            dist = data['minerals']
            target = data['target']
        else:
            dist = {}
            target = {}
    return img, dist, target


def make_data():
    mars_map = MarsMap("../imgs/mars_map_cropped.png")
    p = '../concurrent_task/training/concurrent_{}.{}'
    num_examples = 5
    for i in range(num_examples):
        output = {}
        if i == 0:
            img, dist = mars_map.make_task_instance(False, p.format(i, 'png').format(i))
        else:
            img, dist = mars_map.make_task_instance(True, p.format(i, 'png').format(i))
            for k, v in dist.items():
                v = v[0]
                v = [float(vv) for vv in v]
                dist[k] = v
            output = {'minerals': dist, 'target': str(np.random.choice(a=list(dist.keys())))}
        fname = p.format(i, 'yaml')
        with open(fname, 'w') as f:
            yaml.dump(output, f, default_flow_style=None, sort_keys=False)


if __name__ == '__main__':
    make_data()
