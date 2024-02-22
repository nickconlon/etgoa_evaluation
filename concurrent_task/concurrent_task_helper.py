import matplotlib.pyplot as plt
from PIL import Image, ImageOps
import numpy as np
import yaml

metals = ['Iron', 'Gold', 'Lithium', 'Cobalt', 'Zinc']
colors = ['sienna', 'goldenrod', 'silver', 'steelblue', 'forestgreen']


def get_image(path):
    img = Image.open(path)
    return np.asarray(img)


class MarsMap:
    def __init__(self, imgpath):
        img = get_image(imgpath)
        self.base_img = img
        self.maxy, self.maxx, _ = img.shape

    def find_nearest(self, array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return idx

    def make_randomized_subset(self, save_file=None):
        fig, ax = plt.subplots(figsize=(25 * 0.65, 15 * 0.65))

        img = self.base_img.copy()
        _miny = np.random.randint(-90, 90)
        _height = np.random.randint(10, 90)
        _minx = np.random.randint(0, 360 - 2 * _height)
        _width = _height * 2
        print('H', _height)
        print('W', _width)
        print('Miny', _miny)
        print('Minx', _minx)
        if _miny >= 0:
            plt.imshow(img, extent=[_minx, _minx + _width, _miny, _miny + _height])
        else:
            plt.imshow(img, extent=[_minx, _minx + _width, _miny, _miny + _height])

        means = {m: [] for m in metals}
        for i in range(1):
            for c, m in zip(colors, metals):
                scale = np.random.uniform(1, _height*0.1)
                size = np.random.randint(20, 50)
                x = np.random.randint(_minx, _minx+_width)
                y = np.random.randint(_miny, _miny+_height)

                xs, ys = np.random.normal(loc=x, scale=scale, size=size), np.random.normal(loc=y, scale=scale,size=size)
                xs = xs[xs > _minx]
                xs = xs[xs < _minx+_width]
                ys = ys[ys > _miny]
                ys = ys[ys < _miny+_height]

                if len(ys) > len(xs):
                    ys = ys[:len(xs)]
                elif len(xs) > len(ys):
                    xs = xs[:len(ys)]

                plt.scatter(xs, ys, s=0.5, c=c, alpha=0.5)
                #plt.annotate(m, (x, y))
                means[m].append((x, y, scale))

        print('means', means)

        plt.ylabel('Latitude')
        plt.xlabel('Longitude')
        ax.set_xticks(np.arange(_minx, _minx+_width+1, 10))
        ax.set_xticks(np.arange(_minx, _minx+_width+1, 1), minor=True)
        ax.set_yticks(np.arange(_miny, _miny+_height+1, 10))
        ax.set_yticks(np.arange(_miny, _miny+_height+1, 1), minor=True)
        plt.grid(color='black', linestyle='-', linewidth=0.5, axis='both')
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
        #img.show()
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


def make_data_subset_map():
    p = '../concurrent_task/episodes/concurrent_{}.{}'
    num_examples = 1
    for i in range(num_examples):
        map_subset = np.random.randint(0, 10)
        mars_map = MarsMap('test_{}.png'.format(map_subset))
        if np.random.choice(a=[0, 1]):
            img = Image.open('test_{}.png'.format(map_subset))
            img = ImageOps.flip(img)
            mars_map.base_img = np.asarray(img)
            print('flipped')
        img, dist = mars_map.make_randomized_subset(p.format(i, 'png').format(i))
        for k, v in dist.items():
            v = v[0]
            v = [float(vv) for vv in v]
            dist[k] = v
        output = {'minerals': dist, 'target': str(np.random.choice(a=list(dist.keys())))}
        fname = p.format(i, 'yaml')
        with open(fname, 'w') as f:
            yaml.dump(output, f, default_flow_style=None, sort_keys=False)


if __name__ == '__main__':
    make_data_subset_map()
