from PIL import Image
import numpy as np

# Opens a image in RGB mode
im = Image.open('../imgs/mars_map_cropped.png')

# Size of the image in pixels (size of original image)
# (This is not mandatory)
width, height = im.size

for i in [7]: #np.arange(6, 10):
    # Setting the points for cropped image
    tgt_width = int(1600/4)
    tgt_height = int(800/4)
    left = np.random.randint(0, 1600-tgt_width)
    top = np.random.randint(0, 800-tgt_height)
    right = left + tgt_width
    bottom = top + tgt_height

    # Cropped image of above dimension
    # (It will not change original image)
    im1 = im.crop((left, top, right, bottom))
    im1 = im1.resize((1600, 800))
    # Shows the image in image viewer
    im1.show()
    im1.save('test_{}.png'.format(i))