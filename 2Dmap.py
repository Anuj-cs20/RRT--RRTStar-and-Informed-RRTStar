from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt

# img = Image.open('cspace.png')
# img = ImageOps.grayscale(img)
# np_img = np.array(img)
# np_img = ~np_img

# invert B&W
# np_img[np_img > 1] = 1
# plt.set_cmap('binary')
# plt.imshow(np_img)

# Save Image
# np.save('cspace.npy', np_img)

# Read Image
grid = np.load('cspace.npy')
plt. imshow(grid, cmap="binary")
plt.tight_layout()
plt.show()
