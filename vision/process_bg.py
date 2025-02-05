import cv2

dir = 'Data/puppy/'

ind = 11

bg_file = dir + 'bg/%04d.png' % ind

image_file = dir + 'images/%04d.jpg' % ind

fg_file = dir + 'fg/%04d.jpg' % ind

bg_image = cv2.imread(bg_file, cv2.IMREAD_UNCHANGED)
image = cv2.imread(image_file)
fg_image = image

alpha_channel = bg_image[:, :, 3]

fg_image[alpha_channel==255] = 0

cv2.imwrite(fg_file, fg_image)