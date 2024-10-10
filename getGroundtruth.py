import numpy as np
import cv2
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser(description="Target image of landmarks")
parser.add_argument("--image_file", type=str, default="lm_test.jpg", help="image file")
args = parser.parse_args()

img = cv2.imread(args.image_file)
img = img[:,:,::-1]

plt.imshow(img)
plt.title("Pick the warping points")
points_src = plt.ginput(4)

print("Warping Points (px):")
print(points_src)

points_dst = [
    (0.0,0.0), (3.5,0.0), (3.5,2.0), (0.0,2.0)
]

print("Destination Points (m):")
print(points_dst)

plt.close()
translation_offset = np.reshape((500.0,500.0), (2,1))
points_dst = np.array(points_dst) * 1000.0 + translation_offset.T
points_src = np.array(points_src)
print("Calculating homography")
h, status = cv2.findHomography(points_src, points_dst)



im_warped = cv2.warpPerspective(img, h, (5000, 3500))

plt.imshow(im_warped)
plt.title("Warped Image")
plt.gca().invert_yaxis()


landmark_pts = plt.ginput(20, 1e9)

for lm in landmark_pts:
    print(lm[0]/1000-0.5, lm[1]/1000-0.5)


plt.show()