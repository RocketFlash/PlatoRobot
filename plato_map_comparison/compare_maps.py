import numpy as np
import cv2
import icp


def find_nearest_white(img_1, img_2):
    nonzero_1 = cv2.findNonZero(img_1)
    nonzero_2 = cv2.findNonZero(img_2)
    all_min_distances = np.zeros(nonzero_2.shape[0])
    all_min_coordinates = np.zeros_like(nonzero_2)
    for i in range(nonzero_2.shape[0]):
        distances = np.absolute(nonzero_1[:, :, 0] - nonzero_2[i][0][0]) + \
            np.absolute(nonzero_1[:, :, 1] - nonzero_2[i][0][1])
        nearest_index = np.argmin(distances)
        min_dist = distances[nearest_index]
        all_min_distances[i] = min_dist
        all_min_coordinates[i] = nonzero_1[nearest_index]
    return all_min_distances, all_min_coordinates


def find_image_center(img_1):
    nonzero_1 = cv2.findNonZero(img_1)
    cx = int(np.mean(nonzero_1[:, :, 0]))
    cy = int(np.mean(nonzero_1[:, :, 1]))
    return cx, cy


MAP_NAME_GENERATED = 'test_generated_map.pgm'
MAP_NAME_REAL = 'test_gmapping.pgm'

map_real = cv2.imread('maps/{}'.format(MAP_NAME_REAL), -1)
map_generated = cv2.imread('maps/{}'.format(MAP_NAME_GENERATED), -1)

rows, cols = map_real.shape
rows_g, cols_g = map_generated.shape
map_gen = np.ones((rows, cols), np.uint8)
map_gen *= 255
x_offset, y_offset = int(rows/2 - rows_g/2), int(cols/2 - cols_g/2)
map_gen[x_offset:x_offset+map_generated.shape[0],
        y_offset:y_offset+map_generated.shape[1]] = map_generated
map_real[map_real == 205] = 255
cv2.namedWindow('image_before', cv2.WINDOW_NORMAL)
cv2.namedWindow('image_after', cv2.WINDOW_NORMAL)
size = np.size(map_real)
skel = np.zeros(map_real.shape, np.uint8)
ret, map_real = cv2.threshold(map_real, 127, 255, cv2.THRESH_BINARY_INV)
ret, map_gen = cv2.threshold(map_gen, 127, 255, cv2.THRESH_BINARY_INV)

im2, contours, hierarchy = cv2.findContours(map_gen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

cnt = contours[0]
M = cv2.moments(cnt)

cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])


map_real = cv2.morphologyEx(map_real, cv2.MORPH_OPEN, (3, 3))
map_real = cv2.morphologyEx(map_real, cv2.MORPH_CLOSE, (3, 3))
element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
done = False

cx_real, cy_real = find_image_center(map_real)

cv2.imshow("image_before", map_real)
cv2.waitKey(1)

# Thinning
while(not done):
    eroded = cv2.erode(map_real, element)
    temp = cv2.dilate(eroded, element)
    temp = cv2.subtract(map_real, temp)
    skel = cv2.bitwise_or(skel, temp)
    map_real = eroded.copy()

    zeros = size - cv2.countNonZero(map_real)
    if zeros == size:
        done = True


ranges_XY = [1000, 500, 200, 100, 10, 5]
step_XY = [100, 50, 20, 10, 2, 1]
current_min_x = int(cx_real - rows/2)
current_min_y = int(cy_real - cols/2)

for idx, range_i in enumerate(ranges_XY):
    x_min, x_max, x_step = current_min_x-range_i, current_min_x+range_i, step_XY[idx]
    y_min, y_max, y_step = current_min_y-range_i, current_min_y+range_i, step_XY[idx]
    angle_min, angle_max, angle_step = -90, -70, 1
    min_error = 100000000

    # x_min, x_max, x_step = 622, 632, 1
    # y_min, y_max, y_step = -223, -213, 1
    # angle_min, angle_max, angle_step = -85, -80, 1
    print('RANGE {}:'.format(range_i))
    for x_shift in range(x_min, x_max, x_step):
        for y_shift in range(y_min, y_max, y_step):
            for angle in range(angle_min, angle_max, angle_step):
                M = cv2.getRotationMatrix2D((cx, cy), angle, 1)
                des = cv2.warpAffine(map_gen, M, (cols, rows))
                M = np.float32([[1, 0, x_shift], [0, 1, y_shift]])
                des = cv2.warpAffine(des, M, (cols, rows))
                des_or = cv2.bitwise_or(des, skel)
                min_distances, min_coordinates = find_nearest_white(des, skel)
                error = np.sum(min_distances)
                if error < min_error:
                    min_error = error
                    min_x_shift = x_shift
                    min_y_shift = y_shift
                    min_angle_shift = angle
                    best_match = des_or
                    print("NEW BEST SCORE: {}".format(min_error))
                # cv2.imshow("image_after", best_match)
                # cv2.waitKey(1)
    current_min_x = min_x_shift
    current_min_y = min_y_shift


cv2.imshow("image_after", best_match)
cv2.waitKey(0)
print("BEST RESULT: {}".format(min_error))
print("best x_shift: {}, best y_shift: {}, best angle_shift: {}".format(
    min_x_shift, min_y_shift, min_angle_shift))
cv2.destroyAllWindows()
