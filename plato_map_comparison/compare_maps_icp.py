import numpy as np
import cv2
import icp
import glob

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

def compare_maps(map_generated, map_real):
    rows, cols = map_real.shape
    rows_g, cols_g = map_generated.shape

    if rows_g < rows and cols_g < cols:
        map_gen = np.ones((rows, cols), np.uint8)
        map_gen *= 255
        x_offset, y_offset = int(rows/2 - rows_g/2), int(cols/2 - cols_g/2)
        map_gen[x_offset:x_offset+map_generated.shape[0],
                y_offset:y_offset+map_generated.shape[1]] = map_generated
    else:
        map_gen = map_generated
        map_rel = np.ones((rows_g, cols_g), np.uint8)
        map_rel *= 255
        x_offset, y_offset = int(rows_g/2 - rows/2), int(cols_g/2 - cols/2)
        map_rel[x_offset:x_offset+map_real.shape[0],
                y_offset:y_offset+map_real.shape[1]] = map_real
        map_real = map_rel
    map_real[map_real > 100] = 255

    size = np.size(map_real)
    map_real_thin = np.zeros(map_real.shape, np.uint8)
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

    # Thinning
    while(not done):
        eroded = cv2.erode(map_real, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(map_real, temp)
        map_real_thin = cv2.bitwise_or(map_real_thin, temp)
        map_real = eroded.copy()

        zeros = size - cv2.countNonZero(map_real)
        if zeros == size:
            done = True


    points_gen = cv2.findNonZero(map_gen)
    points_real = cv2.findNonZero(map_real_thin)

    if points_gen.shape[0] > points_real.shape[0]:
        num_of_el = points_real.shape[0]
        points_gen = points_gen[np.random.choice(points_gen.shape[0], num_of_el, replace=False)]
    else:
        num_of_el = points_gen.shape[0]
        points_real = points_real[np.random.choice(points_real.shape[0], num_of_el, replace=False)]

    T, distances, iterations = icp.icp(points_real[:,0,:], points_gen[:,0,:], max_iterations=1000, tolerance=0.0000001)

    if cols_g > cols and rows_g > rows:
        map_transformed = cv2.warpAffine(map_real_thin, T[0:2,0:3], (cols_g, rows_g))
    else:
        map_transformed = cv2.warpAffine(map_real_thin, T[0:2,0:3], (cols, rows))

    return map_transformed, map_gen, np.sum(distances)/num_of_el

MAP_NAME_GENERATED = 'generated_318.pgm'
MAP_NAMES_REAL = glob.glob('maps/room_318/*.pgm')

map_generated = cv2.imread('../plato_navigation/maps/{}'.format(MAP_NAME_GENERATED), -1)

with open('comparison_result/room_318/results.txt', 'a') as the_file:

    for i in range(len(MAP_NAMES_REAL)):
        # cv2.namedWindow("maps comparison {}".format(i), cv2.WINDOW_NORMAL)
        map_real = cv2.imread('{}'.format(MAP_NAMES_REAL[i]), -1)
        map_rel, map_gen, comparison_error = compare_maps(map_generated, map_real)
        map_comparison = np.ones((map_rel.shape[0], map_rel.shape[1], 3), np.uint8)
        map_comparison *= 255
        kernel = np.ones((5,5),np.uint8)
        map_gen = cv2.dilate(map_gen,kernel,iterations = 1)
        map_comparison[map_gen>0] = (0,0,255)
        map_comparison[map_rel>0] = (255,0,0)
        # map_result = cv2.bitwise_or(map_rel, map_gen)
        # cv2.imshow("maps comparison {}".format(i), map_comparison)
        cv2.imwrite('comparison_result/room_318/{}.png'.format(MAP_NAMES_REAL[i].split('/')[1].split('.')[0]), map_comparison)
        the_file.write('{}:     {}\n'.format(MAP_NAMES_REAL[i].split('/')[1].split('.')[0], comparison_error))
        cv2.waitKey(1)
        print("maps comparison {} summary error {}".format(i, comparison_error))

cv2.waitKey(0)
cv2.destroyAllWindows()
