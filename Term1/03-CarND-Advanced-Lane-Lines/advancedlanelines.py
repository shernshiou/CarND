import numpy as np
import cv2
import glob
import os
import pickle
from matplotlib import pyplot as plt
from moviepy.editor import VideoFileClip

def calibrate_camera(camera_folder='camera_cal', output_folder='output_images', test_folder='test_images'):
    '''
    Camera calibration
    '''
    if (os.path.isfile('calibration.p')):
        calibration = pickle.load(open('calibration.p', 'rb'))
        ret, mtx, dist, rvecs, tvecs = calibration
    else:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objpoints = []
        imgpoints = []
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

        camera_images = glob.glob(camera_folder + '/*.jpg')
        test_images = glob.glob(test_folder + '/*.jpg')

        for imagefile in camera_images:
            img = cv2.imread(imagefile)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                img_board = cv2.drawChessboardCorners(img, (9, 6), corners2, ret)

                filename = os.path.basename(imagefile)
                cv2.imwrite('./' + output_folder + '/camera/calib/' + filename, img_board)

        calibration = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        ret, mtx, dist, rvecs, tvecs = calibration
        pickle.dump(calibration, open('calibration.p', 'wb'))

        for imagefile in camera_images:
            filename = os.path.basename(imagefile)
            img = cv2.imread(imagefile)
            h,  w = img.shape[:2]
            newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
            dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
            x,y,w,h = roi
            dst = dst[y:y+h, x:x+w]
            cv2.imwrite('./' + output_folder + '/camera/undist/' + filename, dst)

        for testfile in test_images:
            filename = os.path.basename(testfile)
            img = cv2.imread(testfile)
            h,  w = img.shape[:2]
            dst = cv2.undistort(img, mtx, dist)

            # Draw perspective polygon
            pts = np.array([[516, 460], [756, 460], [1200, 720], [100, 720]], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.polylines(dst,[pts],True,(0,0,255))

            cv2.imwrite('./' + output_folder + '/test/' + filename, dst)

    return ret, mtx, dist, rvecs, tvecs

def transform(img):
    pts1 = np.float32([[516, 460], [756, 460], [1200, 720], [100, 720]])
    pts2 = np.float32([[0, 0], [1280, 0], [1050, 720], [250, 720]])

    M = cv2.getPerspectiveTransform(pts1,pts2)
    transformed = cv2.warpPerspective(img,M,(img.shape[1], img.shape[0]))

    return transformed

def untransform(img):
    pts1 = np.float32([[516, 460], [756, 460], [1200, 720], [100, 720]])
    pts2 = np.float32([[0, 0], [1280, 0], [1050, 720], [250, 720]])

    M = cv2.getPerspectiveTransform(pts2,pts1)
    mask = cv2.warpPerspective(img,M,(img.shape[1], img.shape[0]))

    return mask

def sobel_threshold(img, orient='x', sobel_kernel=3, thresh=(0,255)):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    if orient == 'y':
        sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize = sobel_kernel)
    else:
        sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize = sobel_kernel)

    sobel_abs = np.absolute(sobel)
    sobel_scaled = np.uint8(255*sobel_abs/np.max(sobel_abs))
    mask = np.zeros_like(sobel_scaled)
    mask[(sobel_scaled > thresh[0]) & (sobel_scaled < thresh[1]) ] = 1
    return mask

def mag_threshold(img, sobel_kernel=3, mag_thresh=(0, 255)):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, sobel_kernel)
    sobelxy = np.sqrt(sobelx**2 + sobely**2)
    scale = np.max(sobelxy)/255
    sobel_scaled = (sobelxy/scale).astype(np.uint8)
    mask = np.zeros_like(sobel_scaled)
    mask[(sobel_scaled > mag_thresh[0]) & (sobel_scaled < mag_thresh[1]) ] = 1
    return mask

def dir_threshold(img, sobel_kernel=3, thresh=(0, np.pi/2)):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    dir1 = np.arctan2(abs_sobely, abs_sobelx)
    mask = np.uint8(np.zeros_like(dir1))
    mask[(dir1 >= thresh[0]) & (dir1 <= thresh[1])] = 1
    return mask

def threshold_type1(img):
    ksize = 25
    thresh_mag = (15, 130)

    # Magnitude
    mag_bin = mag_threshold(img, sobel_kernel=ksize, mag_thresh=thresh_mag)

    # Extract Yellow
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    lower_yellow = np.array([18,10,10])
    upper_yellow = np.array([80,240,240])
    yellow_mask = cv2.inRange(hls, lower_yellow, upper_yellow)

    # Extract White
    white_mask = np.zeros_like(hls[:, :, 0])
    white_mask[(hls[:,:,2] > 100) & (hls[:,:,0] < 90)] = 255

    combined = np.zeros_like(mag_bin)
    combined[(mag_bin == 1) | (yellow_mask == 255) | (white_mask == 255)] = 255

    # Dilate + Erode
    kernel = np.ones((5,5),np.uint8)
    return cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)

def threshold_type2(img):
    ksize = 31
    thresh_sobel = (50, 150)
    thresh_mag = (50, 255)
    thresh_dir = (0.75, 1.15)

    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)

    gradx = sobel_threshold(img, orient='x', sobel_kernel=ksize, thresh=thresh_sobel)
    mag_bin = mag_threshold(img, sobel_kernel=ksize, mag_thresh=thresh_mag)
    dir_bin = dir_threshold(img, sobel_kernel=ksize, thresh=thresh_dir)

    # Combine Thresholds 1
    comb_bin = np.zeros_like(gradx)
    comb_bin[(gradx == 1) | ((dir_bin == 1) & (mag_bin == 1))] = 1

    # Color Threshold S-channel
    thresh_s = (170, 255)
    img_color = hls[:,:,2]
    color_bin = np.zeros_like(img_color)
    color_bin[(img_color > thresh_s[0]) & (img_color <= thresh_s[1])] = 1

    # Color Threshold R-channel
    thresh_r = (200, 255)
    r_img = img[:,:,0]

    r_bin = np.zeros_like(r_img)
    r_bin[(r_img > thresh_r[0]) & (r_img <= thresh_r[1])] = 1

    # Combined Gradient/Mag + Color S + Color R
    combined = np.zeros_like(comb_bin)
    combined[(comb_bin == 1) | (color_bin == 1) | (r_bin == 1)] = 255

    return combined

def centroids(img, chunks=10):
    h, w = img.shape[:2]
    y = 0
    left_x, left_y = np.empty((0)), np.empty((0))
    right_x, right_y = np.empty((0)), np.empty((0))
    for i in range(chunks):
        y = i * h/chunks
        # print(i+1, y, y+(h/chunks))
        chunk = img[int(y): int(y+(h/chunks)), 0:int(w)]
        hist = np.sum(chunk/255, axis=0)
        split = np.array_split(hist, 2)

        # Left
        left_split = split[0]
        reversed_left_split = split[0][::-1]
        first_left = np.argmax(left_split)
        last_left = len(reversed_left_split) - np.argmax(reversed_left_split) - 1
        if np.amax(left_split) > 1:
            mid_left = first_left + int((last_left - first_left) / 2)
            # Discard noise in relation with previous point
            if left_x.size != 0:
                if abs(left_x[-1] - mid_left) < 110:
                    left_x = np.append(left_x, mid_left)
                    left_y = np.append(left_y, int(y + chunks/2))
            else:
                left_x = np.append(left_x, mid_left)
                left_y = np.append(left_y, int(y + chunks/2))

        # Right
        right_split = split[1]
        reversed_right_split = split[1][::-1]
        first_right = np.argmax(right_split)
        last_right = len(reversed_right_split) - np.argmax(reversed_right_split) - 1
        if np.amax(right_split) > 1:
            mid_right = int(first_right + ((last_right - first_right) / 2) + w/2)
            # Discard noise in relation with previous point
            if right_x.size != 0:
                # print(right_x[-1], mid_right, abs(right_x[-1] - mid_right))
                if abs(right_x[-1] - mid_right) < 110:
                    right_x = np.append(right_x, mid_right)
                    right_y = np.append(right_y, int(y + chunks/2))
            else:
                right_x = np.append(right_x, mid_right)
                right_y = np.append(right_y, int(y + chunks/2))
    return {
        'left': (left_x, left_y),
        'right': (right_x, right_y)
    }

def calculate_radius(width, height, ploty, left_fit, right_fit, leftx, rightx):
    y_eval = np.max(ploty)
    left_curverad = ((1 + (2 * left_fit[0] * y_eval + left_fit[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit[0])
    right_curverad = ((1 + (2 * right_fit[0] * y_eval + right_fit[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit[0])

    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30 / 720  # meters per pixel in y dimension
    xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty * ym_per_pix, leftx * xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty * ym_per_pix, rightx * xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad_real = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad_real = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters

    left = left_fit[0] * height ** 2 + left_fit[1] * height + left_fit[2]
    right = right_fit[0] * height ** 2 + right_fit[1] * height + right_fit[2]
    center_point = (left + right) / 2
    center_distance = (width / 2 - center_point) * xm_per_pix

    return left_curverad_real, right_curverad_real, center_distance

def best_fit(t1, t2, img):
    h, w = img.shape[:2]
    leftfit_t1 = np.polyfit(t1['left'][1], t1['left'][0], 2, full=True)
    rightfit_t1 = np.polyfit(t1['right'][1], t1['right'][0], 2, full=True)
    # print('Type 1')
    # print(t1['left'][0].size, t1['right'][0].size)
    # print(leftfit_t1[1], rightfit_t1[1])

    leftfit_t2 = np.polyfit(t2['left'][1], t2['left'][0], 2, full=True)
    rightfit_t2 = np.polyfit(t2['right'][1], t2['right'][0], 2, full=True)
    # print('Type 2')
    # print(t2['left'][0].size, t2['right'][0].size)
    # print(leftfit_t2[1], rightfit_t2[1])

    if (t1['left'][0].size > 5) and (t2['left'][0].size > 5):
        if abs(leftfit_t1[1] - leftfit_t2[1]) < 500:
            leftfit = leftfit_t1[0]
        elif leftfit_t1[1] < leftfit_t2[1]:
            leftfit = leftfit_t1[0]
        else:
            leftfit = leftfit_t2[0]
    elif t1['left'][0].size > 5:
        leftfit = leftfit_t1[0]
    elif t2['left'][0].size > 5:
        leftfit = leftfit_t2[0]
    else:
        leftfit = leftfit_t1[0]

    # if (t1['right'][0].size > 5) and (t2['right'][0].size > 5):
    #     if rightfit_t1[1] > rightfit_t2[1]:
    #         rightfit = rightfit_t2[0]
    #     else:
    #         rightfit = rightfit_t1[0]
    # elif t1['right'][0].size > 5:
    #     rightfit = rightfit_t1[0]
    # elif t2['right'][0].size > 5:
    #     rightfit = rightfit_t2[0]
    # else:
    #     rightfit = rightfit_t2[0]

    # if abs(leftfit_t1[1] - leftfit_t2[1]) < 500:
    #     leftfit = leftfit_t1[0]
    # elif leftfit_t1[1] < leftfit_t2[1]:
    #     leftfit = leftfit_t1[0]
    # else:
    #     leftfit = leftfit_t2[0]

    if rightfit_t1[1] > rightfit_t2[1]:
        rightfit = rightfit_t2[0]
    else:
        rightfit = rightfit_t1[0]

    # Calculate additional details
    ploty = np.linspace(0, h - 1, num=h)
    leftfit_x = leftfit[0] * ploty ** 2 + leftfit[1] * ploty + leftfit[2]
    rightfit_x = rightfit[0] * ploty ** 2 + rightfit[1] * ploty + rightfit[2]
    left_curverad, right_curverad, center_distance = calculate_radius(w, h, ploty, leftfit, rightfit, leftfit_x, rightfit_x)

    return {
        'fit': (np.poly1d(leftfit), np.poly1d(rightfit)),
        'fitx': (leftfit_x, rightfit_x),
        'detail': (left_curverad, right_curverad, center_distance)
    }

def curve_viz(img, fit):
    h, w = img.shape[:2]
    points = []
    overlay = np.zeros_like(img)
    alpha = 0.3

    # print(fit)

    for i in range(h):
        x = int(fit[0](i))
        y = int(i)
        points.append([x,y])
        x = int(fit[1](i))
        y = int(i)
        points.append([x,y])

    points = np.array(points).reshape((-1,1,2))
    cv2.polylines(overlay,[points],True,(0,255,0))
    cv2.addWeighted(overlay, alpha, img, 1 - alpha,0, img)

    return (img, overlay)

def overlay(img, mask, detail):
    h, w = img.shape[:2]
    alpha = 0.3
    mask = untransform(mask)
    cv2.addWeighted(mask, alpha, img, 1 - alpha,0, img)

    left_curverad, right_curverad, center_distance = detail
    cv2.putText(img, "Center Distance:  {:6.2f} m".format(center_distance),
        (20, 20 + 1 * 40 + 60), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2)
    cv2.putText(img, "Left Line Curve:  {:6.2f} m".format(left_curverad),
        (20, 20 + 2 * 40 + 60), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2)
    cv2.putText(img, "Right Line Curve: {:6.2f} m".format(right_curverad),
        (20, 20 + 3 * 40 + 60), cv2.FONT_HERSHEY_SIMPLEX, 1, [255, 255, 255], 2)

    return img

def video_pipeline(img):
    calibration = pickle.load(open('calibration.p', 'rb'))
    ret, mtx, dist, rvecs, tvecs = calibration

    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    calibrated = cv2.undistort(img, mtx, dist)
    transformed = transform(calibrated)
    type1 = threshold_type1(transformed)
    type2 = threshold_type2(transformed)

    centroids_t1 = centroids(type1)
    centroids_t2 = centroids(type2)
    ret = best_fit(centroids_t1, centroids_t2, calibrated)

    transformed, mask = curve_viz(transformed, ret['fit'])
    output = overlay(calibrated, mask, ret['detail'])

    return cv2.cvtColor(output, cv2.COLOR_BGR2RGB)

def main():
    # Prepare directory
    os.makedirs('./output_images/camera/calib', exist_ok=True)
    os.makedirs('./output_images/camera/undist', exist_ok=True)
    os.makedirs('./output_images/test/', exist_ok=True)

    # Camera Calibration
    ret, mtx, dist, rvecs, tvecs = calibrate_camera()

    # Static Image Frames
    test_images = glob.glob('./test_images/*.jpg')
    for testfile in test_images:
        test_img = cv2.imread(testfile)
        calibrated = cv2.undistort(test_img, mtx, dist)
        transformed = transform(calibrated)
        type1 = threshold_type1(transformed)
        type2 = threshold_type2(transformed)

        centroids_t1 = centroids(type1)
        centroids_t2 = centroids(type2)
        ret = best_fit(centroids_t1, centroids_t2, calibrated)

        transformed, mask = curve_viz(transformed, ret['fit'])
        output = overlay(calibrated, mask, ret['detail'])

        filename = os.path.splitext(os.path.basename(testfile))[0]
        print(filename)
        cv2.imwrite('./output_images/test/' + filename + '_transformed.jpg', transformed)
        cv2.imwrite('./output_images/test/' + filename + '_t1.jpg', type1)
        cv2.imwrite('./output_images/test/' + filename + '_t2.jpg', type2)
        cv2.imwrite('./output_images/test/' + filename + '_mask.jpg', mask)
        cv2.imwrite('./output_images/test/' + filename + '_output.jpg', output)

    # Process video
    clip = VideoFileClip('./project_video.mp4')
    output_clip = clip.fl_image(video_pipeline)
    output_clip.write_videofile('./output_images/project_video.mp4', audio=False)

if __name__ == '__main__':
    main()
