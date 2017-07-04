import glob
import cv2
import numpy as np
import os
from util.draw import generate_sliding_windows
from util.draw import extract_heatmap
from util.classifier import svm_classifier
from util.classifier import transform_features
from sklearn.preprocessing import StandardScaler
from moviepy.editor import VideoFileClip

heatmap_buffer = np.zeros((10,720,1280))
heatmap_idx = 0

def video_pipeline(img):
    global heatmap_buffer
    global heatmap_idx
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    heatmap = np.zeros_like(img[:,:,0]).astype(np.float)
    windows = generate_sliding_windows(img.shape)

    for bboxes in windows:
        for bbox in bboxes:
            roi = cv2.resize(img[bbox[0][1]:bbox[1][1], bbox[0][0]:bbox[1][0]], (64, 64))
            features = transform_features(roi, cspace='YUV', orient=9, pix_per_cell=8, cell_per_block=2, hog_channel='ALL')
            scaled_features = X_scaler.transform(features)
            prediction = clf.predict(scaled_features)[0]
            if prediction > 0:
                # cv2.rectangle(imgcopy, bbox[0], bbox[1], color=(0, 255, 0))
                heatmap[bbox[0][1]:bbox[1][1], bbox[0][0]:bbox[1][0]] += 1

    heatmap_buffer[heatmap_idx] = heatmap
    heatmap_idx = (heatmap_idx + 1) % 10

    avgheatmap = 0.3 * heatmap + 0.7 * np.mean(heatmap_buffer, axis=0)

    avgheatmap[avgheatmap <= 3] = 0 # Filter heatmap
    # heatmap = heatmap * 250 / 50
    # heatmap = cv2.applyColorMap(heatmap, cv2.COLORMAP_HOT)
    extracted = extract_heatmap(img, avgheatmap)

    extracted = cv2.cvtColor(extracted, cv2.COLOR_BGR2RGB)

    return extracted

if __name__ == "__main__":
    images = glob.glob('./test_images/*.jpg')
    scaler = np.load('scaler.npz')
    X_scaler = StandardScaler()
    X_scaler.mean_, X_scaler.scale_ = scaler['mean'], scaler['scale']
    clf = svm_classifier()

    for image in images:
        # cnt = 0
        filename = os.path.splitext(os.path.basename(image))[0]
        test_image = cv2.imread(image)
        imgcopy = np.copy(test_image)
        heatmap = np.zeros_like(imgcopy[:,:,0]).astype(np.float)
        windows = generate_sliding_windows(test_image.shape)

        for bboxes in windows:
            for bbox in bboxes:
                roi = cv2.resize(test_image[bbox[0][1]:bbox[1][1], bbox[0][0]:bbox[1][0]], (64, 64))
                features = transform_features(roi, cspace='YUV', orient=9, pix_per_cell=8, cell_per_block=2, hog_channel='ALL')
                scaled_features = X_scaler.transform(features)
                prediction = clf.predict(scaled_features)[0]
                if prediction > 0:
                    cv2.rectangle(imgcopy, bbox[0], bbox[1], color=(0, 255, 0))
                    heatmap[bbox[0][1]:bbox[1][1], bbox[0][0]:bbox[1][0]] += 1
                    # cv2.imwrite('./output_images/' + filename + str(cnt) + '.jpg', roi)
                    # cnt += 1

        heatmap[heatmap <= 3] = 0 # Filter heatmap
        heatmap = heatmap * 250 / 50
        heatmap = cv2.applyColorMap(heatmap, cv2.COLORMAP_HOT)
        extracted = extract_heatmap(test_image, heatmap)
        cv2.imwrite('./output_images/' + filename + '_heat.jpg', heatmap)
        cv2.imwrite('./output_images/' + filename + '_extracted.jpg', extracted)
        cv2.imwrite('./output_images/' + filename + '.jpg', imgcopy)

    # Video
    clip = VideoFileClip('./project_video.mp4')
    output_clip = clip.fl_image(video_pipeline)
    output_clip.write_videofile('./output_images/project_video99.mp4', audio=False)
