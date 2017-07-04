import glob
import cv2
import numpy as np
import time
import os.path
from sklearn import svm
from sklearn.preprocessing import StandardScaler
from skimage.feature import hog
from sklearn.model_selection import train_test_split
from sklearn.model_selection import StratifiedShuffleSplit
from sklearn.model_selection import GridSearchCV
from sklearn.externals import joblib

def svm_classifier(accuracy=False):
    if os.path.isfile('svm.pkl'):
        clf = joblib.load('svm.pkl')
    else:
        vehicles = []
        non_vehicles = []
        non_v_files = glob.glob('../data/non-vehicles/*/*.png')
        v_files = glob.glob('../data/vehicles/*/*.png')

        print("Num vehicles: ", len(v_files))
        print("Num non-vehicles: ", len(non_v_files))
        t0=time.time()

        # X
        non_v_features = extract_features(non_v_files, cspace='YUV', orient=9, pix_per_cell=8, cell_per_block=2, hog_channel='ALL')
        v_features = extract_features(v_files, cspace='YUV', orient=9, pix_per_cell=8, cell_per_block=2, hog_channel='ALL')
        print(round(time.time()-t0, 2), 'Seconds to extract features...')
        X = np.vstack((v_features, non_v_features)).astype(np.float64)
        X_scaler = StandardScaler().fit(X)
        scaled_X = X_scaler.transform(X)
        np.savez("scaler.npz", mean=X_scaler.mean_, scale=X_scaler.scale_)

        # y
        y = np.hstack((np.ones(len(v_features)), np.zeros(len(non_v_features))))

        rand_state = np.random.randint(0, 100)
        X_train, X_test, y_train, y_test = train_test_split(scaled_X, y, test_size=0.2, random_state=rand_state)
        print(X_train.shape, y_train.shape)

        # SVM
        # C_range = np.logspace(-2, 10, 13)
        # gamma_range = np.logspace(-9, 3, 13)
        # param_grid = dict(gamma=gamma_range, C=C_range)
        # cv = StratifiedShuffleSplit(n_splits=5, test_size=0.2, random_state=42)
        # grid = GridSearchCV(svm.SVC(), param_grid=param_grid, cv=cv)
        # grid.fit(X_train[0:500], y_train[0:500])
        # print("The best parameters are %s with a score of %0.2f" % (grid.best_params_, grid.best_score_))

        print('Start training SVM')
        t0=time.time()
        clf = svm.LinearSVC(random_state=rand_state)
        # clf = svm.SVC() # RBF
        clf.fit(X_train, y_train)
        print(round(time.time()-t0, 2), 'Seconds to train SVC...')

        if accuracy:
            print('Test Accuracy of SVC = ', round(clf.score(X_test, y_test), 4))

        joblib.dump(clf, 'svm.pkl')
    return clf

def get_hog_features(img, orient, pix_per_cell, cell_per_block, vis=False, feature_vec=True):
    if vis == True:
        features, hog_image = hog(img, orientations=orient, pixels_per_cell=(pix_per_cell, pix_per_cell),
                                  cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=False,
                                  visualise=True, feature_vector=False)
        return features, hog_image
    else:
        features = hog(img, orientations=orient, pixels_per_cell=(pix_per_cell, pix_per_cell),
                       cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=False,
                       visualise=False, feature_vector=feature_vec)
        return features

def bin_spatial(img, size=(32, 32)):
    feature_image = np.copy(img)
    # Use cv2.resize().ravel() to create the feature vector
    features = cv2.resize(feature_image, size).ravel()
    # Return the feature vector
    return features

def color_hist(img, nbins=32, bins_range=(0, 256)):
    feature_image = np.copy(img)
    # Compute the histogram of the color channels separately
    channel1_hist = np.histogram(feature_image[:,:,0], bins=nbins, range=bins_range)
    channel2_hist = np.histogram(feature_image[:,:,1], bins=nbins, range=bins_range)
    channel3_hist = np.histogram(feature_image[:,:,2], bins=nbins, range=bins_range)
    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0]))
    # Return the individual histograms, bin_centers and feature vector
    return hist_features

def transform_features(img, cspace='BGR', orient=9, pix_per_cell=8, cell_per_block=2, hog_channel=0, spatial_size=(16, 16)):
    features = []

    file_features = []
    if cspace != 'BGR':
        if cspace == 'HSV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        elif cspace == 'LUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGR2LUV)
        elif cspace == 'HLS':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        elif cspace == 'YUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        elif cspace == 'YCrCb':
            feature_image = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    else: feature_image = np.copy(img)

    if hog_channel == 'ALL':
        hog_features = []
        for channel in range(feature_image.shape[2]):
            hog_features.append(get_hog_features(feature_image[:,:,channel],
                                orient, pix_per_cell, cell_per_block,
                                vis=False, feature_vec=True))
        hog_features = np.ravel(hog_features)
    else:
        hog_features = get_hog_features(feature_image[:,:,hog_channel], orient,
                    pix_per_cell, cell_per_block, vis=False, feature_vec=True)

    file_features.append(hog_features)

    spatial_features = bin_spatial(feature_image, size=spatial_size)
    file_features.append(spatial_features)

    # color_features = color_hist(feature_image)
    # file_features.append(color_features)

    features.append(np.concatenate(file_features))

    return np.array(features)

def extract_features(imgs, cspace='BGR', orient=9, pix_per_cell=8, cell_per_block=2, hog_channel=0, spatial_size=(16, 16)):
    # Create a list to append feature vectors to
    features = []
    # Iterate through the list of images
    for file in imgs:
        file_features = []
        # Read in each one by one
        image = cv2.imread(file)
        if cspace != 'BGR':
            if cspace == 'HSV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            elif cspace == 'LUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_BGR2LUV)
            elif cspace == 'HLS':
                feature_image = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
            elif cspace == 'YUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
            elif cspace == 'YCrCb':
                feature_image = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
        else: feature_image = np.copy(image)

        # Call get_hog_features() with vis=False, feature_vec=True
        if hog_channel == 'ALL':
            hog_features = []
            for channel in range(feature_image.shape[2]):
                hog_features.append(get_hog_features(feature_image[:,:,channel],
                                    orient, pix_per_cell, cell_per_block,
                                    vis=False, feature_vec=True))
            hog_features = np.ravel(hog_features)
        else:
            hog_features = get_hog_features(feature_image[:,:,hog_channel], orient,
                        pix_per_cell, cell_per_block, vis=False, feature_vec=True)
        # Append the new feature vector to the features list
        file_features.append(hog_features)

        spatial_features = bin_spatial(feature_image, size=spatial_size)
        file_features.append(spatial_features)

        # color_features = color_hist(feature_image)
        # file_features.append(color_features)

        features.append(np.concatenate(file_features))

    # Return list of feature vectors
    return np.array(features)

if __name__ == "__main__":
    svm_classifier(accuracy=True)
