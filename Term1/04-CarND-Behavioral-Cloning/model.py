from keras.models import Sequential, model_from_json
from keras.layers import Dense, Dropout, Activation, Flatten, Convolution2D, MaxPooling2D, Lambda, ELU
from keras.layers.normalization import BatchNormalization
from keras.optimizers import Adam

import cv2
import csv
import numpy as np
import os
from random import random
from sklearn.model_selection import train_test_split

DATA_PATH = './data/t1/'


def trans_image(image,steer,trans_range):
    #
    # Translate image
    # Ref: https://chatbotslife.com/using-augmentation-to-mimic-human-driving-496b569760a9#.s1pwczi3q
    #
    rows, cols, _ = image.shape
    tr_x = trans_range*np.random.uniform()-trans_range/2
    steer_ang = steer + tr_x/trans_range*2*.2
    tr_y = 40*np.random.uniform()-40/2
    Trans_M = np.float32([[1,0,tr_x],[0,1,tr_y]])
    image_tr = cv2.warpAffine(image,Trans_M,(cols,rows))

    return image_tr, steer_ang

def gen_data(X, y, batch_size=128, validation=False):
    #
    # Generate data for fit_generator
    #
    gen_start = 0
    while True:
        features = []
        labels = []

        if gen_start >= len(y):
            gen_start = 0
        ending = min(gen_start+batch_size, len(y))

        for idx, row in enumerate(y[gen_start:ending]):
            center_img = cv2.imread(DATA_PATH + X[gen_start+idx][0].strip())
            center_img = cv2.cvtColor(center_img, cv2.COLOR_BGR2HSV)
            center_label = float(row[0])

            # Augmentation 1: Jitter image
            center_img, center_label = trans_image(center_img, center_label, 100)

            # Augmentation 2: Occasionally flip straight
            if random() > 0.5 and abs(center_label) > 0.1:
                center_img = cv2.flip(center_img, 1)
                labels.append(-center_label)
            else:
                labels.append(center_label)

            # Augmentation 3: Random brightness
            random_bright = .25 + np.random.uniform()
            center_img[:,:,2] = center_img[:,:,2]*random_bright

            features.append(center_img)

            if not validation:
                # Augmentation 4: +0.25 to Left Image
                left_img = cv2.imread(DATA_PATH + X[gen_start+idx][1].strip())
                features.append(left_img)
                labels.append(float(row[0]) + 0.15)

                # Augmentation 5: -0.25 to Right Image
                right_img = cv2.imread(DATA_PATH + X[gen_start+idx][2].strip())
                features.append(right_img)
                labels.append(float(row[0]) - 0.15)

        gen_start += batch_size

        features = np.array(features)
        labels = np.array(labels)

        yield features, labels

def nvidia_model(row=66, col=200, ch=3, dropout=0.3, lr=0.0001):
    #
    # NVIDIA CNN model
    # Ref: https://arxiv.org/abs/1604.07316
    #
    input_shape = (row, col, ch)
    model = Sequential()
    model.add(BatchNormalization(axis=1, input_shape=input_shape))
    model.add(Convolution2D(24, 5, 5, border_mode='valid',
        subsample=(2, 2), activation='elu'))
    model.add(Dropout(dropout))
    model.add(Convolution2D(36, 5, 5, border_mode='valid',
        subsample=(2, 2), activation='elu'))
    model.add(Dropout(dropout))
    model.add(Convolution2D(48, 5, 5, border_mode='valid',
        subsample=(2, 2), activation='elu'))
    model.add(Dropout(dropout))
    model.add(Convolution2D(64, 3, 3, border_mode='valid',
        subsample=(1, 1), activation='elu'))
    model.add(Dropout(dropout))
    model.add(Convolution2D(64, 3, 3, border_mode='valid',
        subsample=(1, 1), activation='elu'))
    model.add(Dropout(dropout))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Activation('elu'))
    model.add(Dropout(dropout))
    model.add(Dense(50))
    model.add(Activation('elu'))
    model.add(Dropout(dropout))
    model.add(Dense(10))
    model.add(Activation('elu'))
    model.add(Dropout(dropout))
    model.add(Dense(1))
    model.add(Activation('elu'))

    model.compile(optimizer=Adam(lr=lr), loss='mse', metrics=['accuracy'])
    print(model.summary())
    return model

def nvidialite_model(row=33, col=100, ch=3, dropout=0.3, lr=0.0001):
    #
    # Modified of NVIDIA CNN Model (Dysfunctional)
    #
    input_shape = (row, col, ch)
    model = Sequential()
    model.add(BatchNormalization(axis=1, input_shape=input_shape))
    model.add(Convolution2D(24, 5, 5, border_mode='valid',
        subsample=(2, 2), activation='elu'))
    model.add(Convolution2D(36, 5, 5, border_mode='valid',
        subsample=(2, 2), activation='elu'))
    model.add(Convolution2D(48, 3, 3, border_mode='valid',
        subsample=(1, 1), activation='elu'))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Activation('elu'))
    model.add(Dropout(dropout))
    model.add(Dense(50))
    model.add(Activation('elu'))
    model.add(Dropout(dropout))
    model.add(Dense(10))
    model.add(Activation('elu'))
    model.add(Dropout(dropout))
    model.add(Dense(1))
    model.add(Activation('elu'))

    model.compile(optimizer=Adam(lr=lr), loss='mse', metrics=['accuracy'])
    print(model.summary())
    return model

def load_data(filter=True):
    #
    # Load and split data
    # CSV: center,left,right,steering,throttle,brake,speed
    #
    total = 0
    with open(DATA_PATH + 'driving_log.csv', 'r') as f:
        reader = csv.reader(f)
        data = [row for row in reader]

    data = np.array(data)
    X = data[:,[0,1,2]]
    y = data[:,[3]]

    print('Total samples:', total)
    print('Total samples (after filter):', len(X))

    return train_test_split(X, y, test_size=0.2, random_state=42)

def load_model(lr=0.001):
    #
    # Load the existing model and weight
    #
    with open('model.json', 'r') as jfile:
        model = model_from_json(jfile.read())
    model.compile(optimizer=Adam(lr=lr), loss='mse', metrics=['accuracy'])
    model.load_weights('model.h5')
    return model

def main():
    # Load data
    X_train, X_val, y_train, y_val = load_data()

    print('X_train shape:', X_train.shape)
    print('X_val shape:', X_val.shape)

    # Build model
    if 'model.json' in os.listdir():
        model = load_model()
    else:
        model = nvidia_model()

    model.fit_generator(gen_data(X_train, y_train),
        samples_per_epoch=len(X_train)*3, nb_epoch=8,
        validation_data=gen_data(X_val, y_val, validation=True),
        nb_val_samples=len(X_val))

    # Save model
    json = model.to_json()
    model.save_weights('model.h5')
    with open('model.json', 'w') as f:
        f.write(json)

if __name__ == "__main__": main()
