# Behavioral Learning

## Preprocessing
Every data that are feed into training go through some modifications including resizing and cropping. The preprocessing is done before the training started in order to reduce the training time. If the training is to include processing the images on-the-fly, the total training time will be going to be lengthy. The command for ImageMagick can be found below:

```sh
mogrify -path ./IMG -format jpg -resize "200x66^" -gravity north -crop 200x66+0+15 +repage "./IMGori/*.jpg"
```

Besides, the driving_log.csv is edited by removing the unnecessary path that is produced by the simulator.

### Resubmission
Please refer to the sample images from the dataset as included in the folder.

## Data Augmentation
I decided to augment the training data for two main purposes, to create more training data and to help generalization. To create more data, I used all three angles (left, center, and right). An angle of 0.25 is added to the left camera and 0.25 is deduceted from the right camera to create an illusion of the car is driving from the left or the right side of the lane. With this, the training data is expanded 3-fold. To help on generalization, flipping and brightness adjustment are used. Occasionally, the center image is flip horizontally. When it is flipped, the steering angle is inverted too. A randomized brightness is added on every center image to reduce the over-reliance on the colour brightness.

### Resubmission
On my first submission, the self-driving car was a hit and miss. I decided to improve the data by adding another data augmentation so that the car can generalize better. I referred to https://chatbotslife.com/using-augmentation-to-mimic-human-driving-496b569760a9#.s1pwczi3q for clues. I included his method of translating image and adjusting the steering angle.

## Training Model
The training model that is used for the training is based on NVIDIA's End to End Learning for Self-Driving Cars paper (Ref: https://arxiv.org/abs/1604.07316) with slight modifications. According to the paper, the model is trained for images with HSV colour model. Therefore, the training data that are feeded into the model are converted to HSV colour model before training.

```
____________________________________________________________________________________________________
Layer (type)                     Output Shape          Param #     Connected to                     
====================================================================================================
batchnormalization_1 (BatchNorma (None, 66, 200, 3)    264         batchnormalization_input_1[0][0]
____________________________________________________________________________________________________
convolution2d_1 (Convolution2D)  (None, 31, 98, 24)    1824        batchnormalization_1[0][0]       
____________________________________________________________________________________________________
dropout_1 (Dropout)              (None, 31, 98, 24)    0           convolution2d_1[0][0]            
____________________________________________________________________________________________________
convolution2d_2 (Convolution2D)  (None, 14, 47, 36)    21636       dropout_1[0][0]                  
____________________________________________________________________________________________________
dropout_2 (Dropout)              (None, 14, 47, 36)    0           convolution2d_2[0][0]            
____________________________________________________________________________________________________
convolution2d_3 (Convolution2D)  (None, 5, 22, 48)     43248       dropout_2[0][0]                  
____________________________________________________________________________________________________
dropout_3 (Dropout)              (None, 5, 22, 48)     0           convolution2d_3[0][0]            
____________________________________________________________________________________________________
convolution2d_4 (Convolution2D)  (None, 3, 20, 64)     27712       dropout_3[0][0]                  
____________________________________________________________________________________________________
dropout_4 (Dropout)              (None, 3, 20, 64)     0           convolution2d_4[0][0]            
____________________________________________________________________________________________________
convolution2d_5 (Convolution2D)  (None, 1, 18, 64)     36928       dropout_4[0][0]                  
____________________________________________________________________________________________________
dropout_5 (Dropout)              (None, 1, 18, 64)     0           convolution2d_5[0][0]            
____________________________________________________________________________________________________
flatten_1 (Flatten)              (None, 1152)          0           dropout_5[0][0]                  
____________________________________________________________________________________________________
dense_1 (Dense)                  (None, 100)           115300      flatten_1[0][0]                  
____________________________________________________________________________________________________
activation_1 (Activation)        (None, 100)           0           dense_1[0][0]                    
____________________________________________________________________________________________________
dropout_6 (Dropout)              (None, 100)           0           activation_1[0][0]               
____________________________________________________________________________________________________
dense_2 (Dense)                  (None, 50)            5050        dropout_6[0][0]                  
____________________________________________________________________________________________________
activation_2 (Activation)        (None, 50)            0           dense_2[0][0]                    
____________________________________________________________________________________________________
dropout_7 (Dropout)              (None, 50)            0           activation_2[0][0]               
____________________________________________________________________________________________________
dense_3 (Dense)                  (None, 10)            510         dropout_7[0][0]                  
____________________________________________________________________________________________________
activation_3 (Activation)        (None, 10)            0           dense_3[0][0]                    
____________________________________________________________________________________________________
dropout_8 (Dropout)              (None, 10)            0           activation_3[0][0]               
____________________________________________________________________________________________________
dense_4 (Dense)                  (None, 1)             11          dropout_8[0][0]                  
____________________________________________________________________________________________________
activation_4 (Activation)        (None, 1)             0           dense_4[0][0]                    
====================================================================================================
```

I use ELU instead of RELU for activation because I found that the model not only generalize better but also reduced loss. A dropout layer (of 0.3) is added after each layer to help the generalization too.

Before using the full-blown NVIDIA CNN model, I created a smaller model based on the original NVIDIA model. The intention is to use smaller images (100x33) because the images produced by the simulator are not as detailed as the normal images for real world. However, the model did not perform well and therefore I discarded the effort.


## Training Process
The trainings were done on batches. I uses the concept of transfer learning in training the model. It was first train on the sample data provided by Udacity. Using the model based on the sample data, the car could self-drive until the bridge where it drove off the track. I suspected the training data are not enough to determine the steering direction of the bridge. I then loaded the same model and weights but train it on a different set of training data (24679 images) which I produced myself. The car drove better where it hit another corner and failed. I then produced another set of training data (18306 images) which purposely trained to tackle the hard corners I identified. Using the final model, it drove well only if I changed the throttle to 0.15

### Resubmission
After introducing another data augmentation method, I noticed the training loss go down very slowly. I adjusted the learning rate from 0.0001 to 0.001. Although it improved but the data converged very slowly. Initially, I started with 6 epochs for each dataset. I later increased the number of epoch to 8. For the second dataset, the training loss and validation loss did not converge on the first attempt. Instead of increasing the number of epoch, I just reload the model and weights, and restart the whole training again. I monitored the validation loss instead of training loss because the validation loss is calculated on samples outside of training set and it only focuses on the center camera. The validation loss decreases slowly, at the end of 2nd attempt on the 2nd dataset, the validation loss plateau at 0.0096.
