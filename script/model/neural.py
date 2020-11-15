#!/usr/bin/env python
# coding: utf-8



import cv2
import os
import numpy as np
from matplotlib import pyplot as plt
import tensorflow as tf



no_intersection = [cv2.cvtColor(cv2.imread("data/false/"+i), cv2.COLOR_BGR2GRAY) for i in os.listdir("data/false")]
intersection = [cv2.cvtColor(cv2.imread("data/true/"+i), cv2.COLOR_BGR2GRAY) for i in os.listdir("data/true")]



'''for i,image in enumerate(no_intersection):
    no_intersection[i] = cv2.resize(no_intersection[i],(320,240))
for i,image in enumerate(intersection):
    intersection[i] = cv2.resize(intersection[i],(320,240))

'''

x_1 = np.array(no_intersection)
x_2 = np.array(intersection)
x_1.shape



true = [1 for i in range(len(intersection))]
false = [0 for i in range(len(no_intersection))]



train_x = np.array(no_intersection+intersection)
train_x = train_x / 255.0
train_y = np.array(false + true)
train_x.shape



model = tf.keras.Sequential([
    tf.keras.layers.Flatten(input_shape=(480,640)),
    tf.keras.layers.Dense(30,activation='relu'),
    tf.keras.layers.Dense(100,activation='relu'),
    tf.keras.layers.Dense(50,activation='relu'),
    tf.keras.layers.Dense(2,activation='relu')
])
model.summary()



model.compile(optimizer='adam',
              loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])
model.fit(train_x, train_y, epochs=20)



print(train_x[0].shape)
predictions = model.predict(x_1)
output = []
for item in predictions:
    if(item[0] > item[1]):
        output.append(0)
    else:
        output.append(1)
print(output)



model.save('model.h5') 



model1 = tf.keras.models.load_model('model.h5')



predictions = model1.predict(x_2)
output = []
for item in predictions:
    if(item[0] > item[1]):
        output.append(0)
    else:
        output.append(1)
print(output)

