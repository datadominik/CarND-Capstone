import numpy as np
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Dense, Flatten, GlobalAveragePooling2D, Activation, Dropout
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import ModelCheckpoint
from keras.models import load_model
from keras import backend as K
import tensorflow as tf

from tensorflow.python.framework import graph_util
from tensorflow.python.framework import graph_io

BATCH_SIZE=16
TARGET_SIZE = (32,32)

np.random.seed(1337)

model = Sequential()
model.add(Conv2D(16, (3,3), input_shape=(32,32,3), activation="relu"))
model.add(MaxPooling2D((2,2)))
model.add(Conv2D(32, (3,3), activation="relu"))
model.add(MaxPooling2D((2,2)))
model.add(Flatten())
model.add(Dense(50, activation="relu"))
model.add(Dropout(0.5))
model.add(Dense(3, activation="softmax"))

model.compile(loss="categorical_crossentropy", optimizer="adam", metrics=['accuracy'])

print("Model summary")
model.summary()

train_datagen = ImageDataGenerator(
        rescale=1/255.,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True,
        height_shift_range=0.2,
        width_shift_range=0.2,
        rotation_range=0.2)

test_datagen = ImageDataGenerator(rescale=1/255.)

train_generator = train_datagen.flow_from_directory(
        '/data/train/',
        batch_size=BATCH_SIZE,
        target_size=TARGET_SIZE,
        color_mode='rgb',
        class_mode='categorical')

validation_generator = test_datagen.flow_from_directory(
        '/data/valid/',
        batch_size=1,
        target_size=TARGET_SIZE,
        color_mode='rgb',
        class_mode='categorical')

_callbacks = [ModelCheckpoint('best_weights.h5', monitor='val_loss', verbose=2,
                save_best_only=True, save_weights_only=True, mode='auto')]

print("Model training started")
model.fit_generator(
        train_generator,
        steps_per_epoch=len(train_generator.classes)//BATCH_SIZE,
        epochs=150,
        validation_data=validation_generator,
        validation_steps=len(validation_generator.classes)//BATCH_SIZE,
        verbose=2,
        callbacks=_callbacks)

print("Model training complete")
model.load_weights('best_weights.h5')
model.save("model.h5")

print("Evaluating model")
model.evaluate_generator(validation_generator, steps=len(validation_generator.classes),
        use_multiprocessing = False)

print("Preparing for inference")
num_output = 1
K.set_learning_phase(0)
net_model = load_model('model.h5')

pred = [None]*num_output
pred_node_names = [None]*num_output
for i in range(num_output):
    pred_node_names[i] = "out_"+str(i)
    pred[i] = tf.identity(net_model.outputs[i], name=pred_node_names[i])

sess = K.get_session()

constant_graph = graph_util.convert_variables_to_constants(sess, sess.graph.as_graph_def(), pred_node_names)
graph_io.write_graph(constant_graph, './', 'models/classification.pb', as_text=False)
print('Model ready for inference')
