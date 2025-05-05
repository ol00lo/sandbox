import os
import numpy as np
import random
import math
import tensorflow as tf
from tensorflow.keras.applications import MobileNet
from tensorflow.keras import layers, callbacks
import customs
from tensorflow.keras.models import load_model

#seed_value = np.random.randint(0, 2**32 - 1)
seed_value = 1
random.seed(seed_value)
np.random.seed(seed_value)

class SumMinutesLayer(layers.Layer):
    def call(self, inputs):
        hours, minutes = inputs
        hours = tf.math.floormod(tf.floor(hours), 12)
        minutes = tf.math.floormod(minutes, 60)
        sum_min = hours * 60 + minutes
        return tf.stop_gradient(sum_min)
    
    def compute_output_shape(self, input_shape):
        return input_shape[0]

def data_generator_from_folder(data_dir, batch_size, need_shuffle=True):
    data_files = sorted([f for f in os.listdir(data_dir) if f.startswith('data_') and f.endswith('.npy')])
    label_files = sorted([f for f in os.listdir(data_dir) if f.startswith('labels_') and f.endswith('.npy')])

    while True:
        indices = list(range(len(data_files)))
        if need_shuffle:
            random.shuffle(indices)
            
        for ind in indices:
            data_file = data_files[ind]
            label_file = label_files[ind]
            data_path = os.path.join(data_dir, data_file)
            labels_path = os.path.join(data_dir, label_file)
            
            batch_data = np.load(data_path)
            batch_labels = np.load(labels_path)

            sum_minutes = batch_labels
            hour_labels = (sum_minutes / 60)
            minute_labels = (sum_minutes % 60)

            if need_shuffle:
                bindices = np.linspace(0, len(batch_data)-1, len(batch_data), dtype=int)
                np.random.shuffle(bindices)
                batch_data = batch_data[bindices]
                hour_labels = hour_labels[bindices]
                minute_labels = minute_labels[bindices]
                sum_minutes = sum_minutes[bindices]

            for i in range(0, len(batch_data), batch_size):
                batch_images = batch_data[i:i + batch_size]
                batch_hour = hour_labels[i:i + batch_size]
                batch_minute = minute_labels[i:i + batch_size]
                batch_sum_minutes = sum_minutes[i:i + batch_size]

                batch_images = tf.keras.applications.mobilenet.preprocess_input(batch_images)

                yield batch_images, {'hours_output': batch_hour, 'minutes_output': batch_minute, 'sum_minutes': batch_sum_minutes}


def func():
    BATCH_SIZE = 64
    INIT_EPOCH = 0
    EPOCHS = 200
    train_data_dir = "train_data_dir"
    valid_data_dir = "valid_data_dir"
        
    train_generator = data_generator_from_folder(train_data_dir, BATCH_SIZE, True)
    valid_generator = data_generator_from_folder(valid_data_dir, BATCH_SIZE, False)

    base_model = MobileNet(weights='imagenet', include_top=False)
    base_model.trainable = True

    inp = tf.keras.layers.Input(shape=(224, 224, 3))
    x = base_model(inp, training=True)
    x = layers.GlobalAveragePooling2D()(x)
    x = layers.Dense(128, activation='relu')(x)
    x = layers.Dropout(0.5)(x)

    hours = layers.Dense(1, activation='linear', name='hours_output')(x)
    minutes = layers.Dense(1, activation='linear', name='minutes_output')(x)
    sum_minutes = SumMinutesLayer(name='sum_minutes')([hours, minutes])

    model = tf.keras.Model(inputs=inp, outputs=[hours, minutes, sum_minutes])
    #model = load_model('final_model.keras',
    #                   safe_mode=False,
    #                   custom_objects={ 'hours_loss': customs.hours_loss,
    #                                    'minutes_loss': customs.minutes_loss,
    #                                    'custom_accuracy': customs.build_accuracy_metrics(5),
    #                                    'old_loss': customs.old_loss,
    #                                    'SumMinutesLayer': SumMinutesLayer})

    log_dir = os.path.join("logs", "fit", "model_run")

    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=5e-4),
        loss={
            'hours_output': customs.hours_loss,
            'minutes_output': customs.minutes_loss,
            'sum_minutes': lambda y_true, y_pred: tf.zeros_like(y_true)
        },
        loss_weights={
            'hours_output': 0.5,
            'minutes_output': 0.5,
            'sum_minutes': 0.0
        },
        metrics={
            'sum_minutes': [
                customs.build_accuracy_metrics(10),
                customs.old_loss
           ]
        }
    )

    model_checkpoint = callbacks.ModelCheckpoint(
        filepath='models/ep{epoch:02d}_acc_{sum_minutes_custom_accuracy:.4f}.keras',
        save_best_only=True,
        monitor='sum_minutes_custom_accuracy',
        mode='max',
        verbose=1
    )

    tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir)
    callbacks_list = [tensorboard_callback, model_checkpoint]
    
    model.fit(
        train_generator,
        steps_per_epoch=math.ceil(11114 / BATCH_SIZE),
        validation_data=valid_generator,
        validation_steps=math.ceil(1235 / BATCH_SIZE),
        epochs=EPOCHS,
        initial_epoch=INIT_EPOCH,
        callbacks=callbacks_list
    )
    model.save("final_model.keras")
    print("Model saved as final_model.keras")

if __name__ == "__main__":
    func()