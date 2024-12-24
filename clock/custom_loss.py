import tensorflow as tf
import numpy as np

def custom_loss(y_true, y_pred):
    y_true = tf.convert_to_tensor(y_true, dtype=tf.float32)
    y_pred = tf.convert_to_tensor(y_pred, dtype=tf.float32)

    y_true = tf.math.floormod(y_true, 720)
    y_pred = tf.math.floormod(y_pred, 720)

    delta = (y_pred - y_true + 360) % 720 - 360
    return tf.reduce_mean(tf.square(delta))

if __name__ == "__main__":
    true = np.array([0.0, 720.0])
    pred = np.array([720.0, 0.0])
    print(f"0 = {custom_loss(true, pred)}")
    true = np.array([1.0])
    pred = np.array([719.0])
    print(f"{2*2} = {custom_loss(true, pred)}")
    true = np.array([710])
    pred = np.array([350.0])
    print(f"{360*360} = {custom_loss(true, pred)}")
    true = np.array([1.0])
    pred = np.array([2.0])
    print(f"{1*1} = {custom_loss(true, pred)}")