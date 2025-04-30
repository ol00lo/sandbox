import tensorflow as tf
import numpy as np

def diff(y_true, y_pred, N):
    y_true = tf.convert_to_tensor(y_true)
    y_pred = tf.convert_to_tensor(y_pred)
    y_true = tf.cast(y_true, dtype=tf.float32)
    y_pred = tf.cast(y_pred, dtype=tf.float32)
    y_true = tf.math.floormod(y_true, N)
    y_pred = tf.math.floormod(y_pred, N)
    return (y_pred - y_true + N // 2) % N - N//2


def hours_loss(y_true, y_pred):
    #tf.print("\nHOURS TRUE\n", y_true)
    #tf.print("\nHOURS PRED\n", y_pred)
    return tf.reduce_mean(tf.square(diff(y_true, y_pred, 12)/12))

def minutes_loss(y_true, y_pred):
    #tf.print("\nMINUTE TRUE\n", y_true)
    #tf.print("\nMINUTE PRED\n", y_pred)
    return tf.reduce_mean(tf.square(diff(y_true, y_pred, 60)/60))

def build_accuracy_metrics(delta, name = "custom_accuracy"):
    def custom_accuracy(y_true, y_pred):
        angle_diff = diff(y_true, y_pred, 720)
        accuracy = tf.reduce_mean(tf.cast(tf.abs(angle_diff) <= delta, tf.float32))
        #tf.print("[DEBUG] y_true:", y_true[:3], "y_pred:", y_pred[:3], "accuracy:", accuracy)
        return accuracy
    custom_accuracy.__name__ = name 
    return custom_accuracy

def old_loss(y_true, y_pred):
    #tf.print("\nTRUE\n", y_true)
    #tf.print("\nPRED\n", y_pred)
    return tf.reduce_mean(tf.square(diff(y_true, y_pred, 720)))

if __name__ == "__main__":
    true = np.array([0.0, 720.0, 4.0, 713.2])
    pred = np.array([720.0, 0.0, 1.0, 718.1])
    print(f"1 == {build_accuracy_metrics(5)(true, pred)}")
    true = np.array([1312.0])
    pred = np.array([-128.5])
    print(f"-0.5 == {diff(true, pred, 720)}")
    true = np.array([23, -1, 23])
    pred = np.array([11, -13,-1])
    print(f"0 == {diff(true, pred, 12)}")
    true = np.array([90, -30])
    pred = np.array([30, 150])
    print(f"0 == {diff(true, pred, 60)}")
