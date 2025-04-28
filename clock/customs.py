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
        #tf.print("\nTRUE\n", y_true)
        #tf.print("\nPRED\n", y_pred)
        angle_diff = diff(y_true, y_pred, 720)
        return tf.reduce_mean(tf.cast(tf.abs(angle_diff) <= delta, tf.float32))
    custom_accuracy.__name__ = name 
    return custom_accuracy

def old_loss(y_true, y_pred):
    #tf.print("\nTRUE\n", y_true)
    #tf.print("\nPRED\n", y_pred)
    return tf.reduce_mean(tf.square(diff(y_true, y_pred, 720)))
