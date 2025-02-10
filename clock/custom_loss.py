import tensorflow as tf
import numpy as np

def diff(y_true, y_pred):
    y_true = tf.convert_to_tensor(y_true)
    y_pred = tf.convert_to_tensor(y_pred)
    y_true = tf.cast(y_true, dtype=tf.float32)
    y_pred = tf.cast(y_pred, dtype=tf.float32)
    y_true = tf.math.floormod(y_true, 720)
    y_pred = tf.math.floormod(y_pred, 720)

    return (y_pred - y_true + 360) % 720 - 360

def custom_loss(y_true, y_pred):
    return tf.reduce_mean(tf.square(diff(y_true, y_pred)))

def build_accuracy_metrics(delta):
    def custom_accuracy(y_true, y_pred):
        angle_diff = diff(y_true, y_pred)
        return tf.reduce_mean(tf.cast(tf.abs(angle_diff) <= delta, tf.float32))

    return custom_accuracy


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
    
    accuracy_fn = build_accuracy_metrics(5)
    y_true = tf.constant([700.0])
    y_pred = tf.constant([710.0])
    print(f"Accuracy {0} == {accuracy_fn(y_true, y_pred)}")
    y_true = tf.constant([700.0, 710.0])
    y_pred = tf.constant([710.0, 710.0])
    print(f"Accuracy {1/2} == {accuracy_fn(y_true, y_pred)}")
    y_true = tf.constant([700.0, 710.0, 720.0])
    y_pred = tf.constant([710.0, 710.0, 716.0])
    print(f"Accuracy {2/3} == {accuracy_fn(y_true, y_pred)}")
    y_true = tf.constant([700.0, 710.0, 720.0, 719.0])
    y_pred = tf.constant([710.0, 710.0, 716.0, 710.0])
    print(f"Accuracy {2/4} == {accuracy_fn(y_true, y_pred)}")
    y_true = tf.constant([700.0, 710.0, 720.0, 719.0, 1.0])
    y_pred = tf.constant([710.0, 710.0, 716.0, 710.0, 719.0])
    print(f"Accuracy {3/5} == {accuracy_fn(y_true, y_pred)}")
    y_true = tf.constant([700.0, 710.0, 720.0, 719.0, 1.0, 718.0])
    y_pred = tf.constant([710.0, 710.0, 716.0, 710.0, 719.0, 1.0])
    print(f"Accuracy {4/6} == {accuracy_fn(y_true, y_pred)}")
    y_true = tf.constant([700.0, 710.0, 720.0, 719.0, 1.0, 718.0, 0.0])
    y_pred = tf.constant([710.0, 710.0, 716.0, 710.0, 719.0, 1.0, 720.0])
    print(f"Accuracy {5/7} == {accuracy_fn(y_true, y_pred)}")
    y_true = tf.constant([700.0, 710.0, 720.0, 719.0, 1.0, 718.0, 0.0, 718.0])
    y_pred = tf.constant([710.0, 710.0, 716.0, 710.0, 719.0, 1.0, 720.0, 719.0])
    print(f"Accuracy {6/8} == {accuracy_fn(y_true, y_pred)}")
    
    
    print(f"Accuracy {1.0} == {accuracy_fn(tf.constant([2]), tf.constant([718]))}")
    print(f"Accuracy {0.0} == {accuracy_fn(tf.constant([2]), tf.constant([716]))}")
    print(f"Accuracy {1.0} == {accuracy_fn(tf.constant([2]), tf.constant([722]))}")
    print(f"Accuracy {1.0} == {accuracy_fn(tf.constant([2]), tf.constant([1442]))}")
    print(f"Accuracy {0.0} == {accuracy_fn(tf.constant([-4]), tf.constant([1442]))}")
    print(f"Accuracy {1.0} == {accuracy_fn(tf.constant([-721]), tf.constant([2161]))}")