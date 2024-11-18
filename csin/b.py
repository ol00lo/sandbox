import cv2
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt

class Stop(tf.keras.callbacks.Callback):
    def __init__(self, monitor, value):
        super().__init__()
        self.monitor = monitor
        self.value = value

    def on_epoch_end(self, epoch, logs):
        current = logs.get(self.monitor)
        if current <= self.value:
            self.model.stop_training = True

# base model
inp = tf.keras.layers.Input((1,))
dense1 = tf.keras.layers.Dense(20, activation='sigmoid')
dense2 = tf.keras.layers.Dense(20, activation='sigmoid')
dense3 = tf.keras.layers.Dense(20, activation='sigmoid')
output = tf.keras.layers.Dense(1, activation='linear')
x = inp
x = dense1(x)
x = dense2(x)
x = dense3(x)
x = output(x)
model = tf.keras.models.Model(inputs=inp, outputs=x)
model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-2), loss='mse')
x_train = 20*np.random.random((1000, 1))-10.0
y_train = np.sin(x_train)
x_valid = 20*np.random.random((200, 1))-10.0
y_valid = np.sin(x_valid)
cb = [Stop("val_loss", 1e-4)]
res = model.fit(
     x_train, y_train,
     batch_size = 32,
     epochs=5000,
     validation_data=(x_valid, y_valid),
     callbacks=cb)

# plots
x_range = np.linspace(-15, 15, 1000).reshape(-1, 1)
y_pred = model.predict(x_range)
plt.plot(x_range, y_pred, label='model(x)')
plt.plot(x_range, np.sin(x_range), label='sin(x)')
plt.plot(x_range, np.abs(y_pred - np.sin(x_range)), label='|model(x) - sin(x)|')
plt.legend()
plt.show()



# 
def create_model():
    inp = tf.keras.layers.Input((1,))
    dense1 = tf.keras.layers.Dense(9, activation='sigmoid')
    dense2 = tf.keras.layers.Dense(5, activation='sigmoid')
    dense3 = tf.keras.layers.Dense(5, activation='sigmoid')
    output = tf.keras.layers.Dense(1, activation='linear')
    x = inp
    x = dense1(x)
    #x = dense2(x)
    #x = dense3(x)
    x = output(x)
    model = tf.keras.models.Model(inputs=inp, outputs=x)
    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-2), loss='mse')
    return model

with open(f"mseresults.txt", "w") as file:
    for n in range(1):
        model = create_model()
        lr = 7e-3
        bs = 16
        model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=lr), loss="mse")
        x_train = 20*np.random.random((1000, 1))-10.0
        y_train = np.sin(x_train)
        x_valid = 20*np.random.random((200, 1))-10.0
        y_valid = np.sin(x_valid)
        cb = [Stop("val_loss", 1e-4)]
        res = model.fit(
            x_train, y_train,
            batch_size = bs,
            epochs=5000,
            validation_data=(x_valid, y_valid),
            callbacks=cb)
        file.write(f"learning_rate: {lr}, batch_size: {bs}, epochs_to_convergence: {len(res.history['loss'])}\n")
        model.save("model.h5")