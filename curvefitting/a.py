import numpy as np
import matplotlib.pyplot as plt
np.random.seed(8)


class F:
    def __init__(self):
        self.A = 0.5
        self.B = 0.5
        self.C = 0.5
        self.D = 0.5
        self.E = 0.5
        self.lr = 7e-4
        self.dvapi = 2 * np.pi

    def func(self, x):
        return self.A * np.sin(self.dvapi * self.B * x + self.C) + self.D * x + self.E

    def mse(self, x, y):
        r = y - self.func(x)
        return np.mean(r * r)

    def grad(self, x, y):
        f_x = self.func(x)
        m = self.dvapi * self.B * x + self.C
        k = -2 * (y - f_x)
        gA = k * np.sin(m)
        gB = k * (self.dvapi * x * self.A * np.cos(m))
        gC = k * (self.A * np.cos(m))
        gD = k * x
        gE = k
        return gA, gB, gC, gD, gE
    
    def train(self, xdata, ydata):
        for i in range(len(xdata)):
            grads = self.grad(xdata[i], ydata[i])
            self.A -= self.lr * grads[0]
            self.B -= self.lr * grads[1]
            self.C -= self.lr * grads[2]
            self.D -= self.lr * grads[3]
            self.E -= self.lr * grads[4]
            if i % 100 == 0:  
                print(f"MSE = {self.mse(xdata, ydata)}")


def generatedata(npoints, A, B, C, D, E, eps = 0.0):
    x = np.random.uniform(-2 *  np.pi, 2  * np.pi, npoints)
    y = (A * np.sin(2 * np.pi * B * x + C) + D * x + E + eps * np.random.uniform(-1, 1))
    return x, y

def calc(f, x, y, epochs):
    plt.figure(figsize=(10, 6))
    pointx = np.linspace(-2 * np.pi, 2 * np.pi, 400)
    for epoch in range(epochs):
        f.train(x, y)
        pointy = f.func(pointx)
        
        plt.clf()
        plt.scatter(x, y, color='blue', label='Input Data', s=30)
        plt.plot(pointx, pointy, color='red', label='Fitted Function')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.grid()
        plt.pause(0.01)





f = F()
Atrue = 0.9
Btrue = 0.4
Ctrue = 0.3
Dtrue = 0.4
Etrue = 1.0
npoints = 200
xdata, ydata = generatedata(npoints, Atrue, Btrue, Ctrue, Dtrue, Etrue, 0.01)
epochs = 25
calc(f, xdata, ydata, epochs)

print(f"\n\nTrue A = {Atrue}, A = {f.A}")
print(f"True B = {Btrue}, B = {f.B}")
print(f"True C = {Ctrue}, C = {f.C}")
print(f"True D = {Dtrue}, D = {f.D}")
print(f"True E = {Etrue}, E = {f.E}")
#plt.show()