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
        nb = len(x)
        gA = np.sum(k * np.sin(m)) / nb
        gB = np.sum(k * (self.dvapi * x * self.A * np.cos(m))) / nb
        gC = np.sum(k * (self.A * np.cos(m))) / nb
        gD = np.sum(k * x) / nb
        gE = np.sum(k) / nb
        return gA, gB, gC, gD, gE
    
    def train (self, xdata, ydata, batchsize = 1):
        nbatches = len(xdata) // batchsize
        for i in range(nbatches):
            start = i * batchsize
            end = (i + 1) * batchsize
            grads = self.grad(xdata[start:end], ydata[start:end])
            
            self.A -= self.lr * grads[0]
            self.B -= self.lr * grads[1]
            self.C -= self.lr * grads[2]
            self.D -= self.lr * grads[3]
            self.E -= self.lr * grads[4]

            if(i%10==0):
                mse = self.mse(xdata, ydata)
                #print(f"MSE = {mse}")
                #self.plotresults(xdata[0:end], ydata[0:end])
        return mse

    def plotresults(self, xdata, ydata):
        pointx = np.linspace(-self.dvapi, self.dvapi, 400)
        pointy = self.func(pointx)
        plt.clf()
        plt.scatter(xdata, ydata, color='blue', label='Input Data', s=30)
        plt.plot(pointx, pointy, color='red', label='Fitted Function')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.grid()
        plt.pause(0.01)

def generatedata(npoints, A, B, C, D, E, eps = 0.0):
    x = np.random.uniform(-2 *  np.pi, 2  * np.pi, npoints)
    y = A * np.sin(2 * np.pi * B * x + C) + D * x + E + eps * np.random.uniform(-10, 10, npoints)
    return x, y


def calc_mse(f, x, y, epochs, batchsize):
    for epoch in range(epochs):
        lastmse = f.train(x, y, batchsize)
    return lastmse

def func_old(f, xdata, ydata):
    f = F()
    epochs = 1
    batchsize = 10
    lastmse = calc_mse(f, xdata, ydata, epochs, batchsize)
    print(f"\n\nTrue A = {Atrue}, A = {f.A}")
    print(f"True B = {Btrue}, B = {f.B}")
    print(f"True C = {Ctrue}, C = {f.C}")
    print(f"True D = {Dtrue}, D = {f.D}")
    print(f"True E = {Etrue}, E = {f.E}")
    plt.show()

def func2(xdata, ydata):
    with open(f"mseresults.txt", "w") as file:
        file.write(f"Input data size = {len(xdata)} ")
        for nepochs in range(5, 31, 5):
            file.write(f"\nResults for nepochs = {nepochs}:\n\n")
            for batchsize in range(1, 15, 1):
                f = F()
                lastmse = calc_mse(f, xdata, ydata, nepochs, batchsize)
                file.write(f"Batch size: {batchsize}. Last MSE: {lastmse}\n")
    print("Results have been saved to mse_results.txt.")

def calc_epoch(f, x, y, batchsize, acc):
    lastmse = f.mse(x, y)
    epoch = 0
    while lastmse > acc:
         epoch+=1
         lastmse = f.train(x, y, batchsize)

    return epoch

def func3(xdata, ydata):
    acc = 5e-2
    i = 0
    with open(f"epochsresults.txt", "w") as file:
        file.write(f"to achieve accuracy = {acc} you need epochs\n")
        for batchsize in range(1, 15, 1):
            f = F()
            nepochs = calc_epoch(f, xdata, ydata, batchsize, acc)
            file.write(f"Batch size: {batchsize}, nepochs= {nepochs}\n")
            print(f"{batchsize}")
    print("Results have been saved to mse_results.txt.")



Atrue = 0.9
Btrue = 0.4
Ctrue = 0.3
Dtrue = 0.4
Etrue = 1.0
npoints = 200
xdata, ydata = generatedata(npoints, Atrue, Btrue, Ctrue, Dtrue, Etrue, 0.01)

func2(xdata, ydata)