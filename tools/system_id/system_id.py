import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt

def loadFiles(filenames):
    if len(filenames) == 0:
        return None

    Data = np.empty((0,5))
    for filename in filenames:
        fileData = np.loadtxt(filename, delimiter=',', skiprows=1)
        lastValidIdx = np.argwhere( (fileData[:,2] < 3.7) & (fileData[:,1] == 0))
        if len(lastValidIdx) > 0:
            lastValidIdx = lastValidIdx[0][0]
        else:
            lastValidIdx = -1
        print(lastValidIdx)
        fileData = fileData[0:lastValidIdx]
        print(fileData)
        Data = np.vstack((Data, fileData))

    filteredData = Data[ (Data[:,1] >= 8000) & (Data[:,1] <= 0.96 * 65536) ]

    X = filteredData[:, 1] / 65536 # PWM
    Y = filteredData[:, 2] / 4.2 # Vol
    Z = filteredData[:, 0] / 4 # Thrust (gram)
    maxThrust = filteredData[:, 3] / 4

    return X, Y, Z, maxThrust

def system_id(filenames, validations=[]):
    X, Y, Z, maxThrust = loadFiles(filenames)
    if len(validations) > 0:
        X_val, Y_val, Z_val, maxThrust_val = loadFiles(validations)

    plt.scatter(X, Z, label="training data")
    plt.xlabel("PWM")
    plt.ylabel("Thrust")
    
    if len(validations) > 0:
        plt.scatter(X_val, Z_val, label="validation data")
        plt.xlabel("PWM")
        plt.ylabel("Thrust")
    plt.legend()
    plt.show()
    
    Input = np.zeros((X.shape[0], 4))
    Input[:, 0] = X
    Input[:, 1] = Y
    Input[:, 2] = X ** 2
    Input[:, 3] = X * Y
    
    if len(validations) > 0:
        Input_val = np.zeros((X_val.shape[0], 4))
        Input_val[:, 0] = X_val
        Input_val[:, 1] = Y_val
        Input_val[:, 2] = X_val ** 2
        Input_val[:, 3] = X_val * Y_val
    
    reg = LinearRegression().fit(Input, Z)
    print("******************************")
    print("C_00 = " + str(reg.intercept_))
    print("C_10 = " + str(reg.coef_[0]))
    print("C_01 = " + str(reg.coef_[1]))
    print("C_20 = " + str(reg.coef_[2]))
    print("C_11 = " + str(reg.coef_[3]))
    print("score = " + str(reg.score(Input, Z)))
    if len(validations) > 0:
        Error = reg.predict(Input_val) - Z_val.reshape(X_val.shape[0])
        print("validation score = " + str(reg.score(Input_val, Z_val)))
        print("validation max error = " + str(np.max(np.abs(Error))) + " gram")
        print("validation mean error = " + str(np.mean(np.abs(Error))) + " gram")
    
    Input = np.zeros((X.shape[0], 4))
    Input[:, 0] = Z
    Input[:, 1] = Y
    Input[:, 2] = Z ** 2
    Input[:, 3] = Z * Y
    
    if len(validations) > 0:
        Input_val = np.zeros((X_val.shape[0], 4))
        Input_val[:, 0] = Z_val
        Input_val[:, 1] = Y_val
        Input_val[:, 2] = Z_val ** 2
        Input_val[:, 3] = Z_val * Y_val
    
    reg = LinearRegression().fit(Input, X)
    print("******************************")
    print("d00: " + str(reg.intercept_))
    print("d10: " + str(reg.coef_[0]))
    print("d01: " + str(reg.coef_[1]))
    print("d20: " + str(reg.coef_[2]))
    print("d11: " + str(reg.coef_[3]))
    print("score = " + str(reg.score(Input, X)))
    if len(validations) > 0:
        Error = reg.predict(Input_val) - X_val.reshape(X_val.shape[0])
        print("validation score = " + str(reg.score(Input_val, X_val)))
        print("validation max error = " + str(np.max(np.abs(Error))) + " pwm")
        print("validation mean error = " + str(np.mean(np.abs(Error))) + " pwm")

    Input = np.zeros((X.shape[0], 2))
    Input[:, 0] = X
    Input[:, 1] = Y

    reg = LinearRegression().fit(Input, maxThrust)
    print("******************************")
    print("e00: " + str(reg.intercept_))
    print("e10: " + str(reg.coef_[0]))
    print("e01: " + str(reg.coef_[1]))
    print("score = " + str(reg.score(Input, maxThrust)))


system_id(["data_max_thrust_brushless01.csv", "data_max_thrust_brushless03.csv"],
          validations=["data_max_thrust_brushless01.csv"])
