from keras.layers import Dense
from keras.models import Sequential
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler

#Dataset appartement [Pieces, Surface]
x_data = np.array([[1,23],[4,82],[2,48],[2,55],[3,61],[2,54],[3,63],[4,86],[4,77],[4,87],[3,77],[4,84],[3,50]])
y_data = np.array([[397],[440],[645],[731],[747],[785],[864],[1037],[860],[1242],[743],[798],[600]])

x_scaler = MinMaxScaler()
x_data = x_scaler.fit_transform(x_data)

y_scaler = MinMaxScaler()
y_data = y_scaler.fit_transform(y_data)
print(x_data)
print(y_data)

sep = 10
x_train = x_data[:sep]
y_train = y_data[:sep]

x_test = x_data[sep:]
y_test = y_data[sep:]

print(x_train.shape)
print(y_train.shape)
print(x_test.shape)
print(y_test.shape)

model = Sequential()
model.add(Dense(13, input_shape=x_data.shape[1:]))
model.add(Dense(1))

model.compile(optimizer='adam', loss='mean_squared_error')
model.fit(x_train,y_train, epochs=20, validation_data=(x_test, y_test))

test_out = model.predict(x_test)
test_out = y_scaler.inverse_transform(test_out)

print("Real outputs: \n",y_scaler.inverse_transform(y_test))
print("NN outputs: \n", test_out)

model.save("keras_house_prediction_model.h5")

#Test 
model.load_weights("C:/Users/ledonger/Documents/keras_house_prediction_model.h5")
x_topredict = np.array([[4,80]])
print("Input", x_topredict)
x_topredict = x_scaler.transform(x_topredict)
print("Raw input: ", x_topredict)
y_predicted = model.predict(x_topredict)
print("Raw prediction: ",y_predicted)
y_predicted = y_scaler.inverse_transform(y_predicted)

print("Prediction: "+str(y_predicted))
