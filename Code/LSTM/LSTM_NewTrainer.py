# LSTM for sequence classification in the IMDB dataset
import numpy
import matplotlib.pyplot as plt  

from keras.datasets import imdb
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers.embeddings import Embedding
from keras.preprocessing import sequence

# fix random seed for reproducibility
numpy.random.seed(7)

# Load the training and testing data
x_train = numpy.load("FinalFiles/FinalTrainingData.npy")
y_train = numpy.load("FinalFiles/FinalTrainingLabels.npy") ;
x_test = numpy.load("FinalFiles/FinalTestingData.npy")
y_test = numpy.load("FinalFiles/FinalTestingLabels.npy") ;

# Check the data is the correct shape
print("Training data shape: " + str(x_train.shape))
print("Training labels shape: " + str(y_train.shape))
print("Testing data shape: " + str(x_test.shape))
print("Testing labels shape: " + str(y_test.shape))

print("====================================================")

# Creating the model
embedding_vecor_length = 32
model = Sequential()
model.add(LSTM(150, return_sequences=False,
					input_shape=(2000, 367)))
model.add(Dense(1, activation='sigmoid'))
model.compile(loss='binary_crossentropy',
			  optimizer='adam',
			  metrics=['accuracy'])
print(model.summary())
history = model.fit(x_train, y_train,
					epochs=100,
					batch_size=64,
					validation_split=0.2,
					shuffle=True)

print("====================================================")

# Plot the loss
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'validation'], loc='upper left')
plt.show()

# Final evaluation of the model
scores = model.evaluate(x_test, y_test, verbose=0)
print("Final Testing Accuracy: %.2f%%" % (scores[1]*100))

print("====================================================")
print("Saving the models")

# Serialize model to JSON
model_json = model.to_json()
with open("saved_models/model.json", "w") as json_file:
    json_file.write(model_json)
# Serialize weights to HDF5
model.save_weights("saved_models/model.h5")
print("Saved model to disk")
