# LSTM for sequence classification in the IMDB dataset
import numpy
from keras.datasets import imdb
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers.embeddings import Embedding
from keras.preprocessing import sequence

# fix random seed for reproducibility
numpy.random.seed(7)

# load the dataset but only keep the top n words, zero the rest
x_train = numpy.load("FinalData.npy")
y_train = numpy.ones((8,1)).reshape(-1) ;
y_train[2] = 0
y_train[3] = 0

print(x_train.shape)
print(y_train.shape)

# create the model
embedding_vecor_length = 32
model = Sequential()
model.add(LSTM(100, return_sequences=False, input_shape=(2000, 367)))
model.add(Dense(1, activation='sigmoid'))
model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
print(model.summary())
model.fit(x_train, y_train, epochs=100, batch_size=64)

# Final evaluation of the model
scores = model.evaluate(x_train, y_train, verbose=0)
print("Accuracy: %.2f%%" % (scores[1]*100))

# # Serialize model to JSON
# model_json = model.to_json()
# with open("saved_models/model.json", "w") as json_file:
#     json_file.write(model_json)
# # Serialize weights to HDF5
# model.save_weights("saved_models/model.h5")
# print("Saved model to disk")






