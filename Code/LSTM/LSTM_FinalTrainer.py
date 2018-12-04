# LSTM for sequence classification in the IMDB dataset
import numpy
import os
import matplotlib.pyplot as plt  

from keras.datasets import imdb
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers.embeddings import Embedding
from keras.preprocessing import sequence
from keras.callbacks import ModelCheckpoint
from keras.models import model_from_json

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

# Create a checkpoint to save the best validation loss
checkpoint = ModelCheckpoint('saved_models/model-{epoch:03d}.h5',
							verbose=1,
							monitor='val_loss',
							save_best_only=True,
							mode='auto')

# Create the model
model.compile(loss='binary_crossentropy',
			  optimizer='adam',
			  metrics=['accuracy'])

# Print a model summary
print()
print(model.summary())
print()

# Train the model
history = model.fit(x_train, y_train,
					epochs=250,
					batch_size=64,
					validation_split=0.2,
					callbacks=[checkpoint],
					shuffle=True)

print("====================================================")

# Find all the saved weights
results = []
for f in os.listdir("./saved_models/"):
        if f.endswith('.h5'):
            results.append("./saved_models/" + f)

# Get the best save file
final_save = max(results)
print(final_save)

# Work out which was the best model
best_model = int(final_save[21:24]) - 1

# Plot the accuracy
plt.plot(history.history['acc'])
plt.plot(history.history['val_acc'])
plt.axvline(x=best_model, color='r', linestyle='--')
plt.title('Model accuracy')
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'validation', 'best_model'], loc='upper left')
plt.show()

# Plot the loss
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.axvline(x=best_model, color='r', linestyle='--')
plt.title('Model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'validation', 'best_model'], loc='upper left')
plt.show()

print("====================================================")
print("Saving the model")

# Serialize model to JSON
model_json = model.to_json()
with open("saved_models/model.json", "w") as json_file:
    json_file.write(model_json)

print("====================================================")

# Reloading the best model
# Load json and create model
json_file = open('saved_models/model.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
loaded_model = model_from_json(loaded_model_json)

# load weights into new model
loaded_model.load_weights(final_save)
print("Loaded model from disk")

# Compile and run model
loaded_model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

# Final evaluation of the model
scores = loaded_model.evaluate(x_test, y_test, verbose=0)
print("Accuracy: %.2f%%" % (scores[1]*100))
exit()