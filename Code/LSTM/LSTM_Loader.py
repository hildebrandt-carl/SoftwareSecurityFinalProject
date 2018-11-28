# LSTM for sequence classification in the IMDB dataset
import numpy
from keras.datasets import imdb
from keras.models import model_from_json
from keras.preprocessing import sequence

# fix random seed for reproducibility
numpy.random.seed(7)

# load the dataset but only keep the top n words, zero the rest
top_words = 5000
(X_train, y_train), (X_test, y_test) = imdb.load_data(num_words=top_words)

# truncate and pad input sequences
max_review_length = 500
X_train = sequence.pad_sequences(X_train, maxlen=max_review_length)
X_test = sequence.pad_sequences(X_test, maxlen=max_review_length)

# load json and create model
json_file = open('saved_models/model.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
loaded_model = model_from_json(loaded_model_json)
# load weights into new model
loaded_model.load_weights("saved_models/model.h5")
print("Loaded model from disk")

# Compile and run model
loaded_model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
scores = loaded_model.evaluate(X_test, y_test, verbose=0)
print("Accuracy: %.2f%%" % (scores[1]*100))