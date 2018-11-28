# LSTM for sequence classification in the IMDB dataset
import numpy
import array
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

# Our files
bad = "this movie was terrible and bad"
good = "i really liked the movie and had fun"

# Classify the reviews
word_to_id = imdb.get_word_index()
for review in [good, bad]:
    tmp = []
    for word in review.split(" "):
        tmp.append(word_to_id[word])
    tmp_padded = sequence.pad_sequences([tmp], maxlen=max_review_length)
    # test_prediction = loaded_model.predict(X_train[0].reshape([1, max_review_length]))
    # print("%s. Sentiment: %s" % (y_train[0], test_prediction))
    prediction_output = loaded_model.predict(tmp_padded.reshape([1, max_review_length]))
    print("%s. Sentiment: %s" % (review, prediction_output))

