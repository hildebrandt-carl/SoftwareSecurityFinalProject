import os
import numpy as np

from random import shuffle

# Find all the files with npy
results = []
for f in os.listdir("./AttackRuns/"):
        if f.endswith('.npy'):
        	results.append("./AttackRuns/" + f)

for f in os.listdir("./CleanRuns/"):
        if f.endswith('.npy'):
            results.append("./CleanRuns/" + f)

# Check that you could find files
if len(results) < 1:
	print("No files could be found")
	exit()
			

# Check if FinalData has been created
for filename in results:
	if "Final" in filename:
		print("Please delete `FinalData.npy' or 'FinalLabels.npy'")
		exit()

# Shuffle the files so they are saved in a random order
shuffle(results)

# Get the length of the results
total_files = len(results)
print("Total Files Processed: " + str(total_files))

training_labels = np.ones((180,1)).reshape(-1) ;
testing_labels = np.ones((total_files - 180,1)).reshape(-1) ;

training_data = []
testing_data = []

file_counter = 0
for filename in results:
	# Creating the training data
	if file_counter < 180:
		# Load the data file
		print("Processing: " + str(filename))
		data = np.load(filename)
		# Crop the data file to the first 2000 elements
		data = data[:2000]
		# Append to the data list
		training_data.append(data)

		print("Shape of data: " + str(data.shape))

		# Figure out if this was a good or a bad data file
		if "Clean" in filename:
			print("Clean Run")
			training_labels[file_counter] = 0
			file_counter += 1
		elif "Attack" in filename:
			print("Attack Run")
			training_labels[file_counter] = 1
			file_counter += 1

		print("-----------------------------")
	else:
		# Load the data file
		print("Processing: " + str(filename))
		data = np.load(filename)
		# Crop the data file to the first 2000 elements
		data = data[:2000]
		# Append to the data list
		testing_data.append(data)

		print("Shape of data: " + str(data.shape))

		# Figure out if this was a good or a bad data file
		if "Clean" in filename:
			print("Clean Run")
			testing_labels[file_counter - 180] = 0
			file_counter += 1
		elif "Attack" in filename:
			print("Attack Run")
			testing_labels[file_counter - 180] = 1
			file_counter += 1

		print("-----------------------------")



# Stack the final data together
final_training_data = np.stack(training_data)
final_testing_data = np.stack(testing_data)

# Print the final data
print("Final Training Data Shape: " + str(final_training_data.shape))
print("Final Training Labels Shape: " + str(training_labels.shape))

# Print the final data
print("Final Testing Data Shape: " + str(final_testing_data.shape))
print("Final Testing Labels Shape: " + str(testing_labels.shape))

# Save the final data
np.save("FinalFiles/FinalTrainingData.npy", final_training_data)
np.save("FinalFiles/FinalTrainingLabels.npy", training_labels)

# Save the final data
np.save("FinalFiles/FinalTestingData.npy", final_testing_data)
np.save("FinalFiles/FinalTestingLabels.npy", testing_labels)

