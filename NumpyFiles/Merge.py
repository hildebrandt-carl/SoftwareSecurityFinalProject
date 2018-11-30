import os
import numpy as np

results = []
for f in os.listdir():
        if f.endswith('.npy'):
            results.append(f)
			
print(results)

data_file = []
for filename in results:
	# Load the data file
	data = np.load(filename)
	# Crop the data file to the first 2000 elements
	data = data[:2000]
	# Append to the data list
	data_file.append(data)

	# Figure out if this was a good or a bad data file
	if "good" in filename:
		print("Good Run")
	elif "bad" in filename:
		print("Bad Run")


# Stack the final data together
final_data = np.stack(data_file)

# Print the final data
print(final_data.shape)
np.save("FinalData.npy", final_data)

