import numpy as np
from tensorflow import keras


# loading a trained model
model = keras.models.load_model("models/xy_to_xy/xy_to_xy_model.keras")

# loading normalization parameters
with open('models/xy_to_xy/xy_to_xy_model_mean_std.txt', 'r') as f:
    data = f.read().split('\n')[0].split(' ')
    x_mean = float(data[0])
    x_std = float(data[1])
    y_mean = float(data[2])
    y_std = float(data[3])


# Generate x and y values
x_values = np.arange(100, 1801, 100)  # x from 100 to 1800 with step 100
y_values = np.arange(350, 1001, 50)   # y from 350 to 1000 with step 50

# Create a grid of (x, y) coordinates
X_mesh, Y_mesh = np.meshgrid(x_values, y_values)

# Flatten and combine into an array of shape (num_samples, 2)
data = np.column_stack((X_mesh.ravel(), Y_mesh.ravel()))

# normalize testing data
data_standardized = (data - x_mean) / x_std

# Make Predictions
predictions_standardized = model.predict(data_standardized, verbose=0)
predictions = predictions_standardized * y_std + y_mean

# Saving generated data and predictions
with open('testing_data/xy_to_xy_grid_viz_exp/xy_to_xy_testing_input.txt', 'w') as f:
    for i in range(data.shape[0]):
        f.write(f'{data[i][0]} {data[i][1]}\n')

with open('testing_data/xy_to_xy_grid_viz_exp/xy_to_xy_predictions.txt', 'w') as f:
    for i in range(data.shape[0]):
        f.write(f'{predictions[i][0]} {predictions[i][1]}\n')
