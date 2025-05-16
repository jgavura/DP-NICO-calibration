import numpy as np
from tensorflow import keras


ITERATION = 4


def make_predictions(data, target_form='xy'):
    # loading a trained model
    model = keras.models.load_model(f"models/xy_to_{target_form}/xy_to_{target_form}_model.keras")

    # loading normalization parameters
    with open(f'models/xy_to_{target_form}/xy_to_{target_form}_model_mean_std.txt', 'r') as f:
        row = f.read().split('\n')[0].split(' ')
        x_mean = float(row[0])
        x_std = float(row[1])
        y_mean = float(row[2])
        y_std = float(row[3])

    # normalize testing data
    data_standardized = (data - x_mean) / x_std

    # Make Predictions
    predictions_standardized = model.predict(data_standardized, verbose=0)
    predictions = predictions_standardized * y_std + y_mean

    return predictions


for iteration in range(5, 11):
    # Generate x and y values
    x_values = np.random.randint(180, 1911, 25)  # 25 random numbers from 180 to 1910
    y_values = np.random.randint(410, 1071, 25)  # 25 random numbers from 410 to 1070

    # Flatten and combine into an array of shape (num_samples, 2)
    data = np.column_stack((x_values, y_values))

    # Saving generated data and predictions
    with open(f'testing_data/accuracy_exp/targets/targets{iteration}.txt', 'w') as f:
        for i in range(data.shape[0]):
            f.write(f'{data[i][0]} {data[i][1]}\n')

    # For every model
    for target_form in ['xy', 'xyz', 'joints']:
        predictions = make_predictions(data, target_form)

        with open(f'testing_data/accuracy_exp/xy_to_{target_form}/xy_to_{target_form}_predictions{iteration}.txt', 'w') as f:
            for i in range(data.shape[0]):
                f.write(f"{' '.join(map(str, predictions[i]))}\n")
