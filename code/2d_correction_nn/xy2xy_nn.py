import numpy as np
from matplotlib import pyplot as plt
from tensorflow import keras
from sklearn.model_selection import train_test_split
from sklearn.model_selection import KFold
from data_handling import load_data, standardize_data


# Load data xy to xy
data_x, data_y = load_data('xy')

# Normalize data using standardization
x_standardized, y_standardized, x_mean, x_std, y_mean, y_std = standardize_data(data_x, data_y)

print(f"M2 - mean: {0.0009 * y_std}, std: {0.0003 * y_std}")
print(f"M3 - mean: {0.0005 * y_std}, std: {0.0001 * y_std}")


# Split data into training (70%) and val set (30%) first
x_train, x_val, y_train, y_val = train_test_split(
    x_standardized,
    y_standardized,
    test_size=0.3,
    random_state=42,
    shuffle=True
)

# Split temp set into validation (15%) and testing (15%)
# x_val, x_test, y_val, y_test = train_test_split(
#     x_val,
#     y_val,
#     test_size=0.5,
#     random_state=42,
#     shuffle=True
# )


# Define model function
def create_xy2xy_model():
    model = keras.Sequential([
        keras.layers.Input(shape=(2,)),
        keras.layers.Dense(16, activation='relu'),
        keras.layers.Dense(2)
    ])

    model.compile(optimizer='adam', loss='mse', metrics=['mae'])

    return model


# # creating a nn model
# model = create_xy2xy_model()
#
# # training the model
# history = model.fit(
#     x_train, y_train,
#     validation_data=(x_val, y_val),
#     epochs=200,
#     batch_size=16,
#     verbose=1
# )
#
#
# # plot losses over epochs
# plt.subplot(1, 2, 1)
# plt.plot(history.history['loss'], label='Training MSE')
# plt.plot(history.history['val_loss'], label='Validation MSE')
# plt.title('MSE Over Epochs')
# plt.xlabel('Epochs')
# plt.ylabel('Mean Squared Error')
# plt.legend(loc='best')
#
# plt.subplot(1, 2, 2)
# plt.plot(history.history['mae'], label='Training MAE')
# plt.plot(history.history['val_mae'], label='Validation MAE')
# plt.title('MAE Over Epochs')
# plt.xlabel('Epochs')
# plt.ylabel('Mean Absolute Error')
# plt.legend(loc='best')
#
# plt.suptitle('xy to xy', fontsize=16)
# plt.tight_layout()
# plt.show()


# # Perform K-Fold Cross-Validation
# kf = KFold(n_splits=5, shuffle=True, random_state=42)
# mse_scores, mae_scores = [], []
#
# for train_idx, val_idx in kf.split(x_standardized):
#     X_train, X_val = x_standardized[train_idx], x_standardized[val_idx]
#     y_train, y_val = y_standardized[train_idx], y_standardized[val_idx]
#
#     # Create a new model for each fold
#     model = create_xy2xy_model()
#
#     # Train the model
#     model.fit(X_train, y_train, epochs=150, batch_size=16, verbose=0)
#
#     # Evaluate on validation set
#     mse, mae = model.evaluate(X_val, y_val, verbose=0)
#
#     mse_scores.append(mse)
#     mae_scores.append(mae)
#     print(f"Fold MSE: {mse:.4f}, Fold MAE: {mae:.4f}")
#
# # Compute the mean and standard deviation of loss across folds
# print(f"Mean MSE: {np.mean(mse_scores):.4f}, Std Dev MSE: {np.std(mse_scores):.4f}")
# print(f"Mean MAE: {np.mean(mae_scores):.4f}, Std Dev MAE: {np.std(mae_scores):.4f}")


# # Create and train the model on the full dataset
# model = create_xy2xy_model()
# model.fit(x_standardized, y_standardized, epochs=200, batch_size=16, verbose=1)  # No validation split
#
# # saving the model
# model.save("models/xy_to_xy/xy_to_xy_model.keras")
#
# # saving normalization parameters
# with open('models/xy_to_xy/xy_to_xy_model_mean_std.txt', 'w') as f:
#     f.write(f'{x_mean} {x_std} {y_mean} {y_std}')

