import numpy as np


data_files = [
    'exp_grid/grid',
    'exp_lines/diagonal/line_diagonal_1',
    'exp_lines/diagonal/line_diagonal_2',
    'exp_lines/horizontal/line_horizontal_1',
    'exp_lines/horizontal/line_horizontal_2',
    'exp_lines/vertical/line_vertical_1',
    'exp_lines/vertical/line_vertical_2',
    'exp_lines/vertical/line_vertical_3',
    'exp_random/ne/random_ne',
    'exp_random/nw/random_nw',
    'exp_random/se/random_se',
    'exp_random/sw/random_sw',
]


def load_data(target_form='xy'):
    print(target_form)
    data_x, data_y = [], []

    if target_form == 'joints':
        target_file = 'joint'
    else:
        target_file = 'cartesian'

    for file in data_files:
        with (open(f'data/{file}_cartesian_results.txt', 'r') as f_x,
              open(f'data/{file}_{target_file}_targets.txt', 'r') as f_y):
            f_data_x, f_data_y = f_x.read().split('\n'), f_y.read().split('\n')
            for i in range(len(f_data_x)):
                if f_data_x[i] == '':
                    continue
                data_x.append(tuple(map(float, f_data_x[i].split(' '))))
                if target_form == 'xy':
                    data_y.append(tuple(map(float, f_data_y[i].split(' ')))[:2])
                else:
                    data_y.append(tuple(map(float, f_data_y[i].split(' '))))

    return np.array(data_x), np.array(data_y)


def standardize_data(data_x, data_y):
    x_mean, x_std = data_x.mean(), data_x.std()
    y_mean, y_std = data_y.mean(), data_y.std()

    x_standardized = (data_x - x_mean) / x_std
    y_standardized = (data_y - y_mean) / y_std

    return x_standardized, y_standardized, x_mean, x_std, y_mean, y_std


data_x, data_y = load_data()
print(data_x[:5])
print(data_y[:5])

