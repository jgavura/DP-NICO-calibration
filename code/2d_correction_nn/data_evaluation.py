import numpy as np
from data_handling import load_data
from tablet_coords_conversion import sim2tab, pix2cm


def evaluate_data(targets, results):
    q1, q2, q3, q4, full = [], [], [], [], []
    for i in range(len(targets)):
        x, y = targets[i]
        dev = pix2cm(np.sqrt((x - results[i][0])**2 + (y - results[i][1])**2))
        full.append(dev)

        if x > 960 and y > 690:
            q2.append(dev)
        elif x <= 960 and y > 690:
            q1.append(dev)
        elif x > 960 and y <= 690:
            q3.append(dev)
        elif x <= 960 and y <= 690:
            q4.append(dev)

    print(f'q1 - len: {len(q1)}    mean: {np.mean(q1)}    std: {np.std(q1)}')
    print(f'q2 - len: {len(q2)}    mean: {np.mean(q2)}    std: {np.std(q2)}')
    print(f'q3 - len: {len(q3)}    mean: {np.mean(q3)}    std: {np.std(q3)}')
    print(f'q4 - len: {len(q4)}    mean: {np.mean(q4)}    std: {np.std(q4)}')
    print(f'full - len: {len(full)}    mean: {np.mean(full)}    std: {np.std(full)}')


m1_results, m1_targets = load_data('xy')

for i in range(len(m1_targets)):
    m1_targets[i] = np.array(sim2tab(m1_targets[i][0], m1_targets[i][1]))

print('Linear Model')
evaluate_data(m1_targets, m1_results)

for result in ['xy', 'xyz']:
    if result == 'xy':
        print('\nPartially nonlinear model')
    else:
        print('\nNonlinear model')

    results, targets = [], []
    for i in range(1, 11):
        file_name = f'testing_data/accuracy_exp/xy_to_{result}/xy_to_{result}_results{i}.txt'
        with open(file_name, 'r') as f:
            content = f.read().split('\n')
            for j in range(len(content)):
                if content[j] == '':
                    continue
                results.append(list(map(float, content[j].split(' '))))

        file_name = f'testing_data/accuracy_exp/targets/targets{i}.txt'
        with open(file_name, 'r') as f:
            content = f.read().split('\n')
            for j in range(len(content)):
                if content[j] == '':
                    continue
                targets.append(list(map(float, content[j].split(' '))))

    evaluate_data(targets, results)
