import matplotlib.pyplot as plt

num_files = 6
num_cols = 3
num_rows = (num_files + num_cols - 1) // num_cols

fig, axes = plt.subplots(num_rows, num_cols, figsize=(12, 8))

if num_rows == 1:
    axes = [axes]

filenum = 0
for ax, file_name in zip(axes.flat, ['../output/output_0.txt', '../output/output_1.txt', '../output/output_2.txt', '../output/output_3.txt', '../output/output_4.txt', '../output/output_5.txt']):
    with open(file_name, 'r') as file:
        data = file.readlines()

    x_values = []
    y_values = []
    for line in data:
        x, y, _ = map(float, line.strip().split(','))
        x_values.append(x)
        y_values.append(y)

    ax.plot(x_values, y_values, label=file_name)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_title(f"Iteration {filenum}")
    filenum += 1

plt.tight_layout()
plt.show()
