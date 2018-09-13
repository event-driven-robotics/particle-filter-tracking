from read_dataset import load_spike_train
from matplotlib.patches import Circle
import matplotlib.pyplot as plt

def processAndPlot(video_sequence, output_position):

    if video_sequence:
        frames_to_show = min(len(video_sequence), len(output_position))
    else:
        frames_to_show = len(output_position)

    fig, ax = plt.subplots()
    plt.gray()

    for i in range(frames_to_show) :
        ax.cla()
        if video_sequence:
            ax.imshow(video_sequence[i])
        ax.add_patch(Circle((output_position[i][0], output_position[i][1]), output_position[i][2]))

        # plt.show()
        plt.pause(0.01)
