import numpy as np
import matplotlib.pyplot as plt

def load_spike_train(filename):

    try:
        file = open(filename, "r")
    except:
        print "Could not open file"
        return -1

    spike_train = [[]] * 1048576

    for line in file:
        line_split = line.split(";")
        index = int(line_split[0]) + (int(line_split[1]) << 9)

        line_split[2] = line_split[2][:-1]
        if line_split[2]:
            times = line_split[2].split(",")

            times = map(int, times)

            spike_train[index] = times

    file.close()

    return spike_train


def load_vbottle(filename, window_size=100, tsscaler=0.000000080, address_bits = 20, max_stamp = 0x00FFFFFF, height=240, width=304):

    try:
        file = open(filename, "r")
    except:
        print "Could not open file {}".format(filename)
        return -1

    #timestamp wraps
    tsscaler *= 1000
    ftime_ms = 0
    base_ts = -1
    wrap_ts = 0
    pts = 0

    #convert dataset to spike train
    spike_train = [[] for i in range((2 ** address_bits))]

    #convert dataset to event-windows
    video_sequence = []
    video_sequence.append(np.ones((height, width), dtype=np.uint8)*255)

    for line in file:

        _, _, line = line.partition("(")
        line, _, _ = line.partition(")")
        line = np.fromstring(line, dtype=np.uint32, sep=' ')
        timestamps = line[::2]
        events = line[1::2]
        address = (events >> 1) & 0x0001FFFF
        X = (events >> 1) & 0x01FF
        Y = (events >> 10) & 0x00FF

        if base_ts < 0 :
            base_ts = timestamps[0]

        for i in range(len(events)):

            if pts > timestamps[i] :
                pts = timestamps[i]
                wrap_ts += max_stamp

            ctime_ms = int((timestamps[i] + wrap_ts - base_ts) * tsscaler)+0.5

            spike_train[address[i]].append(ctime_ms)

            if(ctime_ms > ftime_ms + window_size) :
                video_sequence.append(np.ones((height, width), dtype=np.uint8) * 255)
                ftime_ms += window_size

            video_sequence[-1][Y[i]][X[i]] = 0

    return spike_train, video_sequence, ftime_ms+window_size
