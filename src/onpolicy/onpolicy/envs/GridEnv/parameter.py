
# save training data
SUMMARY_WINDOW = 16  ##32  # how many training steps before writing data to tensorboard
LOAD_MODEL = False  # do you want to load the model trained before
SAVE_IMG_GAP = 100  # how many episodes before saving a gif

# map and planning resolution
CELL_SIZE = 0.1  # meter, your map resolution
NODE_RESOLUTION = 1.0  # meter, your node resolution
FRONTIER_CELL_SIZE = 2 * CELL_SIZE  # do you want to downsample the frontiers

# map representation
FREE = 254  # value of free cells in the map
OCCUPIED = 0  # value of obstacle cells in the map
UNKNOWN = 205  # value of unknown cells in the map

# sensor and utility range
SENSOR_RANGE = 3  # meter
UTILITY_RANGE = 0.8 * SENSOR_RANGE  # consider frontiers within this range as observable
MIN_UTILITY = 0.5  # ignore the utility if observable frontiers are less than this value

# updating map range w.r.t the robot
UPDATING_MAP_SIZE =   (SENSOR_RANGE + NODE_RESOLUTION)/2  # nodes outside this range will not be affected by current measurements

# training parameters
MAX_EPISODE_STEP =64 ##128
REPLAY_SIZE = 5000 ##10000
MINIMUM_BUFFER_SIZE = 2000
BATCH_SIZE = 64 ##128
LR = 1e-5
GAMMA = 1
NUM_META_AGENT = 2 ##16  # how many threads does your CPU have

# network parameters
NODE_INPUT_DIM = 4
EMBEDDING_DIM = 64 ##128

# Graph parameters
K_SIZE = 25  # the number of neighboring nodes, fixed
NODE_PADDING_SIZE = 360  # the number of nodes will be padded to this value, need it for batch training


