"""
Hello World program on Spinnaker

Each core stores into its region in SDRAM the string:
"Hello World from $chip.x, $chip.y, $core"

We then fetch the written data and print it on the python console.
"""

import logging
import time
# import front end
import spinnaker_graph_front_end as front_end
# import graph components
from pacman.model.graphs.machine import MachineEdge
# import for binary location
import cbin
# constants
from helpers import constants
from ICUB_input_vertex.ICUB_input_vertex import ICUBInputVertex
from ICUB_output_vertex.ICUB_output_vertex import ICUBOutputVertex
from pf_fullparticle.pf_fullparticle_vertex import PfFullParticleVertex
from roi_filter.roi_filter_vertex import RetinaFilter
from helpers.process_and_plot import processAndPlot
from helpers.read_dataset import load_vbottle
from helpers.read_dataset import load_spike_train

from spinn_front_end_common.utility_models. \
    reverse_ip_tag_multicast_source_machine_vertex import \
    ReverseIPTagMulticastSourceMachineVertex

# logger!
logger = logging.getLogger(__name__)

# state variables
run_online = True
use_spinn_link = True
filename = "/home/aglover/workspace/datasets/spinnaker_tracking/1/ATIS/data.log"
#filename = "data.log.spiking.txt"
machine_time_step = constants.US_PER_STEP #us
operation_time = constants.US_PER_STEP * constants.MACHINE_STEPS / 1000 #ms
time_scale_factor = 1
n_particles = constants.PARTICLES
n_chips_required = 4
spinnaker_link_used = 0

print "Loading Dataset"
spike_train = []
video_sequence = []
if not use_spinn_link:
    spike_train, video_sequence, data_time_ms = \
        load_vbottle(filename=filename,
                     window_size=constants.US_PER_STEP/1000,
                     tsscaler=0.000000320)
    if spike_train == -1:
        quit()
    print "Dataset goes for {} ms".format(data_time_ms)



front_end.setup(n_chips_required=n_chips_required,
                model_binary_module=cbin,
                machine_time_step=machine_time_step)

# VERTICES
particle_list = list()
filter_list = list()

# create "input"
# running with test data use this vertex
if use_spinn_link:
    # when running on the icub we'll need this vertex
    input_vertex = ICUBInputVertex(
        spinnaker_link_id=spinnaker_link_used, board_address=None,
        label="Input Vertex")
    front_end.add_machine_vertex_instance(input_vertex)

else:
    input_vertex = ReverseIPTagMulticastSourceMachineVertex(
        virtual_key=constants.RETINA_BASE_KEY,
        buffer_notification_ip_address="0.0.0.0",
        n_keys=1048576,
        send_buffer_max_space=20*1024*1024,
        label="Input Vertex", send_buffer_times=spike_train)
    front_end.add_machine_vertex_instance(input_vertex)

# create "output"
output_vertex = ICUBOutputVertex(
    spinnaker_link_id=spinnaker_link_used, board_address=None,
    label="Output Vertex")
front_end.add_machine_vertex_instance(output_vertex)


# create retina filters and edges from retina to filters
for y_row in range(0, constants.RETINA_Y_SIZE):
    partition_identifier = "retina_slice_row_{}".format(y_row)
    vertex = RetinaFilter(
        partition_identifier=partition_identifier, filter=y_row,
        row_id=y_row)
    filter_list.append(vertex)
    front_end.add_machine_vertex_instance(vertex)
    front_end.add_machine_edge_instance(
        MachineEdge(pre_vertex=input_vertex, post_vertex=vertex,
                    label="Edge between retina and filter"),
        partition_identifier)

# create particles
main_particle = True
the_main_particle = None
for x in range(0, n_particles):
    vertex = PfFullParticleVertex(
        x=constants.RETINA_X_SIZE/2, y=constants.RETINA_Y_SIZE/2,
        r=constants.INITIAL_R, batch_size=constants.MAX_BATCH_SIZE,
        n_particles=n_particles, part_id=x,
        label="Particle {}".format(x), main_particle=main_particle)

    if main_particle:
        the_main_particle = vertex
        main_particle = False

    front_end.add_machine_vertex_instance(vertex)
    particle_list.append(vertex)


# EDGES from main_particle to filters
for filter_vertex in filter_list:
    front_end.add_machine_edge_instance(
        MachineEdge(the_main_particle, filter_vertex),
        constants.EDGE_PARTITION_MAIN_TO_FILTER)

# EDGES from filter to particles
for x in range(0, n_particles):
    for filter_vertex in filter_list:
        front_end.add_machine_edge_instance(
            MachineEdge(
                filter_vertex, particle_list[x],
                label="Edge Input to P{}".format(x)),
            constants.EDGE_PARTITION_FILTER_TO_PARTICLES)

# EDGES from particles to particles
for x in range(0, n_particles):
    for y in range(0, n_particles):
        if(x == y):
            continue
        front_end.add_machine_edge_instance(
            MachineEdge(
                particle_list[x],
                particle_list[y], label="Edge P{} to P{}".format(x, y)),
            constants.EDGE_PARTITION_PARTICLE_TO_PARTICLE)

# EDGES from main_particle to output
front_end.add_machine_edge_instance(
    MachineEdge(
        the_main_particle,
        output_vertex,
        label="Final Result Edge"),
    constants.EDGE_PARTITION_TARGET_POSITION)

if(run_online):
    front_end.run(None)
    time.sleep(0.5);
    print 'Experiment Started with',constants.PARTICLES,'particles and',\
        constants.MAX_BATCH_SIZE,'batch size.'
    raw_input("Press Enter to stop operation...")
else:
    front_end.run(operation_time)

    # used with test data
    placements = front_end.placements()
    buffer_manager = front_end.buffer_manager()
    placement = placements.get_placement_of_vertex(the_main_particle)
    data = the_main_particle.get_data(buffer_manager, placement)
    print "The simulation recorded {} steps".format(len(data))
    #processAndPlot(video_sequence, data)


#print "Data Extracted from Buffer"
#print "Size {}".format(data.shape)
#print(data)

front_end.stop()

