from pacman.model.constraints.key_allocator_constraints import \
    FixedKeyAndMaskConstraint
from pacman.model.decorators.overrides import overrides
from pacman.model.graphs.machine import MachineVertex
from pacman.model.resources import CPUCyclesPerTickResource, DTCMResource
from pacman.model.resources import ResourceContainer, SDRAMResource
from pacman.model.routing_info import BaseKeyAndMask
from spinn_front_end_common.abstract_models import \
    AbstractProvidesOutgoingPartitionConstraints

from spinn_front_end_common.utilities import constants
from spinn_front_end_common.interface.simulation import simulation_utilities
from spinn_front_end_common.abstract_models.impl.machine_data_specable_vertex \
    import MachineDataSpecableVertex
from spinn_front_end_common.abstract_models.abstract_has_associated_binary \
    import AbstractHasAssociatedBinary
from spinn_front_end_common.interface.buffer_management.buffer_models\
    .abstract_receive_buffers_to_host import AbstractReceiveBuffersToHost
from spinn_front_end_common.utilities.utility_objs import ExecutableType
from spinn_front_end_common.utilities import helpful_functions
from spinn_front_end_common.abstract_models.\
    abstract_provides_n_keys_for_partition import \
    AbstractProvidesNKeysForPartition
from spinn_front_end_common.interface.buffer_management \
    import recording_utilities


from pf_spinn import constants as app_constants

import numpy
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class PfFullParticleVertex(
        MachineVertex, MachineDataSpecableVertex, AbstractHasAssociatedBinary,
        AbstractProvidesOutgoingPartitionConstraints,
        AbstractReceiveBuffersToHost,
        AbstractProvidesNKeysForPartition):

    DATA_REGIONS = Enum(
        value="DATA_REGIONS",
        names=[('SYSTEM', 0),
               ('TRANSMISSION_DATA', 1),
               ('CONFIG', 2),
               ('RECORDING', 3)])

    TRANSMISSION_DATA_SIZE = 16
    CONFIG_PARAM_SIZE = 24
    RECORD_BYTES_PER_STEP = 12

    KEYS_REQUIRED = 2

    def __init__(self, x, y, r, n_particles, label, part_id,
                 main_particle, constraints=None):
        MachineVertex.__init__(self, label=label, constraints=constraints)

        AbstractProvidesNKeysForPartition.__init__(self)

        self._x = x
        self._y = y
        self._r = r
        self._n_particles = n_particles
        self._placement = None
        self._part_id = part_id
        self._main = main_particle

    @property
    @overrides(MachineVertex.resources_required)
    def resources_required(self):
        sdram_required = (
            constants.SYSTEM_BYTES_REQUIREMENT +
            self.TRANSMISSION_DATA_SIZE + self.CONFIG_PARAM_SIZE)

        sdram_required += \
            app_constants.MACHINE_STEPS * self.RECORD_BYTES_PER_STEP

        resources = ResourceContainer(
            cpu_cycles=CPUCyclesPerTickResource(45),
            dtcm=DTCMResource(100), sdram=SDRAMResource(sdram_required))

        # resources.extend(recording_utilities.get_recording_resources(
        #     [app_constants.MACHINE_STEPS * self.RECORD_BYTES_PER_STEP],
        #     self._receive_buffer_host, self._receive_buffer_port))

        return resources

    @overrides(AbstractHasAssociatedBinary.get_binary_file_name)
    def get_binary_file_name(self):
        return "pf_fullparticle.aplx"

    @overrides(AbstractHasAssociatedBinary.get_binary_start_type)
    def get_binary_start_type(self):
        return ExecutableType.USES_SIMULATION_INTERFACE

    @overrides(AbstractProvidesNKeysForPartition.get_n_keys_for_partition)
    def get_n_keys_for_partition(self, partition, graph_mapper):
        if partition.identifier == app_constants.EDGE_PARTITION_PARTICLE_TO_PARTICLE:
            return self.KEYS_REQUIRED
        else:
            return 0  # shouldn't be used if mask/key is used instead

    @overrides(AbstractProvidesOutgoingPartitionConstraints.
               get_outgoing_partition_constraints)
    def get_outgoing_partition_constraints(self, partition):
        if partition.identifier == app_constants.EDGE_PARTITION_MAIN_TO_FILTER:
            return [FixedKeyAndMaskConstraint(
                keys_and_masks=[BaseKeyAndMask(
                    base_key=app_constants.MAIN_PARTICLE_ROI_KEY,
                    mask=app_constants.MESSAGE_TYPE_MASK)])]
        elif partition.identifier == app_constants.EDGE_PARTITION_TARGET_POSITION:
            return [FixedKeyAndMaskConstraint(
                keys_and_masks=[BaseKeyAndMask(
                    base_key=app_constants.MAIN_PARTICLE_TARGET_KEY,
                    mask=app_constants.MESSAGE_TYPE_MASK)])]
        elif partition.identifier == app_constants.EDGE_PARTITION_PARTICLE_TO_PARTICLE:
            return []
        else:
            raise Exception("Asking for a partition not defined")

    @overrides(MachineDataSpecableVertex.generate_machine_data_specification)
    def generate_machine_data_specification(
            self, spec, placement, machine_graph, routing_info, iptags,
            reverse_iptags, machine_time_step, time_scale_factor):

        #    HAS_KEY = 0, P2P_KEY = 1, FILTER_UPDATE_KEY = 2, OUTPUT_KEY = 3
        #    X_COORD = 0, Y_COORD = 1, RADIUS = 2, P2P_ID = 3, IS_MAIN = 4, N_PARTICLES = 5

        self._placement = placement

        # Setup words + 1 for flags + 1 for recording size
        setup_size = constants.SYSTEM_BYTES_REQUIREMENT

        # Create the data regions for hello world
        self._reserve_memory_regions(spec, setup_size)

        # write data for the simulation data item
        spec.switch_write_focus(self.DATA_REGIONS.SYSTEM.value)
        spec.write_array(simulation_utilities.get_simulation_header_array(
            self.get_binary_file_name(), machine_time_step,
            time_scale_factor))

        # write transmission key
        spec.switch_write_focus(self.DATA_REGIONS.TRANSMISSION_DATA.value)

        routing_key = routing_info.get_first_key_from_pre_vertex(
            self, app_constants.EDGE_PARTITION_PARTICLE_TO_PARTICLE)
        if routing_key is None:
            raise Exception("Error: the routing key is none!")
        else:
            #HAS_KEY
            spec.write_value(1)
            #P2P_KEY
            spec.write_value(routing_info.get_first_key_from_pre_vertex(
                self, app_constants.EDGE_PARTITION_PARTICLE_TO_PARTICLE))
        if self._main:
            #FILTER_UPDATE_KEY
            spec.write_value(routing_info.get_first_key_from_pre_vertex(
                self, app_constants.EDGE_PARTITION_MAIN_TO_FILTER))
            #OUTPUT_KEY
            spec.write_value(routing_info.get_first_key_from_pre_vertex(
                self, app_constants.EDGE_PARTITION_TARGET_POSITION))
        else:
            spec.write_value(0)
            spec.write_value(0)



        # write config params
        spec.switch_write_focus(self.DATA_REGIONS.CONFIG.value)
        spec.write_value(self._x)
        spec.write_value(self._y)
        spec.write_value(self._r)
        spec.write_value(self._part_id)
        if self._main:
            spec.write_value(1)
        else:
            spec.write_value(0)
        spec.write_value(self._n_particles)

        #initialise recording region
        spec.switch_write_focus(self.DATA_REGIONS.RECORDING.value)
        spec.write_array(recording_utilities.get_recording_header_array(
            [app_constants.MACHINE_STEPS * self.RECORD_BYTES_PER_STEP]))

        # End-of-Spec:
        spec.end_specification()

    def _reserve_memory_regions(self, spec, system_size):
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.SYSTEM.value, size=system_size,
            label='systemInfo')
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.TRANSMISSION_DATA.value,
            size=self.TRANSMISSION_DATA_SIZE,
            label="My Key")
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.CONFIG.value,
            size=self.CONFIG_PARAM_SIZE,
            label="Config x, y, r")
        spec.reserve_memory_region(
            region=self.DATA_REGIONS.RECORDING.value,
            size=recording_utilities.get_recording_header_size(1),
            label="Recording Region")

    def get_data(self, buffer_manager, placement):
        """ Read back the samples
        """

        # Read the data recorded
        data_values, _ = buffer_manager.get_data_for_vertex(placement, 0)
        converted = data_values.read_all()
        data_shape = list()
        data_shape.append(("x", numpy.float32))
        data_shape.append(("y", numpy.float32))
        data_shape.append(("r", numpy.float32))

        data_view = numpy.array(converted, dtype=numpy.uint8).view(data_shape)

        return data_view


    def get_minimum_buffer_sdram_usage(self):
        return self._string_data_size

    def get_n_timesteps_in_buffer_space(self, buffer_space, machine_time_step):
        return recording_utilities.get_n_timesteps_in_buffer_space(
            buffer_space,
            [app_constants.MACHINE_STEPS * self.RECORD_BYTES_PER_STEP])

    def get_recorded_region_ids(self):
        return [0]

    def get_recording_region_base_address(self, txrx, placement):
        return helpful_functions.locate_memory_region_for_placement(
            placement, self.DATA_REGIONS.RECORDING.value, txrx)

