from enum import Enum

from pacman.model.constraints.key_allocator_constraints import \
    FixedKeyAndMaskConstraint
from pacman.model.graphs.machine import MachineVertex
from pacman.model.resources import ResourceContainer, \
    CPUCyclesPerTickResource, DTCMResource, SDRAMResource
from pacman.model.routing_info import BaseKeyAndMask
from spinn_front_end_common.abstract_models import \
    AbstractHasAssociatedBinary, \
    AbstractProvidesOutgoingPartitionConstraints, \
    AbstractProvidesIncomingPartitionConstraints
from spinn_front_end_common.abstract_models.impl import \
    MachineDataSpecableVertex
from spinn_front_end_common.interface.simulation import simulation_utilities
from spinn_front_end_common.utilities import constants
from spinn_front_end_common.utilities.utility_objs import ExecutableType
from pf_spinn import constants as app_constants


class RetinaFilter(
        MachineVertex, MachineDataSpecableVertex, AbstractHasAssociatedBinary,
        AbstractProvidesOutgoingPartitionConstraints,
        AbstractProvidesIncomingPartitionConstraints):

    CORE_APP_IDENTIFIER = 0xBEEF
    TRANSMISSION_DATA_SIZE = 8
    CONFIG_REGION_SIZE = 8

    DATA_REGIONS = Enum(
        value="DATA_REGIONS",
        names=[('SYSTEM', 0),
               ('TRANSMISSION_DATA', 1),
               ('CONFIG', 2)])

    def __init__(
            self, partition_identifier, filter, row_id, constraints=None):
        label = "retina filter for row {}".format(row_id)
        MachineVertex.__init__(self, label, constraints)
        MachineDataSpecableVertex.__init__(self)
        AbstractHasAssociatedBinary.__init__(self)
        AbstractProvidesOutgoingPartitionConstraints.__init__(self)
        self._partition_identifier = partition_identifier
        self._filter = filter
        self._row_id = row_id

    def get_outgoing_partition_constraints(self, partition):
        base_key = app_constants.FILTER_BASE_KEY | (
            self._row_id << app_constants.RETINA_Y_BIT_SHIFT)
        return [FixedKeyAndMaskConstraint(
            keys_and_masks=[BaseKeyAndMask(
                base_key=base_key,
                mask=app_constants.FILTER_BASE_MASK)])]

    def get_incoming_partition_constraints(self, partition):
        if partition.identifier == self._partition_identifier:
            base_key = app_constants.RETINA_BASE_KEY | \
                (self._row_id << app_constants.RETINA_Y_BIT_SHIFT)
            return [FixedKeyAndMaskConstraint(
                keys_and_masks=[BaseKeyAndMask(
                    base_key=base_key,
                    mask=app_constants.FILTER_BASE_MASK)])]
        return []

    @property
    def resources_required(self):
        sdram_required = (
            constants.SYSTEM_BYTES_REQUIREMENT +
            self.TRANSMISSION_DATA_SIZE +
            self.CONFIG_REGION_SIZE)
        resources = ResourceContainer(
            cpu_cycles=CPUCyclesPerTickResource(45),
            dtcm=DTCMResource(100), sdram=SDRAMResource(sdram_required))
        return resources

    def generate_machine_data_specification(
            self, spec, placement, machine_graph, routing_info,
            iptags, reverse_iptags, machine_time_step, time_scale_factor):
        # Create the data regions for hello world
        self._reserve_memory_regions(spec, constants.SYSTEM_BYTES_REQUIREMENT)
        # write data for the simulation data item
        spec.switch_write_focus(self.DATA_REGIONS.SYSTEM.value)
        spec.write_array(simulation_utilities.get_simulation_header_array(
            self.get_binary_file_name(), machine_time_step,
            time_scale_factor))

        # write transmission key
        spec.switch_write_focus(self.DATA_REGIONS.TRANSMISSION_DATA.value)

        out_going_routing_key = routing_info.get_first_key_from_pre_vertex(
            self, app_constants.EDGE_PARTITION_FILTER_TO_PARTICLES)

        if out_going_routing_key is None:
            spec.write_value(0)
            spec.write_value(0)
        else:
            spec.write_value(1)
            spec.write_value(out_going_routing_key)

        spec.switch_write_focus(self.DATA_REGIONS.CONFIG.value)
        spec.write_value(self._row_id)
        spec.write_value(app_constants.RETINA_X_SIZE)

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
            size=self.CONFIG_REGION_SIZE,
            label="config")

    def get_binary_file_name(self):
        return "roi_filter.aplx"

    def get_binary_start_type(self):
        return ExecutableType.USES_SIMULATION_INTERFACE
