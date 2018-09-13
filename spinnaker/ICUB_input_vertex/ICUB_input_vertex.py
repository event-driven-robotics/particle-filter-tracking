from pacman.model.constraints.key_allocator_constraints import \
    FixedKeyAndMaskConstraint
from pacman.model.graphs.machine import MachineSpiNNakerLinkVertex
from pacman.model.routing_info import BaseKeyAndMask
from spinn_front_end_common.abstract_models.\
    abstract_provides_n_keys_for_partition import \
    AbstractProvidesNKeysForPartition
from spinn_front_end_common.abstract_models.\
    abstract_provides_outgoing_partition_constraints import \
    AbstractProvidesOutgoingPartitionConstraints
from spinn_utilities.overrides import overrides
from pf_spinn import constants


class ICUBInputVertex(
        MachineSpiNNakerLinkVertex,
        #AbstractProvidesNKeysForPartition,
        #AbstractProvidesOutgoingPartitionConstraints
):

    def __init__(self, label, spinnaker_link_id, board_address,
                 constraints=None):

        MachineSpiNNakerLinkVertex.__init__(
            self, spinnaker_link_id=spinnaker_link_id,
            board_address=board_address, label=label, constraints=constraints)
        #AbstractProvidesNKeysForPartition.__init__(self)
        #AbstractProvidesOutgoingPartitionConstraints.__init__(self)

    # @overrides(AbstractProvidesNKeysForPartition.get_n_keys_for_partition)
    # def get_n_keys_for_partition(self, partition, graph_mapper):
    #     return []

    # @overrides(AbstractProvidesOutgoingPartitionConstraints.
    #            get_outgoing_partition_constraints)
    # def get_outgoing_partition_constraints(self, partition):
    #     return [FixedKeyAndMaskConstraint(
    #         keys_and_masks=[BaseKeyAndMask(
    #             base_key=constants.RETINA_BASE_KEY,
    #             mask=constants.RETINA_MASK)])]

