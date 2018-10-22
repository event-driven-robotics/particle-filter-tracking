from spinn_utilities.overrides import overrides
from pacman.model.graphs.machine import MachineSpiNNakerLinkVertex
from spinn_front_end_common.abstract_models \
    import AbstractSendMeMulticastCommandsVertex
from spinn_front_end_common.utility_models.multi_cast_command \
    import MultiCastCommand

class ICUBOutputVertex(MachineSpiNNakerLinkVertex,
                       AbstractSendMeMulticastCommandsVertex):

    def __init__(self, label, spinnaker_link_id, board_address,
                 constraints=None):
        MachineSpiNNakerLinkVertex.__init__(
            self, spinnaker_link_id=spinnaker_link_id,
            board_address=board_address, label=label, constraints=constraints)
        AbstractSendMeMulticastCommandsVertex.__init__(self)


    @property
    @overrides(AbstractSendMeMulticastCommandsVertex.start_resume_commands)
    def start_resume_commands(self):
        return [MultiCastCommand(
            key=0x80000000, payload=0, repeat=5, delay_between_repeats=100)]


    @property
    @overrides(AbstractSendMeMulticastCommandsVertex.pause_stop_commands)
    def pause_stop_commands(self):
        return [MultiCastCommand(
            key=0x40000000, payload=0, repeat=5, delay_between_repeats=100)]

    @property
    @overrides(AbstractSendMeMulticastCommandsVertex.timed_commands)
    def timed_commands(self):
        return []