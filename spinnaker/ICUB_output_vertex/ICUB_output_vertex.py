from pacman.model.graphs.machine import MachineSpiNNakerLinkVertex


class ICUBOutputVertex(MachineSpiNNakerLinkVertex):

    def __init__(self, label, spinnaker_link_id, board_address,
                 constraints=None):
        MachineSpiNNakerLinkVertex.__init__(
            self, spinnaker_link_id=spinnaker_link_id,
            board_address=board_address, label=label, constraints=constraints)

