class CommunicationLink:
    """Model a datalink that delivers packets late, out of order, or not at all.

    Each packet arrives after a fixed delay plus a random jitter, drawn
    uniformly between zero and the jitter bound, so packets can overtake each
    other. Each packet can also be lost at random, and every packet sent
    during an outage window is lost.
    """

    def __init__(self, rng, delay=0.0, jitter=0.0, loss_probability=0.0, outages=()):
        self.rng = rng
        self.delay = delay
        self.jitter = jitter
        self.loss_probability = loss_probability
        self.outages = tuple(outages)
        self.in_transit = []
        self.sent_count = 0
        self.lost_count = 0

    def send(self, time, packet):
        self.sent_count += 1

        # every packet draws the same random numbers whether or not it is
        # lost, so changing the loss probability leaves the delays unchanged
        jitter = self.rng.random() * self.jitter
        lost = self.rng.random() < self.loss_probability
        in_outage = any(start <= time < end for start, end in self.outages)
        if lost or in_outage:
            self.lost_count += 1
            return

        self.in_transit.append((time + self.delay + jitter, packet))

    def receive(self, time):
        # a small tolerance absorbs floating-point error in the accumulated time
        arrived = [item for item in self.in_transit if item[0] <= time + 1e-9]
        self.in_transit = [item for item in self.in_transit if item[0] > time + 1e-9]
        arrived.sort(key=lambda item: item[0])
        return [packet for _, packet in arrived]
