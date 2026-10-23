import numpy as np

from robust_flight.communication.link import CommunicationLink

DT = 0.01


def send_every_fifth_of_a_second(link, steps=6000):
    """Send the send time as the packet every 0.2 s; return (sent, arrived)."""
    time = 0.0
    deliveries = []
    for step in range(1, steps + 1):
        time += DT
        if step % 20 == 0:
            link.send(time, time)
        for sent in link.receive(time):
            deliveries.append((sent, time))
    return np.array(deliveries).reshape(-1, 2)


def test_default_link_delivers_every_packet_in_the_same_step():
    deliveries = send_every_fifth_of_a_second(
        CommunicationLink(np.random.default_rng(0))
    )

    assert len(deliveries) == 300
    assert np.array_equal(deliveries[:, 0], deliveries[:, 1])


def test_link_delay_is_exact():
    link = CommunicationLink(np.random.default_rng(0), delay=0.35)
    deliveries = send_every_fifth_of_a_second(link)

    assert np.allclose(deliveries[:, 1] - deliveries[:, 0], 0.35)


def test_jitter_stays_within_bounds_and_reorders_packets():
    link = CommunicationLink(np.random.default_rng(1), delay=0.1, jitter=0.4)
    deliveries = send_every_fifth_of_a_second(link)
    delays = deliveries[:, 1] - deliveries[:, 0]

    assert delays.min() >= 0.1 - 1e-9
    # arrivals are noticed at the next step, so up to one step late
    assert delays.max() <= 0.5 + DT + 1e-9
    assert np.any(np.diff(deliveries[:, 0]) < 0)


def test_nothing_sent_during_an_outage_arrives():
    link = CommunicationLink(np.random.default_rng(0), outages=((10.0, 15.0),))
    deliveries = send_every_fifth_of_a_second(link)
    sent = deliveries[:, 0]

    assert not np.any((sent >= 10.0) & (sent < 15.0))
    assert link.lost_count == 25


def test_packet_loss_has_the_configured_rate():
    link = CommunicationLink(np.random.default_rng(2), loss_probability=0.25)
    send_every_fifth_of_a_second(link, steps=60000)

    assert 0.22 <= link.lost_count / link.sent_count <= 0.28


def test_loss_leaves_the_delays_of_kept_packets_unchanged():
    jittered = dict(
        send_every_fifth_of_a_second(
            CommunicationLink(np.random.default_rng(3), delay=0.1, jitter=0.4)
        )
    )
    lossy = send_every_fifth_of_a_second(
        CommunicationLink(
            np.random.default_rng(3), delay=0.1, jitter=0.4, loss_probability=0.3
        )
    )

    assert all(np.isclose(jittered[sent], arrived) for sent, arrived in lossy)
