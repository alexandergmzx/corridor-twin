# Integration tests

This directory will hold deterministic end-to-end tests that feed synthetic
camera frames to the observer and score its output against truth available only
to the test harness. The observer itself must never subscribe to the truth
topic.
