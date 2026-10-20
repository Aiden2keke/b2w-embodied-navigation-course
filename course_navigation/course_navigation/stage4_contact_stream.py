"""Bounded paired report ingestion; coverage qualification remains unavailable."""
import json
from .stage4_contact_intervals import ContactIntervals


class ContactStream:
    physical_coverage_qualified=False  # ECS names do not prove engine construction.

    def __init__(self,names,*,first_sequence):
        self.intervals=ContactIntervals(names,first_sequence=first_sequence)

    @property
    def failure(self):return self.intervals.failure or self.intervals.gate.failure

    def inventory(self,value,*,wall):self.intervals.inventory(value,wall=wall)

    def contact_message(self,msg,*,wall):
        try:
            prefix='b2w_evaluation_seq_'
            tag=msg.header.frame_id
            if not tag.startswith(prefix) or not tag[len(prefix):].isdigit():raise ValueError('missing_contact_sequence')
            self.intervals.contacts(int(tag[len(prefix):]),[(c.collision1.name,c.collision2.name) for c in msg.contacts],
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,wall=wall)
        except (AttributeError,TypeError,ValueError) as exc:self.intervals.fail(str(exc))

    def ready(self,*,sim,wall):
        return not self.failure and self.intervals.gate.complete(sim=sim,wall=wall)
