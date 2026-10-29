"""Bounded paired report ingestion; coverage qualification remains unavailable."""
import json
from .stage4_contact_intervals import ContactIntervals


class ContactStream:
    physical_coverage_qualified=False  # ECS names do not prove engine construction.

    def __init__(self,names,*,first_sequence,observation_only=False):
        if first_sequence is None and observation_only is not True:
            raise ValueError('Motion contact stream requires explicit first sequence')
        self.names=tuple(names);self.initial_failure=None
        self.intervals=None if first_sequence is None else ContactIntervals(names,first_sequence=first_sequence)
        from .stage4_contact_evidence import ContactEvidence
        ContactEvidence(names)  # validate pinned inventory even before the first observed interval

    def bind_observation(self,sequence):
        if self.intervals is None:
            try:self.intervals=ContactIntervals(self.names,first_sequence=sequence)
            except (TypeError,ValueError):self.initial_failure='invalid_contact_interval_metadata'
        return self.intervals is not None

    @property
    def failure(self):return self.initial_failure or (self.intervals.failure or self.intervals.gate.failure if self.intervals else None)

    def inventory(self,value,*,wall):
        if self.bind_observation(value.get('sequence')):self.intervals.inventory(value,wall=wall)

    def contact_message(self,msg,*,wall):
        try:
            prefix='b2w_evaluation_seq_'
            tag=msg.header.frame_id
            if not tag.startswith(prefix) or not tag[len(prefix):].isdigit():raise ValueError('missing_contact_sequence')
            if not self.bind_observation(int(tag[len(prefix):])):return
            self.intervals.contacts(int(tag[len(prefix):]),[(c.collision1.name,c.collision2.name) for c in msg.contacts],
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,wall=wall)
        except (AttributeError,TypeError,ValueError) as exc:self.initial_failure=self.initial_failure or str(exc)

    def ready(self,*,sim,wall):
        return self.intervals is not None and not self.failure and self.intervals.gate.complete(sim=sim,wall=wall)
