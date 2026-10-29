"""Short-lived, exact path/grid/registration dispatch binding; not a live veto."""
import copy
from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class DispatchCertificate:
    canonical_bindings: str
    wall: float
    sim: float

    @classmethod
    def issue(cls,bindings,*,wall,sim):
        if not all(type(v) in (int,float) and math.isfinite(v) and v>=0 for v in (wall,sim)):
            raise ValueError('Invalid dispatch certificate clocks')
        return cls(json.dumps(copy.deepcopy(bindings),sort_keys=True,allow_nan=False),wall,sim)

    def validate(self,bindings,*,wall,sim):
        if (not all(type(v) in (int,float) and math.isfinite(v) for v in (wall,sim)) or
                not 0<=wall-self.wall<=.5 or not 0<=sim-self.sim<=.5 or
                json.dumps(bindings,sort_keys=True,allow_nan=False)!=self.canonical_bindings):
            raise ValueError('Path/grid/frame/registration changed or dispatch certificate expired')
