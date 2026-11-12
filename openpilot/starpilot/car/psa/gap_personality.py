"""PSA: the car's ACC time gap sets openpilot's following-distance personality.

The distance button on the cruise stalk is not on CAN. The BSI only broadcasts the time gap it
selected (INTER_VEHICLE_TIME_SETPOINT), so there is no press to hand to the wheel-action owner,
and a press synthesized from the gap never reaches selfdrived either: card drops distance
releases from the published carState. The gap itself is therefore the personality. carstate maps
it to CS.gap_personality; this writes it to LongitudinalPersonality, which selfdrived re-reads
at 10 Hz and the onroad UI announces with its personality notice.

Only a change is written, so a personality picked on screen holds until the driver moves the
stalk or ACC is switched on again.
"""

PARAM = 'LongitudinalPersonality'


def qualified(CP) -> bool:
  return bool(CP.brand == 'psa' and CP.openpilotLongitudinalControl and not CP.passive and not CP.dashcamOnly)


class GapPersonality:
  def __init__(self):
    self.last: int | None = None

  def update(self, CP, car_state, params) -> None:
    """car_state is the car interface's CarState object (CI.CS), not the published struct."""
    personality = getattr(car_state, 'gap_personality', None) if qualified(CP) else None
    if personality not in (0, 1, 2):
      personality = None
    if personality is not None and personality != self.last:
      params.put(PARAM, personality)
    self.last = personality
