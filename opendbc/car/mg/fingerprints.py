from opendbc.car.structs import CarParams
from opendbc.car.mg.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.MG_5_EV: {
    (Ecu.eps, 0x721, None): [
      b'\x10gs\x16\x01',
    ],
    (Ecu.fwdCamera, 0x733, None): [
      b'\x10y\x00 \x01',
    ],
    (Ecu.fwdRadar, 0x734, None): [
      b'\x10y\x000\x01',
    ],
  },
  CAR.MG_ZS_EV: {
    (Ecu.eps, 0x721, None): [
      b'\x11\x06c\x94\x01',
    ],
    (Ecu.fwdCamera, 0x733, None): [
      b'\x11\x03\t!\x01',
    ],
    (Ecu.fwdRadar, 0x734, None): [
      b'\x11\x03\t\x18\x01',
    ],
  },
  # NOTE: MG_ZS has no FW fingerprint yet. An EMPTY FW_VERSIONS entry matches
  # every FW query in match_fw_to_car_exact (its ecu loop never runs, so it is
  # never invalidated), acting as a phantom 2nd candidate that breaks FW
  # fingerprinting fork-wide (e.g. a real Tesla EPS match becomes ambiguous ->
  # MOCK). Keep MG_ZS out of FW_VERSIONS until real FW is captured via
  # tools/car_porting/auto_fingerprint.py.
  # CAR.MG_ZS: {
  # },
}
