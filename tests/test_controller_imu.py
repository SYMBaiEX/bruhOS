import pytest
from bruhos.body.controller_imu import ControllerIMU

class Board:
    def __init__(self, sample): self.sample=sample; self.enabled=False
    def enable_reception(self): self.enabled=True
    def get_imu(self): return self.sample

@pytest.mark.parametrize('accel,fallen,direction', [
    ((9.8,0,0),False,None), ((0,0,9.8),True,True),
    ((0,0,-9.8),True,False), ((0,9.8,0),True,None)])
def test_board_orientation(accel,fallen,direction):
    board=Board((*accel,0,0,0))
    result,raw=ControllerIMU(board).read()
    assert board.enabled
    assert result.fallen is fallen
    assert result.fell_forward is direction
    assert len(raw)==6

@pytest.mark.parametrize('sample', [(0,0,0,0,0,0), (float('nan'),1,2,3,4,5), (1,2)])
def test_invalid_samples_not_silently_upright(sample):
    with pytest.raises(ValueError): ControllerIMU(Board(sample)).read()

def test_no_new_sample_is_explicit():
    assert ControllerIMU(Board(None)).read() is None


def test_inverted_upright_axis_is_configurable():
    value,_=ControllerIMU(Board((-1,0,.1,0,0,0)),upright_sign=-1).read()
    assert not value.fallen
    assert 5 < value.pitch_deg < 6


def test_upright_zero_offsets_change_reference_not_raw_measurements():
    sample=(-1,0,.1,0,0,0)
    baseline,_=ControllerIMU(Board(sample),upright_sign=-1).read()
    corrected,raw=ControllerIMU(Board(sample),upright_sign=-1,pitch_zero_deg=baseline.pitch_deg).read()
    assert abs(corrected.pitch_deg)<.02
    assert raw==list(sample)
    fallen,_=ControllerIMU(Board((0,0,1,0,0,0)),upright_sign=-1,pitch_zero_deg=baseline.pitch_deg).read()
    assert fallen.fallen

@pytest.mark.parametrize('offset',[float('nan'),float('inf'),21,-21])
def test_unsafe_zero_offsets_are_rejected(offset):
    with pytest.raises(ValueError): ControllerIMU(Board(None),pitch_zero_deg=offset)
