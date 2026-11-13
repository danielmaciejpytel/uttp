"""Tests of the key-pose schema + apply_keyposes.py (python -I test_apply_keyposes.py); needs the dumps (melee_dump.py all)."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # -I ignores the current directory
import apply_keyposes as AK  # noqa: E402


def test_schema_file_is_valid_json_and_example_validates():
    sch = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'keypose_schema.json'), encoding='utf-8'))
    assert sch['$id'] == 'utpp-keypose-correction/1'
    ex = sch['examples'][0]
    assert AK.validate(ex) == [], AK.validate(ex)


def test_validation_catches_bad_files():
    base = dict(schema='utpp-keypose-correction/1', set='pwa_katana', clip='x')
    assert AK.validate(dict(base, keys=[dict(id='a', src_frame=5, time_frame=9), dict(id='b', src_frame=8, time_frame=9)]))
    assert AK.validate(dict(base, keys=[dict(id='a', src_frame=5, rot=[dict(bone='LeftArm', space='world', euler_deg=[0, 0, 0])])]))
    assert AK.validate(dict(base, keys=[dict(id='a', src_frame=5, keep=['Head'])]))


if __name__ == '__main__':
    test_schema_file_is_valid_json_and_example_validates()
    test_validation_catches_bad_files()
    AK.selftest()
    print('test_apply_keyposes OK')
