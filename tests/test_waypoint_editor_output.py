from pathlib import Path
import shutil
import uuid

from waypoint_editor.core import Editor


ROOT = Path(__file__).resolve().parents[1]
WAYPOINTS = ROOT / 'experiments/g1_g5/waypoints/20261019-v1'


def test_candidate_export_supports_repository_course_results():
    editor = Editor(
        WAYPOINTS / 'warehouse.yaml',
        ROOT / 'course_navigation/config/nav2.yaml',
        ROOT / 'course_navigation/config/warehouse_route.yaml',
    )
    editor.import_candidate(WAYPOINTS / 'candidate-16nlistv/candidate.yaml')
    output = ROOT / 'course_results' / f'test-waypoint-export-{uuid.uuid4().hex}'
    try:
        exported = editor.export(editor.initial, output)
        assert (exported / 'candidate.yaml').is_file()
        assert (exported / 'manifest.yaml').is_file()
    finally:
        if output.exists():
            shutil.rmtree(output)
