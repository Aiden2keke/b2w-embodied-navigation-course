"""Extract the complete, unchanged production method for an offline boundary harness."""
import pathlib
import sys

source = pathlib.Path(sys.argv[1]).read_text()
start = source.index('bool ControllerServer::isGoalReached()')
end = source.index('\nbool ControllerServer::getRobotPose(', start)
pathlib.Path(sys.argv[2]).write_text(source[start:end])
