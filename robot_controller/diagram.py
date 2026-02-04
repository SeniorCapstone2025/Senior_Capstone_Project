from bare_scan_routine import BareScanRoutine
from statemachine.contrib.diagram import DotGraphMachine

graph = DotGraphMachine(BareScanRoutine)  # also accepts instances

graph.state_font_size = "16"
graph.transition_font_size = "14"

dot = graph()

dot.write_png("bare_scan_routine.png")

