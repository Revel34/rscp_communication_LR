import argparse as ap
parser = ap.ArgumentParser(
description="Set stage of the rover operation"
)
parser.add_argument("-s","--set", default=0, type=int, help="Stage 1-4")
anon_args = parser.parse_args()    
#------------------------
set_stage=anon_args.set
with open("stage_state.txt","w") as file:
  # file.write("stetet")
  file.write(str(set_stage))