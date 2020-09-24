import os
import sys
import json

from OCC.Extend.TopologyUtils import get_type_as_string
from OCC.Extend.DataExchange import read_step_file
from abstract_cp import *

def main():
    input_folder = "{:08d}".format(int(sys.argv[1]))
    print(int(sys.argv[1]))
    filename = [file for file in os.listdir(os.path.join('/adarsh-lab/Aditya/ABCDataset',input_folder)) if os.path.splitext(file)[1] in ['.step', '.stp']][0]
    filename = os.path.join('/adarsh-lab/Aditya/ABCDataset',input_folder,filename)
    try:
        shp = read_step_file(filename)
        shape_type = get_type_as_string(shp)
        t =  TopologyFactory(shape_type) 
        shape = t.create_shape_object(shp)  
        shape.parse_shape()
        # seq=(os.path.splitext(filename)[0],"primitive",".json")

        out_filename = input_folder+'.json'
        out_filename = os.path.join('/adarsh-lab/Anjana/Points',out_filename)
        with open(out_filename, "w") as f:
            json.dump(shape.config, f, indent=4)

    except KeyError:
        print("Some Invalid Shape")
    
        
    

0
0

if __name__ == '__main__':
        main()
