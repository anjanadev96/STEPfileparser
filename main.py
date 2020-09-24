
import os
import json

from OCC.Extend.TopologyUtils import TopologyExplorer, discretize_edge, discretize_wire, get_type_as_string
from OCC.Extend.ShapeFactory import get_oriented_boundingbox, get_aligned_boundingbox, measure_shape_mass_center_of_gravity
from OCC.Extend.DataExchange import read_step_file, export_shape_to_svg
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve, BRepAdaptor_Curve2d
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBoundsProperties
from OCC.Core.GeomAbs import *
from OCC.Core.TColgp import *
from OCC.Core.TColStd import *
from OCC.Core.TopoDS import TopoDS_Face, topods
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import (brepgprop_LinearProperties,
                                brepgprop_SurfaceProperties,
                                brepgprop_VolumeProperties)

from OCC.Core.gp import gp_Pnt2d
from abstract import *

def main():
    shp = read_step_file('Models/00000000_290a9120f9f249a7a05cfe9c_step_000.step')
    shape_type = get_type_as_string(shp)
    t =  TopologyFactory(shape_type) 
    shape = t.create_shape_object(shp)  
    shape.parse_shape()
    with open("model2.json", "w") as f:
        json.dump(shape.config, f, indent=4)



if __name__ == '__main__':
    main()