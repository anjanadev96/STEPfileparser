import os
import json
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Extend.TopologyUtils import TopologyExplorer, discretize_edge, discretize_wire, get_type_as_string
from OCC.Extend.ShapeFactory import get_oriented_boundingbox, get_aligned_boundingbox, measure_shape_mass_center_of_gravity
from OCC.Extend.DataExchange import read_step_file, export_shape_to_svg
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve, BRepAdaptor_Curve2d
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBoundsProperties
from OCC.Core.GeomAbs import *
from OCC.Core.TColgp import *
from OCC.Core.TColStd import *
from OCC.Core.BRepTools import *
from OCC.Core.BRep import *
from OCC.Core.TopoDS import TopoDS_Face, topods
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopoDS import TopoDS_Compound, topods_Face
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import (brepgprop_LinearProperties,
                                brepgprop_SurfaceProperties,
                                brepgprop_VolumeProperties)

from OCC.Core.gp import gp_Pnt2d

TOLERANCE = 1e-6
from abc import ABC,abstractmethod
from surface import Surface, SurfaceFactory


class Topology(ABC):


    @abstractmethod
    def parse_shape(self,shape):
        raise NotImplementedError


    @classmethod
    def get_shape(self,shape):
        return get_type_as_string(shape)



class TopologyFactory:
    def __init__(self,shape_type):
        self.shape_type = shape_type


    def create_shape_object(self,shape):
        if self.shape_type == "Compound":
            return Compound(shape, self.shape_type)

        if self.shape_type == "Solid":
            return Solid(shape, self.shape_type, 1)



class Curve():
    """docstring for Curve"""
    # def __init__(self, curve):
    #     super(Curve, self).__init__()
    #     self.arg = arg
    pass
        




        

class Solid(Topology):
    """docstring for Solid"""

    def __init__(self, shape,shape_type, s_id):
        super(Solid, self).__init__()
        self.shape = shape
        self.s_id = s_id
        self.shape_type = super(Solid, self).get_shape(self.shape)
        self.triangles = {}
        self.config = {}

    def parse_shape(self):
        assert self.shape_type is "Solid"
        self.config['shape'] = self.shape_type
        self.config['solid id'] = self.s_id
        self.config['bounding_box']=self.get_bounding_box()
        verts, triangles = self.triangulate_solid()
        if verts!=None and triangles!=None:
            self.triangles['vertices']= verts
            self.triangles['triangles'] = triangles
        self.config['triangles'] = self.triangles
       
        self.config['data'] = []
        f_id = 1
        t = TopologyExplorer(self.shape)
        for subshape in t.faces():
            surface_factory = SurfaceFactory()
            surface = surface_factory.create_surface_object(subshape, f_id)
            if surface is not None:
                surface.extract_data()
            
            
                
            # print(surface)
                self.config['data'].append(surface.config)
            f_id = f_id + 1
            
            
            
    def triangulate_solid(self):
        
        mesh = BRepMesh_IncrementalMesh(self.shape, 0.3, False, 0.5, True)
        mesh.Perform()
        
        b = BRep_Tool()
        ex = TopExp_Explorer(self.shape, TopAbs_FACE)
        faces = []
        verts = []
        while ex.More():
            face = topods_Face(ex.Current())
            location = TopLoc_Location()
            triang_face = (b.Triangulation(face,location))
            
            if triang_face != None:
                tab = triang_face.Nodes()
                tri = triang_face.Triangles()
                

                for i in range(1, triang_face.NbTriangles() + 1):
                    try:
                        verts.append(list(tab.Value(i).Coord()))
                    except:
                        continue


                
                for i in range(1, triang_face.NbTriangles()+1):
                    try:
                        index1, index2, index3 = tri.Value(i).Get()
                        faces.append([index1-1, index2 - 1, index3-1])
                    except:
                        continue
        
            ex.Next()
       
        
                


            
        return verts, faces
            
        



    
    def get_bounding_box(self,tol=TOLERANCE):
        bbox = Bnd_Box()
        brepbndlib_Add(self.shape, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        return ["%.2f"%xmin, "%.2f"%ymin, "%.2f"%zmin, "%.2f"%xmax, "%.2f"%ymax, "%.2f"%zmax, "%.2f"%(xmax-xmin), "%.2f"%(ymax-ymin), "%.2f"%(zmax-zmin)]


    def volume():
        pass



class Compound(Topology):
    """docstring for Compound"""
    def __init__(self, shape, shape_type):
        super(Compound, self).__init__()
        self.shape = shape
        self.shape_type = shape_type
        self.config = {}
    

    def parse_shape(self):
        assert self.shape_type is "Compound"
        self.config['shape'] = self.shape_type
        self.config['data'] = []
        s_id = 1
        t = TopologyExplorer(self.shape)
        for subshape in t.solids():
            print(s_id)
            solid = Solid(subshape, "Solid", s_id)
            solid.parse_shape()
            self.config['data'].append(solid.config)
            s_id = s_id + 1

        
