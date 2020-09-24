import os
import json
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Extend.TopologyUtils import TopologyExplorer, discretize_edge, discretize_wire, get_type_as_string
from OCC.Extend.ShapeFactory import get_oriented_boundingbox, get_aligned_boundingbox, \
    measure_shape_mass_center_of_gravity
from OCC.Extend.DataExchange import read_step_file, export_shape_to_svg
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve, BRepAdaptor_Curve2d
from OCC.Core.ShapeAnalysis import ShapeAnalysis_FreeBoundsProperties
from OCC.Core.GeomAbs import *
from OCC.Core.TColgp import *
from OCC.Core.BRepTools import *
from OCC.Core.BRep import *
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

from abc import ABC, abstractmethod
from OCC.Core.gp import gp_Pnt2d
from Primitive_curve import Curve, CurveFactory, BSplineCurve, Line, Circle, Ellipse
from OCC.Core.TopLoc import TopLoc_Location

TOLERANCE = 1e-6


class Surface():
    """docstring for Surface"""

    def __init__(self, face, surf):
        self.face = face
        self.surf = surf

    def get_bounding_box(self, face, tol=TOLERANCE):
        bbox = Bnd_Box()
        brepbndlib_Add(face, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        return ["%.2f" % xmin, "%.2f" % ymin, "%.2f" % zmin, "%.2f" % xmax, "%.2f" % ymax, "%.2f" % zmax,
                "%.2f" % (xmax - xmin), "%.2f" % (ymax - ymin), "%.2f" % (zmax - zmin)]

    @abstractmethod
    def extract_data(self):
        pass


    def extract_trims_curves(self):
        # Read in the Trim Curves
        trim_curves = []
        trims = ShapeAnalysis_FreeBoundsProperties(self.face)
        trims.Perform()
        num_loops = trims.NbClosedFreeBounds()

        num_open_loops = trims.NbOpenFreeBounds()
        if num_open_loops > 0:
            print('Warning: Face has open boundaries')

        for n_boundary in range(num_loops):
            boundary_data = trims.ClosedFreeBound(n_boundary + 1)
            boundary_wire = boundary_data.FreeBound()
            loop = []
            c_id = 1
            top_ex = TopExp_Explorer(boundary_wire, TopAbs_EDGE)
            while (top_ex.More()):
                edge = topods.Edge(top_ex.Current())
                curve_adapter = BRepAdaptor_Curve2d(edge, self.face)
                curve_factory = CurveFactory()
                if curve_factory is not None:
                    c = curve_factory.create_curve_object(curve_adapter, self.face, self.surf, c_id)
                    curve = c.extract_curve_data(self.f_id)
                    c_id = c_id + 1
                    if curve is None:
                        continue



                loop.append(curve)
                top_ex.Next()

            trim_curves.append(loop)

        trim_dict = {}
        trim_dict['count'] = len(trim_curves)
        trim_dict['data'] = []
        for loop in trim_curves:
            trim_dict['data'].append(loop)
        return trim_dict


class SurfaceFactory:
    def create_surface_object(self, face, f_id):
        surf = BRepAdaptor_Surface(face, True)
        surf_type = surf.GetType()

        if surf_type == GeomAbs_Plane:
            return Plane(face, surf, f_id)
        elif surf_type == GeomAbs_Cylinder:
            return Cylinder(face, surf, f_id)
        elif surf_type == GeomAbs_Cone:
            return Cone(face, surf, f_id)
        elif surf_type == GeomAbs_Sphere:
            return Sphere(face, surf, f_id)
        elif surf_type == GeomAbs_Torus:
            return Torus(face, surf, f_id)






class Plane(Surface):
    def __init__(self, face, surf, f_id):
        super(Plane, self).__init__(face, surf)
        self.face = face
        self.surf = surf
        self.config = {}
        self.f_id = f_id
        self.trimmed = False
        self.trim_curves = []


    def __repr__(self):
        return str(self.config)

    def extract_data(self):
        self.config['shape'] = {}
        self.config['shape']['type'] = "Surface"
        self.config['shape']['bounding_box'] = super(Plane, self).get_bounding_box(self.face)
        self.config['shape']['face_id'] = self.f_id
        self.config['shape']['data'] = {}

        plane_surface = self.surf.Plane()



        face = {}
        face['kind'] = "Plane"
        face["location"] = list(plane_surface.Location().Coord())
        face["x_axis"] = list(plane_surface.XAxis().Direction().Coord())
        face["y_axis"] = list(plane_surface.YAxis().Direction().Coord())
        face["z_axis"] = list(plane_surface.Axis().Direction().Coord())
        face["coefficients"] = list(plane_surface.Coefficients())
        face['trims'] = super(Plane, self).extract_trims_curves()
        self.config['data']=face


class Cylinder(Surface):
    def __init__(self, face, surf, f_id):
        super(Cylinder, self).__init__(face, surf)
        self.face = face
        self.surf = surf
        self.config = {}
        self.trimmed = False
        self.trim_curves = []
        self.f_id = f_id


    def __repr__(self):
        return str(self.config)

    def extract_data(self):
        self.config['shape'] = {}
        self.config['shape']['type'] = "Surface"
        self.config['shape']['bounding_box'] = super(Cylinder, self).get_bounding_box(self.face)
        self.config['shape']['face_id'] = self.f_id
        self.config['shape']['data'] = {}
        cylinder_surface = self.surf.Cylinder()




        cyl_face = {}
        cyl_face['kind']= "Cylinder"
        cyl_face["location"] = list(cylinder_surface.Location().Coord())
        cyl_face["z_axis"] = list(cylinder_surface.Axis().Direction().Coord())
        cyl_face["x_axis"] = list(cylinder_surface.XAxis().Direction().Coord())
        cyl_face["y_axis"] = list(cylinder_surface.YAxis().Direction().Coord())
        cyl_face["coefficients"] = list(cylinder_surface.Coefficients())
        cyl_face["radius"] = cylinder_surface.Radius()
        cyl_face['trims'] = super(Cylinder, self).extract_trims_curves()


        self.config['shape']['data'] = cyl_face


class Cone(Surface):
    def __init__(self, face, surf, f_id):
        super(Cone, self).__init__(face, surf)
        self.face = face
        self.surf = surf
        self.config = {}
        self.f_id = f_id
        self.trimmed = False
        self.trim_curves = []


    def __repr__(self):
        return str(self.config)

    def extract_data(self):
        self.config['shape'] = {}
        self.config['shape']['type'] = "Surface"
        self.config['shape']['bounding_box'] = super(Cone, self).get_bounding_box(self.face)
        self.config['shape']['face_id'] = self.f_id
        self.config['shape']['data'] = {}
        conical_surface = self.surf.Cone()




        face = {}
        face['kind']= "Cone"
        face["location"] = list(conical_surface.Location().Coord())
        face["z_axis"] = list(conical_surface.Axis().Direction().Coord())
        face["x_axis"] = list(conical_surface.XAxis().Direction().Coord())
        face["y_axis"] = list(conical_surface.YAxis().Direction().Coord())
        face["coefficients"] = list(conical_surface.Coefficients())
        face["radius"] = conical_surface.RefRadius()
        face["angle"] = conical_surface.SemiAngle()
        face["apex"] = list(conical_surface.Apex().Coord())
        face['trims'] = super(Cone, self).extract_trims_curves()



        self.config['shape']['data']= face


class Sphere(Surface):
    def __init__(self, face, surf, f_id):
        super(Sphere, self).__init__(face, surf)
        self.face = face
        self.surf = surf
        self.config = {}
        self.trimmed = False
        self.trim_curves = []
        self.f_id = f_id


    def __repr__(self):
        return str(self.config)

    def extract_data(self):
        self.config['shape'] = {}
        self.config['shape']['type'] = "Surface"
        self.config['shape']['bounding_box'] = super(Sphere, self).get_bounding_box(self.face)
        self.config['shape']['f_id'] = self.f_id
        self.config['shape']['data'] = {}
        sphere_surface = self.surf.Sphere()




        face = {}
        face['kind'] = "Sphere"
        face["location"] = list(sphere_surface.Location().Coord())
        face["x_axis"] = list(sphere_surface.XAxis().Direction().Coord())
        face["y_axis"] = list(sphere_surface.YAxis().Direction().Coord())
        face["coefficients"] = list(sphere_surface.Coefficients())
        face["radius"] = sphere_surface.Radius()
        face['trims'] = super(Sphere, self).extract_trims_curves()
        # face['triangles'] = self.triangles


        self.config['shape']['data'] = face


class Torus(Surface):
    def __init__(self, face, surf, f_id):
        super(Torus, self).__init__(face, surf)
        self.face = face
        self.surf = surf
        self.config = {}
        self.trimmed = False
        self.trim_curves = []
        self.f_id = f_id


    def __repr__(self):
        return str(self.config)

    def extract_data(self):
        self.config['shape'] = {}
        self.config['shape']['type'] = "Surface"
        self.config['shape']['bounding_box'] = super(Torus, self).get_bounding_box(self.face)
        self.config['shape']['face_id'] = self.f_id
        self.config['shape']['data'] = {}
        torus_surface = self.surf.Torus()




        face = {}
        face["location"] = list(torus_surface.Location().Coord())
        face["z_axis"] = list(torus_surface.Axis().Direction().Coord())
        face["x_axis"] = list(torus_surface.XAxis().Direction().Coord())
        face["y_axis"] = list(torus_surface.YAxis().Direction().Coord())
        face["max_radius"] = torus_surface.MajorRadius()
        face["min_radius"] = torus_surface.MinorRadius()
        face['trims'] = super(Torus, self).extract_trims_curves()



        self.config['shape']['data'] = face


