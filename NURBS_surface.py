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
from NURBS_curve import Curve, CurveFactory, BSplineCurve, Line, Circle, Ellipse
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
            #         nurbs_surface.trim_curves.append(loop)
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


        if surf_type == GeomAbs_BSplineSurface:
            return BSplineSurface(face, surf, f_id)
        else:
            return None

class BSplineSurface(Surface):
    def __init__(self, face, surf, f_id):
        super(BSplineSurface, self).__init__(face, surf)
        self.config = {}
        self.f_id = f_id
        self.face = face
        self.surf = surf
        self.ctrl_points = []
        self.knotvector_u = []
        self.knotvector_v = []
        self.order_u = 0
        self.order_v = 0
        self.u_points = 0
        self.v_points = 0
        self.trimmed = False
        self.n_points = 0
        self.trim_curves = []
        # self.triangles={}

    def __repr__(self):

        return str(self.config)

    def extract_data(self):

        self.config['shape'] = {}
        self.config['shape']['type']='surface'
        self.config['shape']['bounding_box'] = super(BSplineSurface, self).get_bounding_box(self.face)
        self.config['shape']['face_id'] = self.f_id

        bspline_surface = self.surf.BSpline()
        u_periodic = bspline_surface.IsUPeriodic()
        v_periodic = bspline_surface.IsVPeriodic()

        if u_periodic:
            bspline_surface.SetUNotPeriodic()

        if v_periodic:
            bspline_surface.SetVNotPeriodic()

        self.u_rational = bspline_surface.IsURational()
        self.v_rational = bspline_surface.IsVRational()
        self.u_closed = bspline_surface.IsUClosed()
        self.v_closed = bspline_surface.IsVClosed()
        self.degree_u = bspline_surface.UDegree()
        self.degree_v = bspline_surface.VDegree()
        self.order_u = self.degree_u + 1
        self.order_v = self.degree_v + 1
        self.size_u = bspline_surface.NbUPoles()
        self.size_v = bspline_surface.NbVPoles()
        self.trimmed = False

        p = TColgp_Array2OfPnt(1, bspline_surface.NbUPoles(), 1, bspline_surface.NbVPoles())
        bspline_surface.Poles(p)

        self.ctrl_points = []
        self.weights = []

        for pi in range(p.ColLength()):
            elems = []
            for pj in range(p.RowLength()):
                elems.append(list(p.Value(pi + 1, pj + 1).Coord()))
            self.ctrl_points.append(elems)

        k = TColStd_Array1OfReal(1, bspline_surface.NbUPoles() + bspline_surface.UDegree() + 1)
        bspline_surface.UKnotSequence(k)
        for ki in range(k.Length()):
            self.knotvector_u.append(k.Value(ki + 1))

        k = TColStd_Array1OfReal(1, bspline_surface.NbVPoles() + bspline_surface.VDegree() + 1)
        bspline_surface.VKnotSequence(k)
        for ki in range(k.Length()):
            self.knotvector_v.append(k.Value(ki + 1))

        w = TColStd_Array2OfReal(1, bspline_surface.NbUPoles(), 1, bspline_surface.NbVPoles())
        bspline_surface.Weights(w)
        for wi in range(w.ColLength()):
            elems = []
            for wj in range(w.RowLength()):
                elems.append(w.Value(wi + 1, wj + 1))
            self.weights.append(elems)




        face = {}
        face['kind'] = "Bspline Surface"
        face['rational'] = self.u_rational and self.v_rational
        face['degree_u'] = self.degree_u
        face['degree_v'] = self.degree_v
        face["knotvector_u"] = self.knotvector_u
        face['knotvector_v'] = self.knotvector_v
        face["size_u"]=self.size_u
        face["size_v"]=self.size_v

        control_points = {}
        control_points['points'] = self.ctrl_points
        control_points['weights'] = self.weights
        control_points['trims']=super(BSplineSurface, self).extract_trims_curves()

        face['control_points'] = control_points
        self.config['shape']['data'] = face



