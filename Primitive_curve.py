import os
import json

from OCC.Extend.TopologyUtils import TopologyExplorer, discretize_edge, discretize_wire, get_type_as_string
from OCC.Extend.ShapeFactory import get_oriented_boundingbox, get_aligned_boundingbox, \
    measure_shape_mass_center_of_gravity
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
from abc import ABC, abstractmethod
from OCC.Core.gp import gp_Pnt2d
from OCC.Core.BRepTools import breptools_Dump, breptools_Write


class Curve:
    def __init__(self, face, surf):
        self.face = face
        self.surf = surf

    @abstractmethod
    def extract_curve_data(self):
        pass


class CurveFactory:

    def create_curve_object(self, curve_adapter, face, surf, c_id):

        curve_type = curve_adapter.GetType()

        if curve_type == GeomAbs_BSplineCurve:
            surf.trimmed = True
            bspline_curve = curve_adapter.BSpline()
            return BSplineCurve(bspline_curve, face, surf, c_id)


        elif curve_type == GeomAbs_Line:
            surf.trimmed = True
            line_curve = curve_adapter.Line()
            return Line(line_curve, face, surf, c_id)


        elif curve_type == GeomAbs_Circle:
            surf.trimmed = True
            circle_curve = curve_adapter.Circle()
            return Circle(circle_curve,face,surf,c_id)

        elif curve_type == GeomAbs_Ellipse:
            surf.trimmed = True
            circle_curve = curve_adapter.Ellipse()
            return Ellipse(circle_curve,face,surf,c_id)


class BSplineCurve(Curve):
    def __init__(self, bspline_curve, face, surf, c_id):
        super(BSplineCurve, self).__init__(face, surf)
        self.bspline_curve = bspline_curve
        self.curve_info = {}
        self.curve_type = "spline"
        self.face = face
        self.order = 0
        self.n_points = 0
        self.ctrl_points = []
        self.weights = []
        self.knotvector = []
        self.c_id = c_id


    def extract_curve_data(self, f_id):

        self.degree=self.bspline_curve.Degree()
        self.rational = self.bspline_curve.IsRational()
        self.continuity=self.bspline_curve.Continuity()
        self.order = self.bspline_curve.Degree() + 1
        self.n_points = self.bspline_curve.NbPoles()
        self.periodic = self.bspline_curve.IsPeriodic()

        if self.periodic:
            self.bspline_curve.SetNotPeriodic()

        p = TColgp_Array1OfPnt2d(1, self.bspline_curve.NbPoles())
        self.bspline_curve.Poles(p)

        for pi in range(p.Length()):
            self.ctrl_points.append(list(p.Value(pi + 1).Coord()))

        k = TColStd_Array1OfReal(1, self.bspline_curve.NbPoles() + self.bspline_curve.Degree() + 1)
        self.bspline_curve.KnotSequence(k)

        for ki in range(k.Length()):
            self.knotvector.append(k.Value(ki + 1))

        w = TColStd_Array1OfReal(1, self.bspline_curve.NbPoles())
        self.bspline_curve.Weights(w)
        for wi in range(w.Length()):
            self.weights.append(w.Value(wi + 1))

        self.curve_info['type'] = self.curve_type
        self.curve_info['curve id'] = self.c_id
        self.curve_info['rational']=self.rational
        self.curve_info['degree']=self.degree
        self.curve_info['knotvector']=self.knotvector
        self.curve_info['control_points']={}
        self.curve_info['control_points']['points']=self.ctrl_points
        self.curve_info['control_points']['weights']=self.weights
        self.curve_info['reversed']=0

        return self.curve_info


class Line(Curve):
    def __init__(self, line_curve, face, surf, c_id):
        super(Line, self).__init__(face, surf)
        self.line_curve = line_curve
        self.curve_info = {}
        self.c_id = c_id
        self.face = face
        self.surf = surf
        self.location=[]
        self.direction=[]

    def extract_curve_data(self, f_id):

        self.location=list(self.line_curve.Location().Coord())
        self.direction = list(self.line_curve.Direction().Coord())

        self.curve_info['type'] = 'line'
        self.curve_info['curve id'] = self.c_id
        self.curve_info['data'] = {}
        self.curve_info['data']['location'] = self.location
        self.curve_info['data']['direction'] = self.direction


        return self.curve_info


class Circle(Curve):
    def __init__(self, circle_curve, face, surf, c_id):
        super(Circle, self).__init__(face, surf)
        self.circle_curve = circle_curve
        self.curve_info = {}
        self.c_id = c_id
        self.face = face
        self.surf = surf
        self.location = []
        self.radius=0
        self.x_axis=[]
        self.y_axis=[]

    def extract_curve_data(self, f_id):
        self.location = list(self.circle_curve.Location().Coord())
        self.radius=self.circle_curve.Radius()
        self.x_axis=list(self.circle_curve.XAxis().Direction().Coord())
        self.y_axis = list(self.circle_curve.YAxis().Direction().Coord())

        self.curve_info['type'] = 'circle'
        self.curve_info['curve id'] = self.c_id
        self.curve_info['data'] = {}
        self.curve_info['data']['location'] = self.location
        self.curve_info['data']['radius'] = self.radius
        self.curve_info['data']['x_axis']= self.x_axis
        self.curve_info['data']['y_axis']=self.y_axis

        return self.curve_info


class Ellipse(Curve):
    def __init__(self, ellipse_curve, face, surf, c_id):
        super(Ellipse, self).__init__(face, surf)
        self.ellipse_curve = ellipse_curve
        self.curve_info = {}
        self.c_id = c_id
        self.face = face
        self.surf = surf
        self.focus1=[]
        self.focus2 =[]
        self.x_axis=[]
        self.y_axis=[]

    def extract_curve_data(self, f_id):
        self.location = list(self.ellipse_curve.Location().Coord())
        self.focus1=list(self.ellipse_curve.Focus1().Coord())
        self.focus2=list(self.ellipse_curve.Focus2().Coord())
        self.x_axis=list(self.ellipse_curve.XAxis().Direction().Coord())
        self.y_axis = list(self.ellipse_curve.YAxis().Direction().Coord())
        self.major_radius=self.ellipse_curve.MajorRadius()
        self.minor_radius=self.ellipse_curve.MinorRadius()

        self.curve_info['type'] = 'ellipse'
        self.curve_info['curve id'] = self.c_id
        self.curve_info['data'] = {}
        self.curve_info['data']['location'] = self.location
        self.curve_info['data']['focus1'] = self.focus1
        self.curve_info['data']['focus2'] = self.focus2
        self.curve_info['data']['x_axis']= self.x_axis
        self.curve_info['data']['y_axis']=self.y_axis
        self.curve_info['data']['major_radius'] = self.major_radius
        self.curve_info['data']['minor_radius'] = self.minor_radius

        return self.curve_info





