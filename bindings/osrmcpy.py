#!/usr/bin/env python2

from __future__ import print_function, division

import ctypes as c
from collections import namedtuple
from contextlib import contextmanager

# Native Functions Bridge

lib = c.cdll.LoadLibrary('libosrmc.so')

# Error handling
lib.osrmc_error_message.restype = c.c_char_p
lib.osrmc_error_message.argtypes = [c.c_void_p]

lib.osrmc_error_destruct.restype = None
lib.osrmc_error_destruct.argtypes = [c.c_void_p]

class osrmc_error(c.c_void_p):
    def __str__(self):
        return lib.osrmc_error_message(self).decode('latin1')

    def __del__(self):
        lib.osrmc_error_destruct(self)

def osrmc_error_errcheck(result, func, arguments):
    if len(arguments) > 0 and type(arguments[-1]._obj) == osrmc_error and arguments[-1]._obj.value:
        raise RuntimeError(arguments[-1]._obj)
    return result

# Config
lib.osrmc_config_construct.restype = c.c_void_p
lib.osrmc_config_construct.argtypes = [c.c_char_p, c.c_void_p]
lib.osrmc_config_construct.errcheck = osrmc_error_errcheck

lib.osrmc_config_destruct.restype = None
lib.osrmc_config_destruct.argtypes = [c.c_void_p]

# ORM
lib.osrmc_osrm_construct.restype = c.c_void_p
lib.osrmc_osrm_construct.argtypes = [c.c_void_p, c.c_void_p]
lib.osrmc_osrm_construct.errcheck = osrmc_error_errcheck

lib.osrmc_osrm_destruct.restype = None
lib.osrmc_osrm_destruct.argtypes = [c.c_void_p]

# Generic Param Handling
lib.osrmc_params_add_coordinate.restype = None
lib.osrmc_params_add_coordinate.argtypes = [c.c_void_p, c.c_float, c.c_float, c.c_void_p]
lib.osrmc_params_add_coordinate.errcheck = osrmc_error_errcheck

# Route Params
lib.osrmc_route_params_construct.restype = c.c_void_p
lib.osrmc_route_params_construct.argtypes = [c.c_void_p]
lib.osrmc_route_params_construct.errcheck = osrmc_error_errcheck

lib.osrmc_route_params_destruct.restype = None
lib.osrmc_route_params_destruct.argtypes = [c.c_void_p]

# Route
lib.osrmc_route.restype = c.c_void_p
lib.osrmc_route.argtypes = [c.c_void_p, c.py_object, c.c_void_p]
lib.osrmc_route.errcheck = osrmc_error_errcheck

lib.osrmc_route_response_destruct.restype = None
lib.osrmc_route_response_destruct.argtypes = [c.c_void_p]

lib.osrmc_route_response_distance.restype = c.c_float
lib.osrmc_route_response_distance.argtypes = [c.c_void_p, c.c_void_p]
lib.osrmc_route_response_distance.errcheck = osrmc_error_errcheck

lib.osrmc_route_response_duration.restype = c.c_float
lib.osrmc_route_response_duration.argtypes = [c.c_void_p, c.c_void_p]
lib.osrmc_route_response_duration.errcheck = osrmc_error_errcheck

# Table Params
lib.osrmc_table_params_construct.restype = c.c_void_p
lib.osrmc_table_params_construct.argtypes = [c.c_void_p]
lib.osrmc_table_params_construct.errcheck = osrmc_error_errcheck

lib.osrmc_table_params_destruct.restype = None
lib.osrmc_table_params_destruct.argtypes = [c.c_void_p]

# Table

lib.osrmc_table.restype = c.c_void_p
lib.osrmc_table.argtypes = [c.c_void_p, c.c_void_p, c.c_void_p]
lib.osrmc_table.errcheck = osrmc_error_errcheck

lib.osrmc_table_response_destruct.restype = None
lib.osrmc_table_response_destruct.argtypes = [c.c_void_p]

lib.osrmc_table_response_duration.restype = c.c_float
lib.osrmc_table_response_duration.argtypes = [c.c_void_p, c.c_ulong, c.c_ulong, c.c_void_p]
lib.osrmc_table_response_duration.errcheck = osrmc_error_errcheck

# JSON
lib.osrmc_json_to_pyobj.restype = c.py_object
lib.osrmc_json_to_pyobj.argtypes = [c.c_void_p]

# Python Library Interface

@contextmanager
def scoped_config(base_path):
    config = lib.osrmc_config_construct(base_path.encode('utf-8'), c.byref(osrmc_error()))
    yield config
    lib.osrmc_config_destruct(config)

@contextmanager
def scoped_osrm(config):
    osrm = lib.osrmc_osrm_construct(config, c.byref(osrmc_error()))
    yield osrm
    lib.osrmc_osrm_destruct(osrm)

@contextmanager
def scoped_route_params():
    params = lib.osrmc_route_params_construct(c.byref(osrmc_error()))
    yield params
    lib.osrmc_route_params_destruct(params)

@contextmanager
def scoped_route(osrm, params):
    route = lib.osrmc_route(osrm, params, c.byref(osrmc_error()))
    yield route
    lib.osrmc_route_response_destruct(route)

@contextmanager
def scoped_table_params():
    params = lib.osrmc_table_params_construct(c.byref(osrmc_error()))
    yield params
    lib.osrmc_table_params_destruct(params)

@contextmanager
def scoped_table(osrm, params):
    route = lib.osrmc_table(osrm, params, c.byref(osrmc_error()))
    yield route
    lib.osrmc_table_response_destruct(route)


Coordinate = namedtuple('Coordinate', 'longitude latitude')
Route = namedtuple('Route', 'distance duration')
Table = list


class Route:
    def __init__(_, response):
        _._response = response

    def status(_):
        return _._response['code'] == 'Ok'

    @property
    def routes(_):
        return _._response['routes']

    @property
    def distance(_):
        return sum([r['distance'] for r in _._response['routes']])

    @property
    def duration(_):
        return sum([r['duration'] for r in _._response['routes']])


class OSRM:
    def __init__(_, base_path):
        _.config = None
        _.osrm = None

        _.config = lib.osrmc_config_construct(base_path.encode('utf-8'), c.byref(osrmc_error()))
        assert _.config

        _.osrm = lib.osrmc_osrm_construct(_.config, c.byref(osrmc_error()))
        assert _.osrm

    def __del__(_):
        if _.osrm:
            lib.osrmc_osrm_destruct(_.osrm)
        if _.config:
            lib.osrmc_config_destruct(_.config)

    def route(_, coordinates,
              bearings=[], radiuses=[], generate_hints=False, hints=[],
              alternatives=False, steps=False,
              annotations=False,
              geometries='polyline', overview='simplified',
              continue_straight='default'):
        # bearings is a list of tuples with (bearing, range)
        # radiuses is list of floats
        route = lib.osrmc_route(_.osrm, {
            'coordinates': [(coordinate.longitude, coordinate.latitude)
                            for coordinate in coordinates],
            'bearings': bearings,
            'radiuses': radiuses,
            'generate_hints': generate_hints,
            'hints': hints,
            'alternatives': alternatives,
            'annotations': annotations,
            'geometries': geometries,
            'overview': overview,
            'continue_straight': continue_straight
            }, c.byref(osrmc_error()))
        if not route:
            return

        ret = lib.osrmc_json_to_pyobj(route)
        lib.osrmc_route_response_destruct(route)
        return Route(ret)

    def table(_, coordinates):
        with scoped_table_params() as params:
            assert params

            for coordinate in coordinates:
                lib.osrmc_params_add_coordinate(params, coordinate.longitude, coordinate.latitude, c.byref(osrmc_error()))

            with scoped_table(_.osrm, params) as table:
                if table:
                    n = len(coordinates)  # Only symmetric version supported
                    return Table([lib.osrmc_table_response_duration(table, s, t, c.byref(osrmc_error())) for t in range(n)] for s in range(n))
                else:
                    return None
