#  Copyright (c) 2020, Open Source Robotics Foundation
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of the Open Source Robotics Foundation nor the names of its
#        contributors may be used to endorse or promote products derived from
#        this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.


# make string literals unicode default in Python 2
from __future__ import unicode_literals
from sys import argv


class Namespace(object):
    """
    This class represents a C++ namespace.
    If the name is empty, the namespace block is skipped
    (e.g. for the global namespace)
    """
    def __init__(self, name):
        self._name = name
        self._members = []

    def write(self, out_file):
        """Write namespace including all members into a file"""
        if self._name == "":
            # global namespace
            self._write_members(out_file)
            out_file.write("\n\n")
        else:
            out_file.write("namespace " + self._name + " {\n")
            self._write_members(out_file)
            out_file.write("}\n\n")

    def _write_members(self, out_file):
        for i in self._members:
            i.write(out_file)

    def add_member(self, a_member):
        """Add a namespace member"""
        self._members.append(a_member)
        return self

    def write_name(self, out_file):
        """Write the namespace name and two colons into a file"""
        out_file.write(self._name + "::")

    def write_includes(self, out_file):
        """Write all header includes of the members into a file (sorted)"""
        includes = set()
        for i in self._members:
            includes = i.add_include_header_to_set(includes)
        out_file.write("\n".join(sorted(includes)))
        out_file.write("\n\n")

    def _load_cxx_class(self, varname, *args, **kwargs):
        """
        Load a C++ class and store it in self.<varname>
        The name passed to the C++ Class is set to varname
        but can be overwritten with a name kwarg.
        All other arguments are passed on.
        """
        self.__setattr__(varname, CxxClass(kwargs.pop("name", varname), self, *args, **kwargs))


class Template(object):
    """This class represents a C++ template"""
    def __init__(self, parameters):
        """
        construct a new C++ template wrapper
        parameters: list of tuples with (type, name) of each parameter
        """
        self._parameters = parameters

    def write_declaration(self, out_file):
        """
        Write the template declaration (including names and types,
        like `template <int i>`) into a file
        """
        params = ["%s %s" % i for i in self._parameters]
        if len(params) == 0:
            return
        out_file.write("template<" + ", ".join(params) + ">\n")

    def write_params_for_specialisation(self, out_file, param_range=slice(None)):
        """
        Write the names of the parameters, like `<i>` into a file.
        Used for defining or using template specializations.
        out_file: File object to write into
        param_range: range for names to pick, default all
        """
        params = [i[1] for i in self._parameters[param_range]]
        if len(params) == 0:
            return
        out_file.write("<" + ",".join(params) + ">")

    def get_types(self):
        """Returns a tuple with all C++ template parameter types"""
        return tuple(i[0] for i in self._parameters)


class CxxClassBase(object):
    """Base class for C++ classes"""
    def __init__(self, name, namespace=None, stamped=False):
        self._name = name
        if namespace is None:
            self._namespace = None
        else:
            self._namespace = namespace.add_member(self)
        self._stamped = stamped

    def add_include_header_to_set(self, includes):
        """Add an `#include` statement to a set()"""
        return includes

    def stamped(self):
        """
        Returns True when only conversions to/from stamped message are available,
        False when only conversions to/from unstamped are available and "both"
        when both stamped and unstamped conversions are available.
        """
        return self._stamped

    def compare_stamped(self, other):
        """
        Compare the availability of (un)stamped conversions of two classes.
        It returns either "both", True, False or None
        """
        if self.stamped() == other.stamped() == "both":
            # both stamped and unstamped
            return "both"
        elif self.stamped() and other.stamped():
            # (stamped only, both) or (stamped only, stamped only)
            return True
        elif not (self.stamped() or other.stamped()):
            # (unstamped only, unstamped only)
            return False
        else:
            # (unstamped only, stamped only) -> no match
            return None


class CxxClass(CxxClassBase):
    """This class represents a forward declaration of a C++ class"""
    def __init__(self, name, namespace=None, template=None, stamped=False):
        super(CxxClass, self).__init__(name, namespace, stamped)
        self._template = template

    def write(self, out_file):
        """Write forward declaration into a file (without namespace prefix)"""
        if self._template is not None:
            self._template.write_declaration(out_file)
        out_file.write("class " + self._name + ";\n")

    def get_template_types(self):
        """Return a tuple with the template parameter C++ types"""
        if self._template is None:
            return tuple()
        else:
            return self._template.get_types()

    def write_name(self, out_file):
        """write the name into a file (with namespace prefix but without template parameters)"""
        if self._namespace is None:
            out_file.write("::")
        else:
            self._namespace.write_name(out_file)
        out_file.write(self._name)


class TypeMap(CxxClassBase):
    """
    This class represents a mapping between two C++ classes and a geometry_msgs type
    which can be converted into each other
    """
    def __init__(self, msgs_name, class_a, class_b, namespace, stamped=False):
        """
        Creates a new mapping between the C++ classes class_a and class_b
        with the message type msgs_name.
        If stamped is true, "Stamped" is appended to msgs_name.
        """
        if not isinstance(stamped, bool):
            raise ValueError("stamped must be eighter True or False")
        super(TypeMap, self).__init__("BidirectionalTypeMap", namespace, stamped)
        self._msgs_name = msgs_name
        if stamped:
            self._msgs_name += "Stamped"
        self._classes = (class_a, class_b)
        # build template arguments if necessary by looking at both classes
        self._templates_count = [0, 0]
        template_types = []
        for i, c in enumerate(self._classes):
            needed_types = list(c.get_template_types())
            template_types += needed_types
            self._templates_count[i] = len(needed_types)
        if len(template_types) == 0:
            self._template = None
        else:
            # build template with names T1, T2 etc.
            self._template = Template([(t, "T%d" % i) for i, t in enumerate(template_types)])

    def write(self, out_file):
        """Write the mapping into a file"""
        if self._template:
            self._template.write_declaration(out_file)
        else:
            out_file.write("template<>\n")
        out_file.write("struct %s<" % self._name)

        self._write_class_name(out_file, 0)
        out_file.write(", ")
        self._write_class_name(out_file, 1)

        out_file.write("> {\nusing type = ::geometry_msgs::" + self._msgs_name + ";\n};\n\n")

    def _write_class_name(self, out_file, idx):
        """
        Write the full name including template parameters
        of the first (second) mapped class into a file.
        idx: {0 = first, 1 = second} mapped class
        """
        if self.stamped():
            out_file.write("::tf2::Stamped<")
        self._classes[idx].write_name(out_file)
        if self._template:
            if idx == 0:
                s = slice(0, self._templates_count[0])
            else:
                s = slice(self._templates_count[0], None)
            self._template.write_params_for_specialisation(out_file, s)
        if self.stamped():
            out_file.write(">")

    def add_include_header_to_set(self, includes):
        includes.add("#include <geometry_msgs/%s.h>" % self._msgs_name)
        return includes


def TypeMapIterator(msgs_name, namespace, classes):
    """
    Iterate pairwise over classes and create TypeMap for each pair.
    For example, for classes=[a, b, c, d] it yields [ Typemap(a,b), Typemap(a,c),
    Typemap(a,d), Typemap(b,c), Typemap(b,d), Typemap(c,d)].
    The stamped attribute of each class is honored.
    """
    for i in range(0, len(classes)):
        for next_class in classes[i+1:]:
            is_stamped = next_class.compare_stamped(classes[i])
            if is_stamped is None:
                continue
            if is_stamped:  # True or "both"
                yield TypeMap(msgs_name, classes[i], next_class, namespace, True)
            if (not is_stamped) or (is_stamped == "both"):  # False or "both"
                yield TypeMap(msgs_name, classes[i], next_class, namespace, False)


# Known classes from other libraries #

class Bullet(Namespace):
    def __init__(self):
        super(Bullet, self).__init__("")

        self._load_cxx_class("Vector3", name="btVector3", stamped=True)


class Eigen(Namespace):
    def __init__(self):
        super(Eigen, self).__init__("Eigen")

        self._load_cxx_class("Vector3",
                             Template([("typename", "T"), ("int", "_rows"), ("int", "_cols"),
                                       ("int", "_options"), ("int", "_maxrows"),
                                       ("int", "maxcols")]),
                             stamped="both", name="Matrix")
        self._load_cxx_class("Quaternion",
                             Template([("typename", "T"), ("int", "_options")]),
                             stamped="both")
        self._load_cxx_class("Transform",
                             Template([("typename", "T"), ("int", "_dim"),
                                       ("int", "_mode"), ("int", "_options")]),
                             stamped="both")


class KDL(Namespace):
    def __init__(self):
        super(KDL, self).__init__("KDL")

        self._load_cxx_class("Frame", stamped="both")
        self._load_cxx_class("Vector3", name="Vector", stamped=True)
        self.Twist = CxxClass("Twist", self, stamped=True)
        self.Wrench = CxxClass("Wrench", self, stamped=True)


class TF2_Forward(Namespace):
    def __init__(self):
        super(TF2_Forward, self).__init__("tf2")

        self._load_cxx_class("Vector3", stamped="both")
        self._load_cxx_class("Quaternion", stamped="both")
        self._load_cxx_class("Transform", stamped="both")
        self._load_cxx_class("Wrench", stamped=True)


# generate mappings #

def build_mappings():
    bullet = Bullet()
    eigen = Eigen()
    kdl = KDL()
    tf2_fwd = TF2_Forward()

    maps = {
        "Pose":       [
            tf2_fwd.Transform,
            eigen.Transform,
            kdl.Frame,
            ],
        # "Vector3":    [eigen.Vector3, tf2_fwd.Vector3],
        "Point":      [
            eigen.Vector3,
            tf2_fwd.Vector3,
            kdl.Vector3,
            bullet.Vector3,
            ],
        "Quaternion": [
            eigen.Quaternion,
            tf2_fwd.Quaternion,
            ],
        "Wrench":     [
            tf2_fwd.Wrench,
            kdl.Wrench,
            ],
    }

    ns_tf2 = Namespace("tf2")

    for msgs_name, classes in sorted(maps.items()):
        for mapping in TypeMapIterator(msgs_name, ns_tf2, classes):
            pass

    return (bullet, tf2_fwd, eigen, kdl, ns_tf2)


hdr_begin = """/*
 * Copyright (c) 2020, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Open Source Robotics Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


 // NOTE: This file is generated automatically, do not edit.

#pragma once

#include <tf2/transform_functions.h>
#include <tf2/transform_datatypes.h>

"""

if __name__ == "__main__":
    with open(argv[1], "w") as f:
        f.write(hdr_begin)
        namespaces = build_mappings()
        namespaces[-1].write_includes(f)

        for ns in namespaces:
            ns.write(f)
