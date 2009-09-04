#include <Python.h>

#include "tf/tf.h"

static PyObject *tf_error = NULL;
static PyObject *pModulerospy = NULL;

struct transformer_t {
  PyObject_HEAD
  tf::Transformer *t;
};

static PyTypeObject transformer_Type = {
  PyObject_HEAD_INIT(&PyType_Type)
  0,                               /*size*/
  "_tf.Transformer",                /*name*/
  sizeof(transformer_t),           /*basicsize*/
};

static PyObject *PyObject_BorrowAttrString(PyObject* o, const char *name)
{
    PyObject *r = PyObject_GetAttrString(o, name);
    if (r != NULL)
      Py_DECREF(r);
    return r;
}

static int rostime_converter(PyObject *obj, ros::Time *rt)
{
  PyObject *tsr = PyObject_CallMethod(obj, (char*)"to_seconds", NULL);
  if (tsr == NULL) {
    PyErr_SetString(PyExc_TypeError, "time must have a to_seconds method, e.g. rospy.Time or rospy.Duration");
    return 0;
  } else {
    (*rt).fromSec(PyFloat_AsDouble(tsr));
    Py_DECREF(tsr);
    return 1;
  }
}

static int rosduration_converter(PyObject *obj, ros::Duration *rt)
{
  PyObject *tsr = PyObject_CallMethod(obj, (char*)"to_seconds", NULL);
  if (tsr == NULL) {
    PyErr_SetString(PyExc_TypeError, "time must have a to_seconds method, e.g. rospy.Time or rospy.Duration");
    return 0;
  } else {
    (*rt).fromSec(PyFloat_AsDouble(tsr));
    Py_DECREF(tsr);
    return 1;
  }
}

static int Transformer_init(PyObject *self, PyObject *args, PyObject *kw)
{
  int interpolating = 1;
  ros::Duration cache_time;

  cache_time.fromSec(tf::Transformer::DEFAULT_CACHE_TIME);

  if (!PyArg_ParseTuple(args, "|iO&", &interpolating, rosduration_converter, &cache_time))
    return -1;

  ((transformer_t*)self)->t = new tf::Transformer(interpolating, cache_time);

  return 0;
}

static PyObject *allFramesAsDot(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  return PyString_FromString(t->allFramesAsDot().c_str());
}

static PyObject *allFramesAsString(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  return PyString_FromString(t->allFramesAsString().c_str());
}

static PyObject *canTransform(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame;
  ros::Time time;
  static char *keywords[] = { "target_frame", "source_frame", "time", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&", keywords, &target_frame, &source_frame, rostime_converter, &time))
    return NULL;
  return PyBool_FromLong(t->canTransform(target_frame, source_frame, time));
}

static PyObject *canTransformFull(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  static char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&s", keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame))
    return NULL;
  return PyBool_FromLong(t->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame));
}

static PyObject *asListOfStrings(std::vector< std::string > los)
{
  PyObject *r = PyList_New(los.size());
  size_t i;
  for (i = 0; i < los.size(); i++) {
    PyList_SetItem(r, i, PyString_FromString(los[i].c_str()));
  }
  return r;
}

static PyObject *chain(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  std::vector< std::string > output;
  static char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&s", keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame))
    return NULL;

  t->chainAsVector(target_frame, target_time, source_frame, source_time, fixed_frame, output);
  return asListOfStrings(output);
}

static PyObject *getLatestCommonTime(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *source, *dest;
  std::string error_string;
  ros::Time time;

  if (!PyArg_ParseTuple(args, "ss", &source, &dest))
    return NULL;
  int r = t->getLatestCommonTime(source, dest, time, &error_string);
  if (r == 0) {
    PyObject *rospy_time = PyObject_GetAttrString(pModulerospy, "Time");
    PyObject *args = Py_BuildValue("ii", time.sec, time.nsec);
    PyObject *ob = PyObject_CallObject(rospy_time, args);
    Py_DECREF(args);
    Py_DECREF(rospy_time);
    return ob;
  } else {
    PyErr_SetString(tf_error, error_string.c_str());
    return NULL;
  }
}

static PyObject *lookupTransform(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame;
  ros::Time time;
  static char *keywords[] = { "target_frame", "source_frame", "time", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&", keywords, &target_frame, &source_frame, rostime_converter, &time))
    return NULL;
  tf::Stamped< btTransform > transform;
  try
  {
    t->lookupTransform(target_frame, source_frame, time, transform);
  } 
  catch (const tf::LookupException &e)
  {
    PyErr_SetString(tf_error, e.what());
    return NULL;
  }
  btVector3 origin = transform.getOrigin();
  btQuaternion rotation = transform.getRotation();
  return Py_BuildValue("(ddd)(dddd)",
      origin.x(), origin.y(), origin.z(),
      rotation.x(), rotation.y(), rotation.z(), rotation.w());
}

static PyObject *lookupTransformFull(PyObject *self, PyObject *args, PyObject *kw)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *target_frame, *source_frame, *fixed_frame;
  ros::Time target_time, source_time;
  static char *keywords[] = { "target_frame", "target_time", "source_frame", "source_time", "fixed_frame", NULL };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "sO&sO&s", keywords,
                        &target_frame,
                        rostime_converter,
                        &target_time,
                        &source_frame,
                        rostime_converter,
                        &source_time,
                        &fixed_frame))
    return NULL;
  tf::Stamped< btTransform > transform;
  try
  {
    t->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, transform);
  } 
  catch (const tf::LookupException &e)
  {
    PyErr_SetString(tf_error, e.what());
    return NULL;
  }
  btVector3 origin = transform.getOrigin();
  btQuaternion rotation = transform.getRotation();
  return Py_BuildValue("(ddd)(dddd)",
      origin.x(), origin.y(), origin.z(),
      rotation.x(), rotation.y(), rotation.z(), rotation.w());
}

static PyObject *setTransform(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  PyObject *py_transform;
  char *authority = (char*)"default_authority";

  if (!PyArg_ParseTuple(args, "O|s", &py_transform, &authority))
    return NULL;
  tf::Stamped< btTransform > transform;
  PyObject *header = PyObject_BorrowAttrString(py_transform, "header");
  transform.frame_id_ = PyString_AsString(PyObject_BorrowAttrString(py_transform, "child_frame_id"));
  transform.parent_id_ = PyString_AsString(PyObject_BorrowAttrString(header, "frame_id"));
  if (rostime_converter(PyObject_BorrowAttrString(header, "stamp"), &transform.stamp_) != 1)
    return NULL;

  PyObject *mtransform = PyObject_BorrowAttrString(py_transform, "transform");
  PyObject *translation = PyObject_BorrowAttrString(mtransform, "translation");
  double tx = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "x"));
  double ty = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "y"));
  double tz = PyFloat_AsDouble(PyObject_BorrowAttrString(translation, "z"));
  PyObject *rotation = PyObject_BorrowAttrString(mtransform, "rotation");
  double qx = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "x"));
  double qy = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "y"));
  double qz = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "z"));
  double qw = PyFloat_AsDouble(PyObject_BorrowAttrString(rotation, "w"));

  transform.setData(btTransform(
    btQuaternion(btScalar(qx), btScalar(qy), btScalar(qz), btScalar(qw)),
    btVector3(btScalar(tx), btScalar(ty), btScalar(tz))));
  t->setTransform(transform, authority);
  Py_RETURN_NONE;
}

static PyObject *clear(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  t->clear();
  Py_RETURN_NONE;
}

static PyObject *frameExists(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  char *frame_id_str;
  if (!PyArg_ParseTuple(args, "s", &frame_id_str))
    return NULL;
  return PyBool_FromLong(t->frameExists(frame_id_str));
}

static PyObject *getFrameStrings(PyObject *self, PyObject *args)
{
  tf::Transformer *t = ((transformer_t*)self)->t;
  std::vector< std::string > ids;
  t->getFrameStrings(ids);
  return asListOfStrings(ids);
}

static struct PyMethodDef transformer_methods[] =
{
  {"allFramesAsDot", allFramesAsDot, METH_VARARGS},
  {"allFramesAsString", allFramesAsString, METH_VARARGS},
  {"setTransform", setTransform, METH_VARARGS},
  {"canTransform", (PyCFunction)canTransform, METH_KEYWORDS},
  {"canTransformFull", (PyCFunction)canTransformFull, METH_KEYWORDS},
  {"chain", (PyCFunction)chain, METH_KEYWORDS},
  {"clear", (PyCFunction)clear, METH_KEYWORDS},
  {"frameExists", (PyCFunction)frameExists, METH_VARARGS},
  {"getFrameStrings", (PyCFunction)getFrameStrings, METH_VARARGS},
  {"getLatestCommonTime", (PyCFunction)getLatestCommonTime, METH_VARARGS},
  {"lookupTransform", (PyCFunction)lookupTransform, METH_VARARGS},
  {"lookupTransformFull", (PyCFunction)lookupTransformFull, METH_VARARGS},
  {NULL,          NULL}
};

static PyMethodDef module_methods[] = {
  // {"Transformer", mkTransformer, METH_VARARGS},
  {NULL, NULL, NULL},
};

extern "C" void init_tf()
{
  PyObject *item, *m, *d;

#if PYTHON_API_VERSION >= 1007
  tf_error = PyErr_NewException((char*)"tf.error", NULL, NULL);
#else
  tf_error = PyString_FromString("tf.error");
#endif

  pModulerospy = PyImport_Import(item= PyString_FromString("rospy")); Py_DECREF(item);

  transformer_Type.tp_alloc = PyType_GenericAlloc;
  transformer_Type.tp_new = PyType_GenericNew;
  transformer_Type.tp_init = Transformer_init;
  transformer_Type.tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE;
  transformer_Type.tp_methods = transformer_methods;
  if (PyType_Ready(&transformer_Type) != 0)
    return;

  m = Py_InitModule("_tf", module_methods);
  PyModule_AddObject(m, "Transformer", (PyObject *)&transformer_Type);
  d = PyModule_GetDict(m);
  PyDict_SetItemString(d, "error", tf_error);
}
