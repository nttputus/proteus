%module rawlasertwist

%typemap(in) (float * ranges, int len) {
  int i, size;
  if (!PySequence_Check($input)) {
    PyErr_SetString(PyExc_ValueError,"Expected a sequence");
    return NULL;
  }
  size = PySequence_Length($input);
  $1 = (float *) malloc(size*sizeof(float));
  $2 = size;
  for (i = 0; i < size; i++) {
    PyObject *o = PySequence_GetItem($input,i);
    if (PyNumber_Check(o)) {
      $1[i] = (float) PyFloat_AsDouble(o);
    } else {
      PyErr_SetString(PyExc_ValueError,"Sequence elements must be numbers");      
      free($1);
      return NULL;
    }
  }
}

%typemap(freearg) float * {
   if ($1) free($1);
}

%typemap(out) float * {
  int i;
  $result = PyList_New(2);
  for (i = 0; i < 2; i++) {
    PyObject *o = PyFloat_FromDouble((double) $1[i]);
    PyList_SetItem($result,i,o);
  }
}

extern float * 
rawlasertwist(float * ranges, int len);

