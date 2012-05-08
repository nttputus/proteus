%module rawlasertwist

%typemap(in) (float * ranges, int len) {
  int i;
  $2 = PySequence_Length($input);
  $1 = (float *) malloc($2*sizeof(float));
  for (i = 0; i < $2; i++) {
    PyObject *o = PySequence_GetItem($input,i);
    $1[i] = (float) PyFloat_AsDouble(o);
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

float * rawlasertwist(float * ranges, int len);

