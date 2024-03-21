#include <Python.h>
#include <iostream>
#include <algorithm>

//clear && g++ -c main.cpp $(python3.10-config --includes) && g++ -o caller main.o $(python3.10-config --ldflags) -lpython3.10

// Function to convert a Python list to a C++ std::vector<float>
std::vector<float> convertPythonListToFloatVector(PyObject* list_obj) {
  
  if (not PyList_Check(list_obj)) {

    PyErr_SetString(PyExc_TypeError, "Expected a Python list");
    return {};
  }

  Py_ssize_t size = PyList_Size(list_obj);

  std::vector<float> result(size);
  
  for (int i = 0; i < size; ++i) {

    PyObject* item = PyList_GetItem(list_obj, i);

    if (not PyFloat_Check(item)) {
    
      PyErr_SetString(PyExc_TypeError, "Expected list to contain floats");
      return {};
    }
    
    result[i] = static_cast<float>(PyFloat_AsDouble(item));
  }
  return result;
}

int main() {
  // Initialize Python interpreter
  Py_Initialize();

  // Add the current directory to the Python module search path
  PySys_SetPath(Py_DecodeLocale(".", NULL));

  
  // Import the Python module
  PyObject* telemetryModule = PyImport_ImportModule("Telemetry");
  
  if (not telemetryModule) {

    PyErr_Print();
    return 1;
  }

  // Obtain a reference to the Foo method
  PyObject* fooFunction = PyObject_GetAttrString(telemetryModule, "Foo");
  
  if (not fooFunction) {

    PyErr_Print();
    return 1;
  }

  // Call the Foo method and handle potential errors
  PyObject* result = PyObject_CallObject(fooFunction, NULL);
  if (not result) {
    PyErr_Print();
    return 1;
  }

  // Convert the result to a C++ std::vector<float>
  std::vector<float> data = convertPythonListToFloatVector(result);

  // Print the data (modify this section to use the data in your program)
  if (not data.empty()) 
  {
    std::cout << "Data from Python: ";
    for (float value : data) {
      std::cout << 2*value << " ";
    }
    std::cout << std::endl;
  } 
  else 
  {
    std::cerr << "Error: Could not convert Python list to C++ vector." << std::endl;
  }

  // Clean up
  Py_DECREF(result);
  Py_DECREF(fooFunction);
  Py_DECREF(telemetryModule);
  Py_Finalize();

  return 0;
}
