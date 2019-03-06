// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgridHolo.hpp>
#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/python/boost_serialization_pickle.hpp>

BOOST_PYTHON_MODULE(libaslam_cameras_april_python)
{
  using namespace boost::python;
  using namespace aslam::cameras;

  class_<GridCalibrationTargetAprilgrid::AprilgridOptions>("AprilgridOptions", init<>())
    .def_readwrite("doSubpixRefinement", &GridCalibrationTargetAprilgrid::AprilgridOptions::doSubpixRefinement)
    .def_readwrite("showExtractionVideo", &GridCalibrationTargetAprilgrid::AprilgridOptions::showExtractionVideo)
    .def_readwrite("minTagsForValidObs", &GridCalibrationTargetAprilgrid::AprilgridOptions::minTagsForValidObs)
    .def_readwrite("minBorderDistance", &GridCalibrationTargetAprilgrid::AprilgridOptions::minBorderDistance)
    .def_readwrite("maxSubpixDisplacement2", &GridCalibrationTargetAprilgrid::AprilgridOptions::maxSubpixDisplacement2)
    .def_readwrite("blackTagBorder", &GridCalibrationTargetAprilgrid::AprilgridOptions::blackTagBorder)
    .def_pickle(sm::python::pickle_suite<GridCalibrationTargetAprilgrid::AprilgridOptions>());

  class_<GridCalibrationTargetAprilgrid, bases<GridCalibrationTargetBase>,
      boost::shared_ptr<GridCalibrationTargetAprilgrid>, boost::noncopyable>(
      "GridCalibrationTargetAprilgrid",
      init<size_t, size_t, double, double, GridCalibrationTargetAprilgrid::AprilgridOptions>(
          "GridCalibrationTargetAprilgrid(size_t tagRows, size_t tagCols, double tagSize, double tagSpacing, AprilgridOptions options)"))
      .def(init<size_t, size_t, double, double>(
          "GridCalibrationTargetAprilgrid(size_t tagRows, size_t tagCols, double tagSize, double tagSpacing)"))
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def_pickle(sm::python::pickle_suite<GridCalibrationTargetAprilgrid>());

  class_<GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo>("AprilgridOptionsHolo", init<>())
    .def_readwrite("doSubpixRefinement", &GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo::doSubpixRefinement)
    .def_readwrite("showExtractionVideo", &GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo::showExtractionVideo)
    .def_readwrite("minTagsForValidObs", &GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo::minTagsForValidObs)
    .def_readwrite("minBorderDistance", &GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo::minBorderDistance)
    .def_readwrite("maxSubpixDisplacement2", &GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo::maxSubpixDisplacement2)
    .def_readwrite("blackTagBorder", &GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo::blackTagBorder)
    .def_pickle(sm::python::pickle_suite<GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo>());

  class_<GridCalibrationTargetAprilgridHolo, bases<GridCalibrationTargetBase>,
      boost::shared_ptr<GridCalibrationTargetAprilgridHolo>, boost::noncopyable>(
      "GridCalibrationTargetAprilgridHolo",
      init<size_t, size_t, std::string, GridCalibrationTargetAprilgridHolo::AprilgridOptionsHolo>(
          "GridCalibrationTargetAprilgridHolo(size_t tagRows, size_t tagCols, std::string tagDatabase_file, AprilgridOptionsHolo options)"))
      .def(init<size_t, size_t, std::string>(
          "GridCalibrationTargetAprilgridHolo(size_t tagRows, size_t tagCols, std::string tagDatabase_file)"))
      .def(init<>("Do not use the default constructor. It is only necessary for the pickle interface"))
      .def_pickle(sm::python::pickle_suite<GridCalibrationTargetAprilgridHolo>());
}
