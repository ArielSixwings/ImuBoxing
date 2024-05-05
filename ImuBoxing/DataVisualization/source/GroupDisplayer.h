#pragma once

#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkNamedColors.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkLineSource.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkRenderer.h>
#include <vtkCommand.h>

namespace visualization
{
    class GroupDisplayer
    {
    private:
        /* data */
    public:
        struct PointActor
        {
            PointActor(const std::vector<double> &center);
            PointActor(const std::vector<double> &center, double radius);

            vtkSmartPointer<vtkSphereSource> Center;
            vtkSmartPointer<vtkPolyDataMapper> Mapper;
            vtkSmartPointer<vtkActor> Actor;
        };

        GroupDisplayer();
    };
}