#include "GroupDisplayer.h"

namespace visualization
{
    GroupDisplayer::PointActor::PointActor(const std::vector<double> &center)
    {
        Center = vtkSmartPointer<vtkSphereSource>::New();
        Center->SetCenter(center[0], center[1], center[2]);
        Center->SetRadius(0.1);

        Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        Mapper->SetInputConnection(Center->GetOutputPort());

        Actor = vtkSmartPointer<vtkActor>::New();
        Actor->SetMapper(Mapper);
        Actor->GetProperty()->SetColor(50.0 / 255.0, 205.0 / 255.0, 50.0 / 255.0); // LimeGreen color
    }

    GroupDisplayer::PointActor::PointActor(const std::vector<double> &center, double radius)
        : PointActor(center)
    {

        Center->SetRadius(radius);

        Actor->GetProperty()->SetColor(65.0 / 255.0, 105.0 / 255.0, 225.0 / 255.0); // RoyalBlue1 color
        Actor->GetProperty()->SetOpacity(0.3);
    }

    GroupDisplayer::GroupDisplayer()
    {
        const std::vector<double> center = {1.0, 0.0, 1.0};

        PointActor pointActor(center);
        PointActor sphereActor(center, 1.0);
        vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
        renderer->AddActor(pointActor.Actor);
        renderer->AddActor(sphereActor.Actor);

        // Create a render window and set its size
        vtkSmartPointer<vtkRenderWindow> rendererWindow = vtkSmartPointer<vtkRenderWindow>::New();
        rendererWindow->AddRenderer(renderer);
        rendererWindow->SetSize(600, 600);

        // Create an interactor and start the visualization
        vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        interactor->SetRenderWindow(rendererWindow);

        interactor->Start();
    }
}