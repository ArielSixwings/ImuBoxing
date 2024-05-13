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

    GroupDisplayer::CartesianLineActor::CartesianLineActor(const std::vector<double> &axis)
    {
        vtkSmartPointer<vtkLineSource> Line = vtkSmartPointer<vtkLineSource>::New();
        Line->SetPoint1(-3, 0, 0);
        Line->SetPoint2(3, 0, 0);

        vtkSmartPointer<vtkPolyDataMapper> Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        Mapper->SetInputConnection(Line->GetOutputPort());

        vtkSmartPointer<vtkActor> Actor = vtkSmartPointer<vtkActor>::New();
        Actor->SetMapper(Mapper);
        Actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
    }

    GroupDisplayer::GroupDisplayer()
    {
        const std::vector<double> guard = {-1.100411, 1.663938, 0.849442};
        const std::vector<double> jabEnd = {-0.088908, 0.242127, 0.009735};
        const std::vector<double> hookEnd = {-0.038916, 1.390083, -0.210620};
        const std::vector<double> uppercutEnd = {-1.283240, 0.009307, 2.065155};

        PointActor guardPointActor(guard);
        PointActor guardSphereActor(guard, 1.659983);

        PointActor jabEndPointActor(jabEnd);
        PointActor jabEndSphereActor(jabEnd, 1.515656);

        PointActor hookEndPointActor(hookEnd);
        PointActor hookEndSphereActor(hookEnd, 2.280423);

        PointActor uppercutEndPointActor(uppercutEnd);
        PointActor uppercutEndSphereActor(uppercutEnd, 5.006083);

        vtkSmartPointer<vtkLineSource> xLine = vtkSmartPointer<vtkLineSource>::New();
        xLine->SetPoint1(-3, 0, 0);
        xLine->SetPoint2(3, 0, 0);

        vtkSmartPointer<vtkLineSource> yLine = vtkSmartPointer<vtkLineSource>::New();
        yLine->SetPoint1(0, -3, 0);
        yLine->SetPoint2(0, 3, 0);

        vtkSmartPointer<vtkLineSource> zLine = vtkSmartPointer<vtkLineSource>::New();
        zLine->SetPoint1(0, 0, -3);
        zLine->SetPoint2(0, 0, 3);

        // Create mappers for the lines
        vtkSmartPointer<vtkPolyDataMapper> xMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        xMapper->SetInputConnection(xLine->GetOutputPort());

        vtkSmartPointer<vtkPolyDataMapper> yMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        yMapper->SetInputConnection(yLine->GetOutputPort());

        vtkSmartPointer<vtkPolyDataMapper> zMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        zMapper->SetInputConnection(zLine->GetOutputPort());

        // Create actors for the lines
        vtkSmartPointer<vtkActor> xActor = vtkSmartPointer<vtkActor>::New();
        xActor->SetMapper(xMapper);
        xActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // Red color for x-axis

        vtkSmartPointer<vtkActor> yActor = vtkSmartPointer<vtkActor>::New();
        yActor->SetMapper(yMapper);
        yActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // Green color for y-axis

        vtkSmartPointer<vtkActor> zActor = vtkSmartPointer<vtkActor>::New();
        zActor->SetMapper(zMapper);
        zActor->GetProperty()->SetColor(0.0, 0.0, 1.0); // Blue color for z-axis

        // Create a renderer and add actors

        vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

        renderer->AddActor(guardPointActor.Actor);
        renderer->AddActor(guardSphereActor.Actor);

        renderer->AddActor(jabEndPointActor.Actor);
        renderer->AddActor(jabEndSphereActor.Actor);

        renderer->AddActor(hookEndPointActor.Actor);
        renderer->AddActor(hookEndSphereActor.Actor);

        renderer->AddActor(uppercutEndPointActor.Actor);
        renderer->AddActor(uppercutEndSphereActor.Actor);

        renderer->AddActor(xActor);
        renderer->AddActor(yActor);
        renderer->AddActor(zActor);

        vtkSmartPointer<vtkRenderWindow> rendererWindow = vtkSmartPointer<vtkRenderWindow>::New();
        rendererWindow->AddRenderer(renderer);
        rendererWindow->SetSize(600, 600);

        vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        interactor->SetRenderWindow(rendererWindow);

        interactor->Start();
    }
}