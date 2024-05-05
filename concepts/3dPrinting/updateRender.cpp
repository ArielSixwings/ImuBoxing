#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>

// Function to update points dynamically. This is just an example function.
void UpdateDynamicPoints(vtkSmartPointer<vtkPoints> points)
{
    // Logic to update points could be more complex depending on your application
    // Here, we just move the point randomly for demonstration purposes
    double x = static_cast<double>(rand()) / RAND_MAX * 10.0; // New random x-coordinate
    double y = static_cast<double>(rand()) / RAND_MAX * 10.0; // New random y-coordinate
    double z = static_cast<double>(rand()) / RAND_MAX * 10.0; // New random z-coordinate

    // Update the first point with new coordinates
    points->SetPoint(0, x, y, z);
    points->Modified(); // Notify VTK that the points have been changed
}

// Define the TimerCallback as before but now include the UpdateDynamicPoints call
class TimerCallback : public vtkCommand
{
public:
    static TimerCallback *New()
    {
        return new TimerCallback;
    }

    virtual void Execute(vtkObject *caller, unsigned long eventId, void *vtkNotUsed(callData))
    {
        vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor *>(caller);

        // Update dynamic points
        vtkSmartPointer<vtkPolyData> polyData = vtkPolyData::SafeDownCast(dynamicPointActor->GetMapper()->GetInput());
        vtkPoints *points = polyData->GetPoints();
        UpdateDynamicPoints(points);

        // Force the actor to update and rerender
        dynamicPointActor->GetMapper()->GetInput()->Modified();
        iren->Render(); // Rerender the scene
    }

    vtkActor *dynamicPointActor;

    void SetDynamicActor(vtkActor *actor)
    {
        this->dynamicPointActor = actor;
    }
};

int main()
{
    // Create points for constant and dynamic actors
    vtkSmartPointer<vtkPoints> constantPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPoints> dynamicPoints = vtkSmartPointer<vtkPoints>::New();
    constantPoints->InsertNextPoint(1.0, 0.0, 0.0); // Example point
    dynamicPoints->InsertNextPoint(0.0, 1.0, 0.0);  // Example point

    // Convert vtkPoints to vtkPolyData
    vtkSmartPointer<vtkPolyData> constantPolyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> dynamicPolyData = vtkSmartPointer<vtkPolyData>::New();
    constantPolyData->SetPoints(constantPoints);
    dynamicPolyData->SetPoints(dynamicPoints);

    // Setup mappers
    vtkSmartPointer<vtkPolyDataMapper> constantMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkPolyDataMapper> dynamicMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    constantMapper->SetInputData(constantPolyData);
    dynamicMapper->SetInputData(dynamicPolyData);

    // Setup actors
    vtkSmartPointer<vtkActor> constantActor = vtkSmartPointer<vtkActor>::New();
    vtkSmartPointer<vtkActor> dynamicActor = vtkSmartPointer<vtkActor>::New();
    constantActor->SetMapper(constantMapper);
    dynamicActor->SetMapper(dynamicMapper);
    dynamicActor->GetProperty()->SetColor(65.0 / 255.0, 105.0 / 255.0, 225.0 / 255.0); // RoyalBlue1 color

    // Setup renderer
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Add actors to the renderer
    renderer->AddActor(constantActor);
    renderer->AddActor(dynamicActor);

    // Timer callback to update dynamic actor
    vtkSmartPointer<TimerCallback> timerCallback = vtkSmartPointer<TimerCallback>::New();
    timerCallback->SetDynamicActor(dynamicActor);
    renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, timerCallback);
    renderWindowInteractor->CreateRepeatingTimer(100); // Create a timer that triggers every 100 milliseconds

    // Start interaction
    renderWindow->Render();
    renderWindowInteractor->Start();

    return 0;
}
