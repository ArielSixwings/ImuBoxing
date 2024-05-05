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
    static TimerCallback *New() { return new TimerCallback; }
    virtual void Execute(vtkObject *caller, unsigned long eventId, void *vtkNotUsed(callData))
    {
        vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor *>(caller);
        if (!dynamicPointActor || !dynamicPointActor->GetMapper() || !dynamicPointActor->GetMapper()->GetInput())
            return;

        vtkPolyData *polyData = vtkPolyData::SafeDownCast(dynamicPointActor->GetMapper()->GetInput());
        if (!polyData)
            return;

        vtkPoints *points = polyData->GetPoints();
        if (!points)
            return;

        UpdateDynamicPoints(points);

        polyData->Modified(); // Important: notify VTK of changes to the polyData
        iren->Render();       // Re-render the scene
    }

    void SetDynamicActor(vtkActor *actor) { dynamicPointActor = actor; }

protected:
    vtkActor *dynamicPointActor = nullptr;
};

int main()
{
    vtkNew<vtkNamedColors> colors;

    // Set the background color.
    std::array<unsigned char, 4> bkg{{26, 51, 102, 255}};
    colors->SetColor("BkgColor", bkg.data());

    double largeRadius = 0.5; // Adjust radius as needed
    vtkSmartPointer<vtkSphereSource> largeSphereSourceB = vtkSmartPointer<vtkSphereSource>::New();
    largeSphereSourceB->SetCenter(1.1, 1.1, 1.1);
    largeSphereSourceB->SetRadius(largeRadius);

    // Create a small green sphere source
    double smallRadius = largeRadius * 0.2; // Make the small sphere 1/5th the size of the large sphere
    vtkSmartPointer<vtkSphereSource> smallSphereSourceB = vtkSmartPointer<vtkSphereSource>::New();
    smallSphereSourceB->SetCenter(1.1, 1.1, 1.1); // Place the small sphere at the same center as the large sphere
    smallSphereSourceB->SetRadius(smallRadius);

    // Create a large blue sphere source with desired radius
    vtkSmartPointer<vtkSphereSource> largeSphereSourceA = vtkSmartPointer<vtkSphereSource>::New();
    largeSphereSourceA->SetCenter(0.0, 0.0, 0.0);
    largeSphereSourceA->SetRadius(largeRadius);

    // Create a small green sphere source
    vtkSmartPointer<vtkSphereSource> smallSphereSourceA = vtkSmartPointer<vtkSphereSource>::New();
    smallSphereSourceA->SetCenter(0.0, 0.0, 0.0); // Place the small sphere at the same center as the large sphere
    smallSphereSourceA->SetRadius(smallRadius);

    // Create mappers for both spheres
    vtkSmartPointer<vtkPolyDataMapper> largeMapperA = vtkSmartPointer<vtkPolyDataMapper>::New();
    largeMapperA->SetInputConnection(largeSphereSourceA->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> smallMapperA = vtkSmartPointer<vtkPolyDataMapper>::New();
    smallMapperA->SetInputConnection(smallSphereSourceA->GetOutputPort());

    // Create mappers for both spheres
    vtkSmartPointer<vtkPolyDataMapper> largeMapperB = vtkSmartPointer<vtkPolyDataMapper>::New();
    largeMapperB->SetInputConnection(largeSphereSourceB->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> smallMapperB = vtkSmartPointer<vtkPolyDataMapper>::New();
    smallMapperB->SetInputConnection(smallSphereSourceB->GetOutputPort());

    // Create actors for both spheres and set their properties
    vtkSmartPointer<vtkActor> largeSphereActorB = vtkSmartPointer<vtkActor>::New();
    largeSphereActorB->SetMapper(largeMapperB);
    // Set color directly using RGB components
    largeSphereActorB->GetProperty()->SetColor(65.0 / 255.0, 105.0 / 255.0, 225.0 / 255.0); // RoyalBlue1 color
    largeSphereActorB->GetProperty()->SetOpacity(0.3);

    vtkSmartPointer<vtkActor> smallSphereActorB = vtkSmartPointer<vtkActor>::New();
    smallSphereActorB->SetMapper(smallMapperB);
    // Set color directly using RGB components
    smallSphereActorB->GetProperty()->SetColor(50.0 / 255.0, 205.0 / 255.0, 50.0 / 255.0); // LimeGreen color

    // Create actors for both spheres and set their properties
    vtkSmartPointer<vtkActor> largeSphereActorA = vtkSmartPointer<vtkActor>::New();
    largeSphereActorA->SetMapper(largeMapperA);
    // Set color directly using RGB components
    largeSphereActorA->GetProperty()->SetColor(65.0 / 255.0, 105.0 / 255.0, 225.0 / 255.0); // RoyalBlue1 color
    largeSphereActorA->GetProperty()->SetOpacity(0.3);

    vtkSmartPointer<vtkActor> smallSphereActorA = vtkSmartPointer<vtkActor>::New();
    smallSphereActorA->SetMapper(smallMapperA);
    // Set color directly using RGB components
    smallSphereActorA->GetProperty()->SetColor(50.0 / 255.0, 205.0 / 255.0, 50.0 / 255.0); // LimeGreen color

    // Create lines for the Cartesian plane
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
    renderer->AddActor(largeSphereActorA);
    renderer->AddActor(smallSphereActorA);
    renderer->AddActor(largeSphereActorB);
    renderer->AddActor(smallSphereActorB);
    renderer->AddActor(xActor);
    renderer->AddActor(yActor);
    renderer->AddActor(zActor);

    // Create a render window and set its size
    vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
    renWin->AddRenderer(renderer);
    renWin->SetSize(600, 600);

    // Create an interactor and start the visualization
    vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    iren->SetRenderWindow(renWin);

    vtkSmartPointer<TimerCallback> timerCallback = vtkSmartPointer<TimerCallback>::New();
    timerCallback->SetDynamicActor(smallSphereActorB);
    iren->AddObserver(vtkCommand::TimerEvent, timerCallback);
    iren->CreateRepeatingTimer(100); // Create a timer that triggers every 100 milliseconds

    iren->Start();

    return 0;
}
