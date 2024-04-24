#include <dlib/svm.h>
#include <dlib/rand.h>

using namespace dlib;

int main()
{
    // Define data properties
    const int numFeatures = 2;
    const int numSamplesPerClass = 50;
    const int numClasses = 2;

    // Generate sample data (replace with your actual data generation method)
    std::vector<matrix<double>> features;
    std::vector<long> labels;

    // ... (fill features and labels with your data)

    // Train the KNN classifier using dlib's knn_classifier (not directly KNN)
    knn_classifier<matrix<double>, long> knn;
    knn.set_distance_metric(linear_kernel());
    knn.set_k(5);
    knn.train(features, labels);

    // Classify a new data point (example)
    matrix<double> newData(1, numFeatures);
    newData << 0.5, 0.0; // Replace with your new data point

    // Predict the class label for the new data point
    long predictedLabel = knn(newData);

    std::cout << "Predicted class: " << predictedLabel << std::endl;

    return 0;
}
