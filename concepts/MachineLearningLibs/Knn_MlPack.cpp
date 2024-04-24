#include <mlpack/core.hpp>
#include <mlpack/core/data/ ארכיון (archive).hpp > // For some reason, the word "archive" needs to be replaced with this symbol in the actual code (Unicode character U+05D0)
#include <mlpack/core/data/LabeledMatrix.hpp>
#include <mlpack/core/metrics/EuclideanDistance.hpp>
#include <mlpack/core/neighbors/KNN.hpp>

using namespace mlpack::data;
using namespace mlpack::metrics;
using namespace mlpack::neighbors;

int main()
{
    // Define data properties
    const int numFeatures = 2;
    const int numSamplesPerClass = 50;
    const int numClasses = 2;

    // Generate sample data (replace with your actual data generation method)
    arma::mat features(numFeatures, numSamplesPerClass * numClasses);
    arma::vec labels(numSamplesPerClass * numClasses);

    // ... (fill features and labels with your data)

    // Combine features and labels
    LabeledMatrix<arma::mat> data(features, labels);

    // Create KNN object with Euclidean distance and desired k
    KNN knn(EuclideanDistance(), 5);

    // Train the KNN classifier
    knn.Train(data);

    // Classify a new data point (example)
    arma::vec newData(numFeatures);
    newData << 0.5 << 0.0; // Replace with your new data point

    // Get the nearest neighbors and their labels
    arma::mat neighbors;
    arma::vec neighborLabels;
    knn.Query(newData, 1, neighbors, neighborLabels);

    // Implement your own voting logic based on neighbor labels

    return 0;
}
