#include "KnnNode.h"

#include <shark/Core/Random.h>
#include <shark/Models/Kernels/NearestNeighborClassifier.h> // Change this line
#include <shark/Data/Dataset.h>
#include <shark/Data/DenseData.h>
#include <shark/Data/Labels.h>
#include <shark/ObjectiveFunctions/ClassificationLoss.h>
#include <shark/Data/ClassificationDataset.h>

// Define data structure
typedef shark::DenseData<double> FeatureType;
typedef shark::Label TargetType;

void ClassifyTest()
{
    // Set up random number generation
    shark::Rng<double> rng;

    // Define data properties
    const int numFeatures = 2;
    const int numSamplesPerClass = 50;
    const int numClasses = 2;

    // Generate sample data for two classes
    std::vector<FeatureType> featuresClass1;
    std::vector<TargetType> labelsClass1;
    for (int i = 0; i < numSamplesPerClass; ++i)
    {
        FeatureType feature(numFeatures);
        feature(0) = 1.0 + rng.uniform(-0.5, 0.5);
        feature(1) = 1.0 + rng.uniform(-0.5, 0.5);
        featuresClass1.push_back(feature);
        labelsClass1.push_back(0); // Label for class 1
    }

    std::vector<FeatureType> featuresClass2;
    std::vector<TargetType> labelsClass2;

    for (int i = 0; i < numSamplesPerClass; ++i)
    {
        FeatureType feature(numFeatures);
        feature(0) = -1.0 + rng.uniform(-0.5, 0.5);
        feature(1) = -1.0 + rng.uniform(-0.5, 0.5);
        featuresClass2.push_back(feature);
        labelsClass2.push_back(1); // Label for class 2
    }

    // Combine data and labels
    std::vector<FeatureType> allFeatures;
    std::vector<TargetType> allLabels;
    allFeatures.insert(allFeatures.end(), featuresClass1.begin(), featuresClass1.end());
    allFeatures.insert(allFeatures.end(), featuresClass2.begin(), featuresClass2.end());
    allLabels.insert(allLabels.end(), labelsClass1.begin(), labelsClass1.end());
    allLabels.insert(allLabels.end(), labelsClass2.begin(), labelsClass2.end());

    // Create dataset and KNN classifier
    shark::ClassificationDataset data = shark::ClassificationDataset(allFeatures, allLabels);
    const int numNeighbors = 5; // Adjust this parameter as needed
    shark::KNearestNeighbors<FeatureType, TargetType> knn(numNeighbors);

    // Train the KNN classifier
    knn.train(data);

    // Classify a new data point (example)
    FeatureType newData(numFeatures);
    newData(0) = 0.5;
    newData(1) = 0.0;

    // Predict the class label for the new data point
    TargetType predictedLabel = knn.classify(newData);

    // Evaluate the prediction (optional)
    if (predictedLabel == 0)
    {
        std::cout << "Predicted class: 1" << std::endl;
    }
    else
    {
        std::cout << "Predicted class: 2" << std::endl;
    }
}
