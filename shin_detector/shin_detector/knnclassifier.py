import numpy as np
from sklearn.neighbors import KNeighborsClassifier

# Read in the negative features file
with open('shin_detector/Negative_Features.txt', 'r') as f:
    negative_features = [list(map(np.float64, line.strip().split(','))) for line in f.readlines()]

# Read in the positive features file
with open('shin_detector/Positive_Features.txt', 'r') as f:
    positive_features = [list(map(np.float64, line.strip().split(','))) for line in f.readlines()]

# Combine the positive and negative features into a single array
features = np.concatenate([negative_features, positive_features])

# Create the labels for the features (0 for negative, 1 for positive)
labels = np.concatenate([np.zeros(len(negative_features)), np.ones(len(positive_features))])

# Define the number of nearest neighbors to use
k = 3

# Create the K-nearest neighbor classifier
knn = KNeighborsClassifier(n_neighbors=k)

# Train the classifier on the features and labels
knn.fit(features, labels)

# Define the object to classify
object_to_classify = [14, 0.11291712961462473, 0.10547244660027122, 0.15513897950623567, 1.3440719874268938]

# Predict the class of the object
predicted_class = knn.predict([object_to_classify])[0]

# Output the predicted class
if predicted_class == 0:
    print('The object is classified as negative.')
else:
    print('The object is classified as positive.')
