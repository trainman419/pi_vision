import numpy as np
import milk
n_samples = 10
features = np.random.randn(n_samples,5)
features[:n_samples/2] *= 2

k = 2
cluster_ids, centroids = milk.kmeans(features, k)
print cluster_ids
print centroids

