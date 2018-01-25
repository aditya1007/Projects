# -*- coding: utf-8 -*-
"""
Created on Mon Dec  4 17:14:44 2017

@author: adity
"""


import numpy as np
import matplotlib
matplotlib.use('Qt4Agg')

from matplotlib import pyplot as plt

from sklearn import linear_model, datasets


n_samples = 1000
n_outliers = 7
'''

X, y, coef = datasets.make_regression(n_samples=n_samples, n_features=1,
                                      n_informative=1, noise=10,
                                      coef=True, random_state=0)
'''

X = [[33.5,39.14,37.36,22.79,36.04,37.23,34.79]]
X = np.array(X)
X = np.transpose(X)

y = [[220.8,240.65,242.6,136.5,243.22,223.08,220.6]]
y = np.array(y)
y = np.transpose(y)

'''
# Add outlier data
np.random.seed(0)
X[:n_outliers] = 3 + 0.5 * np.random.normal(size=(n_outliers, 1))
y[:n_outliers] = -3 + 10 * np.random.normal(size=n_outliers)
'''
'''
X = [[33.5,39.14,37.36,22.79,36.04,37.23,34.79]]
X = np.array(X)

y = [[220.8,240.65,242.6,136.5,243.22,223.08,220.6]]
y = np.array(y)
'''
# Fit line using all data
lr = linear_model.LinearRegression()
lr.fit(X, y)

# Robustly fit linear model with RANSAC algorithm
ransac = linear_model.RANSACRegressor()
ransac.fit(X, y)
inlier_mask = ransac.inlier_mask_
outlier_mask = np.logical_not(inlier_mask)

# Predict data of estimated models
line_X = np.arange(X.min(), X.max())[:, np.newaxis]
line_y = lr.predict(line_X)
line_y_ransac = ransac.predict(line_X)


lw = 2

plt.plot(line_X, line_y_ransac,label='RANSAC regressor')