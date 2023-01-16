import matplotlib.pyplot as plt
import numpy as np

# Create random data.
# In your solution, you would provide your own xs, ys, and zs data.
xs = [874,1184,1184,1490,1490,1800,1800,1800,2050,2050,2050,2050,2350,2350,2350,1890,1890,2500,2500,2500] #distance
ys = [107, 102, 111, 102, 111, 111, 102, 106, 106, 106, 111, 102, 102, 108, 111, 109, 102, 102, 108, 105] #angle
zs = [170, 162, 200, 172, 195, 203, 172, 182, 198, 185, 200, 175, 190, 220, 220, 188, 170, 200, 210, 210] #speed


# plot raw data
plt.figure()
ax = plt.subplot(111, projection='3d')
ax.scatter(xs, ys, zs, color='b')

# do fit
tmp_A = []
tmp_b = []
for i in range(len(xs)):
    tmp_A.append([xs[i], ys[i], 1])
    tmp_b.append(zs[i])
b = np.matrix(tmp_b).T
A = np.matrix(tmp_A)

# Manual solution
fit = (A.T * A).I * A.T * b
errors = b - A * fit
residual = np.linalg.norm(errors)

# Or use Scipy
# from scipy.linalg import lstsq
# fit, residual, rnk, s = lstsq(A, b)

print("solution: %f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
print("errors: \n", errors)
print("residual:", residual)

# plot plane
xlim = ax.get_xlim()
ylim = ax.get_ylim()
X,Y = np.meshgrid(np.arange(xlim[0], xlim[1]),
                  np.arange(ylim[0], ylim[1]))
Z = np.zeros(X.shape)
for r in range(X.shape[0]):
    for c in range(X.shape[1]):
        Z[r,c] = fit[0] * X[r,c] + fit[1] * Y[r,c] + fit[2]
ax.plot_wireframe(X,Y,Z, color='k')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()