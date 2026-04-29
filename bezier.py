import numpy as np
from mpl_toolkits.mplot3d import Axes3D

class CubicBezier:
    def __init__(self, p0, p1, p2, p3):
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3

    def evaluate(self, t):
        """Evaluate the cubic Bezier curve at parameter t (0 <= t <= 1)"""
        u = 1 - t
        return (
            u**3 * self.p0
            + 3 * u**2 * t * self.p1
            + 3 * u * t**2 * self.p2
            + t**3 * self.p3
        )
    
    @staticmethod
    def coeff(t):
        return np.array([
            (1 - t) ** 3,
            3 * (1 - t) ** 2 * t,
            3 * (1 - t) * t**2,
            t**3
        ])
    
    @staticmethod
    def coeff_first_derivative(t):
        return np.array([
            -3 * (1 - t) ** 2,
            3 * (1 - t) ** 2 - 6 * (1 - t) * t,
            6 * (1 - t) * t - 3 * t**2,
            3 * t**2
        ])
    
    @staticmethod
    def coeff_second_derivative(t):
        return np.array([
            6 * (1 - t),
            -12 * (1 - t) + 6 * t,
            6 * (1 - t) - 12 * t,
            6 * t
        ])
    
class CubicBezierSpline:
    def __init__(self, control_points):
        """
        
        Parameters:
        control_points: 3D array of shape (n_segments, 4, 3) where each segment has 4 control points in 3D
        """
        self.control_points = control_points
        self.beziers = [CubicBezier(*control_points[i]) for i in range(control_points.shape[0])]

    def evaluate(self, t):
        """Evaluate the cubic Bezier spline at parameter t (0 <= t <= n_segments)"""
        n_segments = len(self.beziers)
        if t < 0 or t > n_segments:
            raise ValueError("t must be in the range [0, n_segments]")
        
        segment_index = min(int(t), n_segments - 1)
        local_t = t - segment_index
        return self.beziers[segment_index].evaluate(local_t)
    

    @classmethod
    def from_waypoints(cls, waypoints):
        """
        Create a cubic Bezier spline from a list of waypoints.
        
        Parameters:
        waypoints: 2D array of shape (n_waypoints, 3) where each row is a waypoint in 3D
        
        Returns:
        CubicBezierSpline instance
        """
        n_waypoints = len(waypoints)
        if n_waypoints < 2:
            raise ValueError("At least 2 waypoints are required to create a spline")
        
        control_points = []
        for dim in range(3):
            A = np.zeros((4 * (n_waypoints - 1), 4 * (n_waypoints - 1)))
            b = np.zeros(4 * (n_waypoints - 1))

            i = 0
            while i < len(A):
                A[i, i:i+4] = CubicBezier.coeff(0)
                b[i] = waypoints[i // 4][dim]
                A[i + 1, i:i+4] = CubicBezier.coeff(1)
                b[i + 1] = waypoints[i//4 + 1][dim]
                if i+2 < len(A) - 2:
                    A[i + 2, i:i+4] = CubicBezier.coeff_first_derivative(1)
                    A[i + 2, (i+4):(i+4)+4] = -CubicBezier.coeff_first_derivative(0)
                    b[i + 2] = 0
                    
                    A[i+3, i:i+4] = CubicBezier.coeff_second_derivative(1)
                    A[i+3, i+4:i+4+4] = -CubicBezier.coeff_second_derivative(0)
                    b[i + 3] = 0
                i += 4    
            A[-1, :4] = CubicBezier.coeff_first_derivative(0)
            b[-1] = 0
            A[-2, -4:] = CubicBezier.coeff_first_derivative(1)
            b[-2] = 0
            control_points.append(np.linalg.solve(A, b).reshape(-1, 4))
        
        control_points = np.stack(control_points, axis=-1)  # shape (n_segments, 4, 3)
        
        return cls(control_points)
    
if __name__ == "__main__":
    example_waypoints = np.array([
        [0, 0, 0],
        [1, 2, 1],
        [2, 0, 2],
        [3, 1, 3]
    ])
    spline = CubicBezierSpline.from_waypoints(example_waypoints)
    t_values = np.linspace(0, len(spline.beziers), 100)
    points = np.array([spline.evaluate(t) for t in t_values])
    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(points[:, 0], points[:, 1], points[:, 2], 'b-', label='Bezier Spline')
    ax.scatter(example_waypoints[:, 0], example_waypoints[:, 1], example_waypoints[:, 2], c='r', s=100, label='Waypoints')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()