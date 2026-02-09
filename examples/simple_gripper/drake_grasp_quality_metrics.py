'''

Example parsed data from Drake simulation:
input_data=
[
# Contact One
{'F_c_W': array([ 0.791613, 10.0232 , -1.67203 ]),
'p_WC_W': array([ 0.201146 , -0.0286318, -0.014975 ])},

# Contact Two
{'F_c_W': array([ -0.501686, -10.1641 , -1.69414 ]),
'p_WC_W': array([ 0.205468 , 0.0287383, -0.0143726])}],

# Object's COM
p_WO_W=[ 0.259356 0.094741 -0.0228324]

'''

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as mpatches
import matplotlib.cm as cm


class DrakeGraspQualityMetrics:
    def __init__(self, input_data, p_WO_W, mu=0.5, gamma=0.1, soft_finger_contact=True, plot_results=True):
        self.input_data = input_data
        self.p_WO_W = np.array(p_WO_W)
        self.mu = mu
        self.gamma = gamma
        self.soft_finger_contact = soft_finger_contact
        self.contact_data = None
        self.plot_results = plot_results

    def compute_grasp_metrics(self):
        self._prepare_data()
        epsilon_metric_linear, epsilon_metric_rotational = self._compute_epsilon_metrics()
        grasp_matrix_min_singular_value_metric = self._compute_grasp_matrix_min_singular_value_metric()
        distance_centroid_COM_metric = self._compute_distance_centroid_COM_metric()

        return (epsilon_metric_linear, epsilon_metric_rotational,
                grasp_matrix_min_singular_value_metric, distance_centroid_COM_metric)

    # High level functions for compute_grasp_metrics

    def _prepare_data(self):
        self.contact_data = []
        for input_contact in self.input_data:
            contact = {
                'F_c': input_contact['F_c_W'],
                'p_OC_W': input_contact['p_WC_W'] - self.p_WO_W
            }
            F_c_W = contact['F_c']
            x_c, z_c = self._calculate_contact_frame(F_c_W)
            y_c = self._optimize_y_c(x_c, z_c)
            contact['x_c'], contact['y_c'], contact['z_c'] = x_c, y_c, z_c
            self.contact_data.append(contact)

        if self.plot_results:
            fig = plt.figure(figsize=(12, 6))
            ax1 = fig.add_subplot(121, projection='3d')
            ax2 = fig.add_subplot(122, projection='3d')

            colors = ['r', 'b']
            max_force_magnitude = 0

            for i, contact in enumerate(self.contact_data):
                for ax, vectors in [(ax1, [('x_c', '-'), ('y_c', '--'), ('z_c', ':')]),
                                    (ax2, [('F_c', '-')])]:
                    for key, style in vectors:
                        ax.quiver(0, 0, 0, *contact[key], color=colors[i], linestyle=style, label=f'{key}{i + 1}')

                # Calculate the magnitude of F_c
                force_magnitude = np.linalg.norm(contact['F_c'])
                if force_magnitude > max_force_magnitude:
                    max_force_magnitude = force_magnitude

            # Set axis limits for ax1
            ax1.set_xlim([-1, 1])
            ax1.set_ylim([-1, 1])
            ax1.set_zlim([-1, 1])

            # Set axis limits for ax2 based on max_force_magnitude
            limit = max_force_magnitude * 1.1  # Add 10% margin
            ax2.set_xlim([-limit, limit])
            ax2.set_ylim([-limit, limit])
            ax2.set_zlim([-limit, limit])

            for ax in [ax1, ax2]:
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.legend()

            ax1.set_title('Contact Frames')
            ax2.set_title('Force Vectors')

            plt.tight_layout()
            plt.show()


    def _compute_epsilon_metrics(self):
        # Calculate forces and moments
        linear_forces, moments_from_linear_forces, moments_from_formula = self.calculate_linear_forces_and_moments(self.contact_data,
                                                                                                                   self.mu,
                                                                                                                   self.gamma)

        # Print forces and moments
        self.print_forces_and_moments(linear_forces, moments_from_linear_forces, moments_from_formula)

        # Visualize forces and moments
        if self.plot_results:
            self.visualize_forces_and_moments(self.contact_data, linear_forces, moments_from_linear_forces,
                                              moments_from_formula)

        # Calculate and visualize GWS
        return self.calculate_and_visualize_gws(linear_forces, moments_from_linear_forces, moments_from_formula)

    def _compute_grasp_matrix_min_singular_value_metric(self):
        if len(self.contact_data) != 2:
            raise ValueError("This implementation supports exactly two contact points.")

        def skew(v):
            return np.array([[0, -v[2], v[1]],
                             [v[2], 0, -v[0]],
                             [-v[1], v[0], 0]])

        def calculate_G_i(R_ci, p_ci):
            p_ci_hat = skew(p_ci)
            basis_matrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 0],
                                     [0, 0, 0, 0],
                                     [0, 0, 0, 1]])
            G_i_full = np.vstack((np.hstack((R_ci, np.zeros((3, 3)))),
                                  np.hstack((p_ci_hat @ R_ci, R_ci))))
            return G_i_full @ basis_matrix

        # Extract R_ci and p_ci from contact_data
        R_c1 = np.column_stack((self.contact_data[0]['x_c'], self.contact_data[0]['y_c'], self.contact_data[0]['z_c']))
        R_c2 = np.column_stack((self.contact_data[1]['x_c'], self.contact_data[1]['y_c'], self.contact_data[1]['z_c']))
        p_c1 = self.contact_data[0]['p_OC_W']
        p_c2 = self.contact_data[1]['p_OC_W']

        G = np.hstack((calculate_G_i(R_c1, p_c1), calculate_G_i(R_c2, p_c2)))
        singular_values = np.linalg.svd(G, compute_uv=False)
        threshold = 1e-10
        rounded_singular_values = np.where(np.abs(singular_values) < threshold, 0, singular_values)
        return np.min(rounded_singular_values)

    def _compute_distance_centroid_COM_metric(self):
        contact_positions = [contact['p_WC_W'] for contact in self.input_data]
        centroid = np.mean(contact_positions, axis=0)
        distance = np.linalg.norm(centroid - self.p_WO_W)
        return distance

    # Low level functions for _prepare_data
    def _calculate_contact_frame(self, F_c_W):
        z_c = F_c_W / np.linalg.norm(F_c_W)
        world_z = np.array([0, 0, 1])
        x_c = np.cross(world_z, z_c)
        x_c = x_c / np.linalg.norm(x_c)
        return x_c, z_c

    def _optimize_y_c(self, x_c, z_c):
        def objective(y_c):
            return np.linalg.norm(np.cross(x_c, y_c) - z_c)

        def norm_constraint(y_c):
            return np.sum(y_c ** 2) - 1

        def angle_between(v1, v2):
            return np.degrees(np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0)))

        def termination_condition(y_c):
            error = np.linalg.norm(np.cross(x_c, y_c) - z_c)
            angles = [angle_between(x_c, y_c), angle_between(x_c, z_c), angle_between(y_c, z_c)]
            #print(f"Current error: {error}, Angles: {angles}")
            return all(abs(angle - 90.0) < 0.01 for angle in angles) and error < 1e-11

        result = minimize(
            objective, np.random.rand(3),
            constraints=[{'type': 'eq', 'fun': norm_constraint}],
            options={'maxiter': 30000, 'ftol': 1e-6},
            callback=termination_condition
        )

        if result.success:
            return result.x / np.linalg.norm(result.x)
        else:
            raise ValueError("Optimization failed to find y_c")

    # Low level functions for _compute_epsilon_metrics

    def calculate_linear_forces(self, F_c, x_c, y_c, mu):
        # Ensure z_c is correctly computed
        z_c = np.cross(x_c, y_c)
        z_c = z_c / np.linalg.norm(z_c)

        # Decompose F_c into normal and tangential components
        F_c_normal_magnitude = np.dot(F_c, z_c)

        if F_c_normal_magnitude <= 0:
            raise ValueError("Normal component of contact force must be positive.")

        F_c_normal = F_c_normal_magnitude * z_c

        # Compute frictional force magnitude
        friction_magnitude = mu * F_c_normal_magnitude

        # Number of directions (32 in this case)
        num_directions = 256
        directions = []

        for i in range(num_directions):
            theta = (i * 2 * np.pi) / num_directions  # Angle in radians
            d = np.cos(theta) * x_c + np.sin(theta) * y_c
            d = d / np.linalg.norm(d)  # Ensure the direction vector is normalized
            directions.append(d)

        # Compute total contact forces at the boundaries of the friction cone
        total_contact_forces = [F_c_normal + friction_magnitude * d for d in directions]

        return np.array(total_contact_forces)

    def calculate_moment(self, position, force):
        return np.cross(position, force)

    def calculate_moments_from_formula(self, contact, gamma, F_c_norm, soft_finger=False):
        x_c = np.array(contact['x_c'])
        y_c = np.array(contact['y_c'])
        z_c = np.array(contact['z_c'])

        if soft_finger:
            moments = {
                'M_z_1': z_c * gamma * F_c_norm,
                'M_z_2': -z_c * gamma * F_c_norm
            }
        else:
            moments = {
                'M_x_1': x_c * gamma * F_c_norm,
                'M_x_2': -x_c * gamma * F_c_norm,
                'M_y_1': y_c * gamma * F_c_norm,
                'M_y_2': -y_c * gamma * F_c_norm,
                'M_z_1': z_c * gamma * F_c_norm,
                'M_z_2': -z_c * gamma * F_c_norm
            }

        return moments

    def calculate_linear_forces_and_moments(self, contact_data, mu, gamma):
        linear_forces = {}
        moments_from_linear_forces = {}
        moments_from_formula = {}

        for i, contact in enumerate(contact_data, 1):
            position = np.array(contact['p_OC_W'])
            x_c = np.array(contact['x_c'])
            y_c = np.array(contact['y_c'])
            F_c = np.array(contact['F_c'])

            forces = self.calculate_linear_forces(F_c, x_c, y_c, mu)
            linear_forces[f'contact_{i}'] = forces

            moments = [self.calculate_moment(position, force) for force in forces]
            moments_from_linear_forces[f'contact_{i}'] = {
                f'M_F{j + 1}': moment for j, moment in enumerate(moments)
            }

            F_c_norm = np.linalg.norm(F_c)
            formula_moments = self.calculate_moments_from_formula(contact, gamma, F_c_norm,
                                                                  soft_finger=self.soft_finger_contact)
            moments_from_formula[f'contact_{i}'] = formula_moments

        return linear_forces, moments_from_linear_forces, moments_from_formula

    def point_in_hull(self, point, hull, tolerance=1e-12):
        return all((np.dot(eq[:-1], point) + eq[-1] <= tolerance) for eq in hull.equations)

    def calculate_epsilon_radius(self, hull):
        distances = []
        for eq in hull.equations:
            normal = eq[:-1]
            offset = eq[-1]
            distance = abs(offset) / np.linalg.norm(normal)
            distances.append(distance)
        return min(distances)

    def calculate_closest_point(self, hull):
        distances = []
        closest_points = []

        for eq in hull.equations:
            normal = eq[:-1]
            offset = eq[-1]
            distance = abs(offset) / np.linalg.norm(normal)
            closest_point = normal * (distance / np.linalg.norm(normal))
            distances.append(distance)
            closest_points.append(closest_point)

        closest_facet = np.argmin(distances)
        return closest_points[closest_facet]

    def visualize_hull(self, ax, points, hull, epsilon_radius, title):
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o', label=f'{title} Vectors')

        for simplex in hull.simplices:
            face = points[simplex]
            ax.add_collection3d(Poly3DCollection([face], alpha=0.2, facecolor='r', edgecolor='k'))

        ax.scatter([0], [0], [0], c='r', marker='o', s=100, label='Origin')

        if epsilon_radius > 0:
            closest_point = self.calculate_closest_point(hull)
            ax.plot([0, closest_point[0]], [0, closest_point[1]], [0, closest_point[2]],
                    'g-', linewidth=2, label='Epsilon Radius')

            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x = epsilon_radius * np.outer(np.cos(u), np.sin(v))
            y = epsilon_radius * np.outer(np.sin(u), np.sin(v))
            z = epsilon_radius * np.outer(np.ones(np.size(u)), np.cos(v))
            ax.plot_surface(x, y, z, color='yellow', alpha=0.2)

        ax.set_xlabel('X', fontsize=20)
        ax.set_ylabel('Y', fontsize=20)
        ax.set_zlabel('Z', fontsize=20)
        ax.set_title(f'3D Convex Hull of {title}s\nEpsilon Radius: {epsilon_radius:.4f}')

        # *** KEY CHANGE 1: Tick Label Size ***
        for label in ax.get_xticklabels() + ax.get_yticklabels() + ax.get_zticklabels():
            label.set_fontsize(24)  # Adjust font size as needed

        epsilon_ball_patch = mpatches.Patch(color='yellow', alpha=0.2, label='Epsilon Ball')

        handles, labels = ax.get_legend_handles_labels()

        if epsilon_radius > 0:
            handles.append(epsilon_ball_patch)

        #ax.legend(handles=handles)
        # *** KEY CHANGE 2: Legend Placement ***
        #ax.legend(handles=handles, loc='upper right', bbox_to_anchor=(0.8, 0.9))  # Adjust bbox_to_anchor as needed
        ax.legend(handles=handles, fontsize=24, loc='upper right', bbox_to_anchor=(0.8, 0.9))

        ax.set_box_aspect((np.ptp(points[:, 0]), np.ptp(points[:, 1]), np.ptp(points[:, 2])))

    def calculate_and_visualize_gws(self, linear_forces, moments_from_linear_forces, moments_from_formula):
        all_forces = np.vstack([forces for forces in linear_forces.values()])

        all_moments = []
        for contact in moments_from_linear_forces.values():
            all_moments.extend(list(contact.values()))
        for contact in moments_from_formula.values():
            all_moments.extend(list(contact.values()))
        all_moments = np.array(all_moments)

        force_hull = ConvexHull(all_forces)
        moment_hull = ConvexHull(all_moments)

        origin = np.zeros(3)
        origin_in_force_hull = self.point_in_hull(origin, force_hull)
        origin_in_moment_hull = self.point_in_hull(origin, moment_hull)

        force_epsilon_radius = self.calculate_epsilon_radius(force_hull) if origin_in_force_hull else 0
        moment_epsilon_radius = self.calculate_epsilon_radius(moment_hull) if origin_in_moment_hull else 0

        print(f"\nOrigin in force hull: {origin_in_force_hull}")
        print(f"Force epsilon radius: {force_epsilon_radius}")
        print(f"\nOrigin in moment hull: {origin_in_moment_hull}")
        print(f"Moment epsilon radius: {moment_epsilon_radius}")

        if self.plot_results:
            fig_force = plt.figure(figsize=(10, 8))
            ax_force = fig_force.add_subplot(111, projection='3d')
            self.visualize_hull(ax_force, all_forces, force_hull, force_epsilon_radius, "Force")
            plt.tight_layout()
            plt.show()

            fig_moment = plt.figure(figsize=(10, 8))
            ax_moment = fig_moment.add_subplot(111, projection='3d')
            self.visualize_hull(ax_moment, all_moments, moment_hull, moment_epsilon_radius, "Moment")
            plt.tight_layout()
            plt.show()

        return force_epsilon_radius, moment_epsilon_radius

    def visualize_forces_and_moments(self, contact_data, linear_forces, moments_from_linear_forces,
                                     moments_from_formula):
        for i, (contact, forces) in enumerate(linear_forces.items(), 1):
            fig = plt.figure(figsize=(15, 12))
            ax = fig.add_subplot(111, projection='3d')

            origin = np.array(contact_data[i - 1]['p_OC_W'])
            x_c = np.array(contact_data[i - 1]['x_c'])
            y_c = np.array(contact_data[i - 1]['y_c'])
            z_c = np.array(contact_data[i - 1]['z_c'])
            F_c = np.array(contact_data[i - 1]['F_c'])

            ax.scatter(*origin, color='k', s=100)

            scale = 0.1  # Scale factor for coordinate system arrows
            ax.quiver(*origin, *(scale * x_c), color='r', arrow_length_ratio=0.1, label='x_c')
            ax.quiver(*origin, *(scale * y_c), color='g', arrow_length_ratio=0.1, label='y_c')
            ax.quiver(*origin, *(scale * z_c), color='b', arrow_length_ratio=0.1, label='z_c')

            # Plot F_c
            ax.quiver(*origin, *F_c, color='m', arrow_length_ratio=0.1, linewidth=2, label='F_c')

            # Plot linear forces using a colormap
            num_forces = len(forces)
            cmap = cm.get_cmap('viridis', num_forces)
            for j, force in enumerate(forces):
                color = cmap(j)
                ax.quiver(*origin, *force, color=color, arrow_length_ratio=0.1, label=f'F{j + 1}')

            # Plot moments from linear forces
            moment_colors = ['c', 'm', 'y', 'k', 'orange', 'purple', 'brown', 'pink']
            linear_force_moments = moments_from_linear_forces[contact]
            for k, (moment_name, moment_value) in enumerate(linear_force_moments.items()):
                color = moment_colors[k % len(moment_colors)]
                ax.quiver(*origin, *moment_value, color=color, linestyle='--', linewidth=2,
                          arrow_length_ratio=0.15, label=moment_name)

                # Add text label near the tip of the vector
                tip = origin + moment_value
                ax.text(tip[0], tip[1], tip[2], moment_name, fontsize=48,
                        verticalalignment='bottom', horizontalalignment='right')

            # Plot moments from formula
            formula_moment_colors = ['r', 'g', 'b', 'c', 'y', 'm', 'k', 'orange']
            formula_moments = moments_from_formula[contact]
            for l, (moment_name, moment_value) in enumerate(formula_moments.items()):
                color = formula_moment_colors[l % len(formula_moment_colors)]
                ax.quiver(*origin, *moment_value, color=color, linestyle=':', linewidth=2,
                          arrow_length_ratio=0.15, label=f'{moment_name} (formula)')

                # Add text label near the tip of the vector
                tip = origin + moment_value
                ax.text(tip[0], tip[1], tip[2], f'{moment_name} (formula)', fontsize=48,
                        verticalalignment='top', horizontalalignment='left')

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(f'Contact Point {i} - Forces and Moments')

            # Calculate limits based on all vectors
            all_points = np.vstack((
                origin,
                origin + scale * x_c,
                origin + scale * y_c,
                origin + scale * z_c,
                origin + F_c,
                *[origin + np.array(f) for f in forces],
                *[origin + np.array(mv) for mv in linear_force_moments.values()],
                *[origin + np.array(mv) for mv in formula_moments.values()]
            ))

            # Calculate the range of the data
            x_min, x_max = np.min(all_points[:, 0]), np.max(all_points[:, 0])
            y_min, y_max = np.min(all_points[:, 1]), np.max(all_points[:, 1])
            z_min, z_max = np.min(all_points[:, 2]), np.max(all_points[:, 2])

            # Add a small margin around the data (10% of the data range)
            x_margin = 0.1 * (x_max - x_min)
            y_margin = 0.1 * (y_max - y_min)
            z_margin = 0.1 * (z_max - z_min)

            ax.set_xlim(x_min - x_margin, x_max + x_margin)
            ax.set_ylim(y_min - y_margin, y_max + y_margin)
            ax.set_zlim(z_min - z_margin, z_max + z_margin)

            # Set the aspect ratio to be equal
            ax.set_box_aspect([x_max - x_min + 2 * x_margin,
                               y_max - y_min + 2 * y_margin,
                               z_max - z_min + 2 * z_margin])

            ax.view_init(elev=20, azim=45)

            ax.legend()
            plt.tight_layout()
            plt.show()

    def print_forces_and_moments(self, linear_forces, moments_from_linear_forces, moments_from_formula):
        for contact, forces in linear_forces.items():
            print(f"{contact}:")
            for i, force in enumerate(forces, 1):
                print(f"  F{i}: {force}")
            print()

        print("Moments from linear forces:")
        for contact, moments in moments_from_linear_forces.items():
            print(f"{contact} moments:")
            for moment_name, moment_value in moments.items():
                print(f"  {moment_name}: {moment_value}")
            print()

        print("Moments from formula:")
        for contact, moments in moments_from_formula.items():
            print(f"{contact} moments:")
            for moment_name, moment_value in moments.items():
                print(f"  {moment_name}: {moment_value}")
            print()

    # Plot functions:

    def _print_contact_data(self):
        print("\nCopy-paste friendly format for contact_data:")
        print("contact_data = [")
        for contact in self.contact_data:
            print("    {")
            for key in ['x_c', 'y_c', 'z_c', 'F_c', 'p_OC_W']:
                print(f"        '{key}': {contact[key].tolist()},")
            print("    },")
        print("]")


