import pandas as pd
import numpy as np
import open3d as o3d
import copy
import argparse
import os
from scipy.spatial.transform import Rotation as R

def predict_robotiq140_gripper_grasp_point_height(cgn_gripper_width):
    # cgn_gripper_width is in meters, convert to mm
    x_mm = cgn_gripper_width * 1000
    height_mm = 229.644116 + 0.004132 * x_mm - 0.000725 * x_mm**2 - 0.000004 * x_mm**3
    # Convert back to meters
    return height_mm / 1000  # Returns height in meters

def get_rotation_matrix_from_two_vectors(vec1, vec2):
    """
    Compute the rotation matrix that aligns vec1 to vec2.
    """
    a = vec1 / np.linalg.norm(vec1)
    b = vec2 / np.linalg.norm(vec2)

    v = np.cross(a, b)
    c = np.dot(a, b)
    if c < -0.999999:
        # Vectors are nearly opposite
        # Find a vector orthogonal to vec1
        orthogonal = np.array([1, 0, 0]) if np.abs(a[0]) < 0.1 else np.array([0, 1, 0])
        v = np.cross(a, orthogonal)
        v /= np.linalg.norm(v)
        return R.from_rotvec(np.pi * v).as_matrix()
    elif c > 0.999999:
        # Vectors are nearly the same
        return np.eye(3)
    else:
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat @ kmat * ((1 - c) / (s ** 2))
        return rotation_matrix

def create_cylinder_between_points(p1, p2, radius=0.0004, color=[1, 0, 0]):
    """
    Create a cylinder mesh between two points p1 and p2.
    The cylinder is correctly aligned and connected between p1 and p2.
    """
    # Compute the vector from p1 to p2
    v = p2 - p1
    # Compute the length of the cylinder
    height = np.linalg.norm(v)
    if height == 0:
        return None
    # Create a cylinder along z-axis centered at (0, 0, height / 2)
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
    cylinder.paint_uniform_color(color)
    # Move the cylinder so that its base is at (0, 0, 0)
    cylinder.translate([0, 0, height / 2])
    # Compute rotation matrix to align z-axis with vector v
    z_axis = np.array([0, 0, 1])
    v_normalized = v / height
    rotation_axis = np.cross(z_axis, v_normalized)
    rotation_angle = np.arccos(np.clip(np.dot(z_axis, v_normalized), -1.0, 1.0))
    if np.linalg.norm(rotation_axis) < 1e-6:
        # Vectors are parallel
        if np.dot(z_axis, v_normalized) > 0:
            R_mat = np.eye(3)
        else:
            R_mat = R.from_euler('x', 180, degrees=True).as_matrix()
    else:
        rotation_axis_normalized = rotation_axis / np.linalg.norm(rotation_axis)
        R_mat = R.from_rotvec(rotation_angle * rotation_axis_normalized).as_matrix()
    # Rotate the cylinder
    cylinder.rotate(R_mat, center=(0, 0, 0))
    # Translate the cylinder to p1
    cylinder.translate(p1)
    return cylinder

def create_gripper_geometry(gripper_opening=0.02, gripper_length=0.127, color=[0, 0, 1], radius=0.0004):
    """
    Creates a gripper geometry represented as a series of cylinders.
    Returns the cylinders and contact points in local coordinates.
    """
    # Calculate key points based on the tuning fork structure
    L = gripper_length
    L_handle = L / 2
    W = gripper_opening

    # Define the points in the local coordinate frame
    P0 = np.array([0, 0, 0])                  # Base of the gripper handle
    P1 = np.array([0, 0, L_handle])           # Top of the handle / center
    P2 = np.array([-W/2, 0, L_handle])        # Left end of the horizontal bar
    P3 = np.array([W/2, 0, L_handle])         # Right end of the horizontal bar
    P4 = np.array([-W/2, 0, L])               # Top of the left prong (contact point)
    P5 = np.array([W/2, 0, L])                # Top of the right prong (contact point)

    cylinders = []

    # Connections to match the original LineSet
    # Handle: P0 to P1
    cylinder = create_cylinder_between_points(P0, P1, radius=radius, color=color)
    if cylinder is not None:
        cylinders.append(cylinder)

    # Left side of horizontal bar: P1 to P2
    cylinder = create_cylinder_between_points(P1, P2, radius=radius, color=color)
    if cylinder is not None:
        cylinders.append(cylinder)

    # Right side of horizontal bar: P1 to P3
    cylinder = create_cylinder_between_points(P1, P3, radius=radius, color=color)
    if cylinder is not None:
        cylinders.append(cylinder)

    # Left prong: P2 to P4
    cylinder = create_cylinder_between_points(P2, P4, radius=radius, color=color)
    if cylinder is not None:
        cylinders.append(cylinder)

    # Right prong: P3 to P5
    cylinder = create_cylinder_between_points(P3, P5, radius=radius, color=color)
    if cylinder is not None:
        cylinders.append(cylinder)

    # Return cylinders and contact points
    contact_points_local = [P4, P5]  # Left and right prongs
    return cylinders, contact_points_local

def transform_gripper(cylinders, position, orientation):
    """
    Applies the given position and orientation to the gripper cylinders.
    Returns the list of transformed cylinders and the transformation matrix.
    """
    rotation = R.from_quat(orientation).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = position
    transformed_cylinders = []
    for cylinder in cylinders:
        cylinder_transformed = copy.deepcopy(cylinder)
        cylinder_transformed.transform(T)
        transformed_cylinders.append(cylinder_transformed)
    return transformed_cylinders, T

def transform_points(points_local, position, orientation):
    """
    Transforms a list of points from local to world coordinates.
    """
    rotation = R.from_quat(orientation).as_matrix()
    points_world = [rotation @ point + position for point in points_local]
    return points_world

def ray_cast_from_point(point_world, dir_world, scene):
    """
    Casts a ray from the given point in the given direction and finds the intersection with the mesh.
    Returns the intersection point and the normal at that point.
    """
    rays = o3d.core.Tensor([[point_world[0], point_world[1], point_world[2],
                             dir_world[0], dir_world[1], dir_world[2]]], dtype=o3d.core.Dtype.Float32)

    ans = scene.cast_rays(rays)
    hit = ans['t_hit'].numpy()[0]
    if np.isinf(hit):
        # No intersection
        return None, None
    else:
        # Intersection point
        intersection = point_world + dir_world * hit
        # Get the normal at the intersection point
        normal = ans['primitive_normals'].numpy()[0]
        normal /= np.linalg.norm(normal)
        return intersection, normal

def visualize_force_vectors(intersections, normals, color, length=0.2, radius=0.0003):
    """
    Creates cylinder objects to represent force vectors from intersection points.
    """
    force_cylinders = []
    for intersection, normal in zip(intersections, normals):
        if intersection is not None and normal is not None:
            end_point = intersection + normal * length
            cylinder = create_cylinder_between_points(intersection, end_point, radius=radius, color=color)
            if cylinder is not None:
                force_cylinders.append(cylinder)
    return force_cylinders

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Gripper Visualization')
    parser.add_argument('--ids', type=str, help='Comma-separated list of grasp IDs to visualize')
    parser.add_argument('--VizPercentage', type=float, default=100.0,
                        help='Percentage of grasps to visualize (default: 100)')
    parser.add_argument('--RandomSeed', type=int, default=42,
                        help='Random seed for grasp selection (default: 42)')
    parser.add_argument('--NoApproach', action='store_true',
                        help='Do not visualize approach poses')
    parser.add_argument('--input_csv', type=str, default='object_1_UOGPLog_heightCorrected.csv',
                        help='Path to input CSV file (default: object_1_UOGPLog_heightCorrected.csv)')
    parser.add_argument('--mesh_path', type=str, default='./CoconutMilkCan.obj',
                        help='Path to object mesh file (default: ./CoconutMilkCan.obj)')
    args = parser.parse_args()

    # Input CSV file path
    input_csv_path = args.input_csv
    # Output CSV file path (append '_withCGNOnlyForceAnalysis' to the input filename)
    base_name, ext = os.path.splitext(input_csv_path)
    output_csv_path = f"{base_name}_withCGNOnlyForceAnalysis{ext}"

    df = pd.read_csv(input_csv_path, sep=';', engine='python')

    # Extract position and orientation from the new CSV format
    df['position'] = df[['position_x', 'position_y', 'position_z']].values.tolist()
    df['orientation'] = df[['orientation_x', 'orientation_y', 'orientation_z', 'orientation_w']].values.tolist()

    # Filter the DataFrame based on provided IDs
    grasp_ids = df['graspID'].unique()

    if args.ids:
        id_list = [int(x.strip()) for x in args.ids.split(',')]
        grasp_ids = np.intersect1d(grasp_ids, id_list)
        df = df[df['graspID'].isin(grasp_ids)]

    # Apply VizPercentage filtering
    if args.VizPercentage < 100.0:
        np.random.seed(args.RandomSeed)
        n_select = int(len(grasp_ids) * (args.VizPercentage / 100.0))
        if n_select < 1:
            n_select = 1
        selected_grasp_ids = np.random.choice(grasp_ids, size=n_select, replace=False)
        grasp_ids = selected_grasp_ids
        df = df[df['graspID'].isin(grasp_ids)]

    # **New Code Start: Filter out rows that are not actual grasps**
    # Only keep rows where matrix_name == 'X_Drake_Grasp'
    df = df[df['matrix_name'] == 'X_Drake_Grasp']
    # **New Code End**

    # Create a coordinate frame for reference
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

    # Mesh file path
    mesh_path = args.mesh_path
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    if not mesh.has_triangle_normals():
        mesh.compute_triangle_normals()
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.7, 0.7, 0.7])  # Light gray

    # Verify that the mesh is loaded
    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        print("Error: Mesh is empty. Please check the mesh file path and format.")
        return

    # Compute the object's COM
    object_com = mesh.get_center()

    geometries = []

    # Create RaycastingScene once
    scene = o3d.t.geometry.RaycastingScene()
    mesh_id = scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

    # Initialize list to store failed grasps and results
    failed_grasps = []
    results = []

    # Group the DataFrame by graspID
    grasp_groups = df.groupby('graspID')

    # Prepare gripper data
    for grasp_id, group in grasp_groups:
        # Assign a unique color for this graspID
        color = np.random.rand(3)

        # Process each pose (grasp) for this graspID
        for idx, row in group.iterrows():
            # Check if the row corresponds to an approach pose
            matrix_name = row.get('matrix_name', 'X_Drake_Grasp')
            # **We already filtered out non-grasp rows, so this check is redundant**
            # if args.NoApproach and matrix_name == 'X_Drake_Grasp_approach':
            #     continue  # Skip approach pose if --NoApproach is specified

            position = np.array(row['position'])
            orientation = np.array(row['orientation'])

            if position is not None and orientation is not None:
                # Get gripper opening and score from CSV
                gripper_opening = row['gripper_opening'] if 'gripper_opening' in row else 0.08  # in meters
                score = row['score'] if 'score' in row else None  # Get score if available

                # Compute gripper length based on gripper opening
                gripper_length = predict_robotiq140_gripper_grasp_point_height(gripper_opening)
                # Adjust for fingerpad length
                robotiq_140_fingerpad_length = 0.0655  # in meters
                gripper_length += robotiq_140_fingerpad_length / 2.0  # Add half of fingerpad length

                # Create gripper geometry and get contact points in local coordinates
                gripper_cylinders, contact_points_local = create_gripper_geometry(
                    gripper_opening=gripper_opening,
                    gripper_length=gripper_length,
                    color=color,
                    radius=0.0004  # Adjusted radius
                )

                # Transform the gripper geometry
                gripper_transformed_cylinders, T = transform_gripper(
                    gripper_cylinders, position, orientation
                )
                geometries.extend(gripper_transformed_cylinders)

                # Transform contact points to world coordinates
                contact_points_world = transform_points(contact_points_local, position, orientation)

                # Output the contact points Z-coordinate
                print(f"Grasp ID {grasp_id}, Row {idx}:")
                for i, cp in enumerate(contact_points_world):
                    print(f"  Contact Point {i+1} Z-coordinate:  {cp[2]}")

                # Define directions in local frame (positive and negative X-direction)
                dirs_local = [np.array([1, 0, 0]), np.array([-1, 0, 0])]

                # Collect all intersections and normals
                intersections = []
                normals = []
                for cp_world in contact_points_world:
                    for dir_local in dirs_local:
                        # Transform direction to world coordinates
                        rotation = R.from_quat(orientation)
                        dir_world = rotation.apply(dir_local)
                        # Ray cast from contact point in dir_world
                        intersection, normal = ray_cast_from_point(cp_world, dir_world, scene)
                        if intersection is not None and normal is not None:
                            intersections.append(intersection)
                            normals.append(normal)

                # Prepare result dictionary
                result = {
                    'id': grasp_id,
                    'position_x': position[0],
                    'position_y': position[1],
                    'position_z': position[2],
                    'orientation_x': orientation[0],
                    'orientation_y': orientation[1],
                    'orientation_z': orientation[2],
                    'orientation_w': orientation[3],
                    'gripper_opening': gripper_opening,
                    'score': score,
                    'object_com_x': object_com[0],
                    'object_com_y': object_com[1],
                    'object_com_z': object_com[2],
                    'status': 'SUCCESS'
                }

                # Check if we have at least one unique intersection
                if len(intersections) < 1:
                    print(f"Grasp ID {grasp_id}, Row {idx}: Grasp failed due to missing contact points.")
                    failed_grasps.append((grasp_id, idx))  # Store graspID and row index
                    result['status'] = 'FAILED'
                    results.append(result)
                    continue

                # Find unique intersections based on positions
                unique_intersections = []
                unique_normals = []
                for i, (inter, norm) in enumerate(zip(intersections, normals)):
                    is_unique = True
                    for u_inter in unique_intersections:
                        if np.linalg.norm(inter - u_inter) < 1e-4:  # Threshold for considering points the same
                            is_unique = False
                            break
                    if is_unique:
                        unique_intersections.append(inter)
                        unique_normals.append(norm)

                num_unique = len(unique_intersections)
                if num_unique < 1 or num_unique > 2:
                    print(f"Grasp ID {grasp_id}, Row {idx}: Grasp failed due to incorrect number of contact points ({num_unique}).")
                    failed_grasps.append((grasp_id, idx))  # Store graspID and row index
                    result['status'] = 'FAILED'
                    results.append(result)
                    continue

                # If we have exactly two unique intersections, proceed
                if num_unique == 2:
                    # Compute the middle point between the intersection points
                    middle_point = (unique_intersections[0] + unique_intersections[1]) / 2.0

                    # Adjust normals based on the method you described
                    adjusted_normals = []
                    for i, (intersection, normal) in enumerate(zip(unique_intersections, unique_normals)):
                        # Compute vector from intersection to middle point
                        vec = middle_point - intersection
                        vec /= np.linalg.norm(vec)
                        # Compute cosine similarity
                        cos_sim = np.dot(vec, normal)
                        # If they point in the same direction, invert the normal
                        if cos_sim > 0:
                            normal = -normal
                        adjusted_normals.append(normal)
                    normals = adjusted_normals

                    # Create cylinders for force vectors
                    force_cylinders = visualize_force_vectors(
                        unique_intersections,
                        normals,
                        color=color,
                        length=0.2,  # Adjusted length for better visualization
                        radius=0.0003  # Adjusted radius
                    )
                    geometries.extend(force_cylinders)

                    # Save force vectors and contact points to result
                    # Force vectors (normals scaled by length)
                    F_Ac_W_c1 = normals[0]
                    F_Ac_W_c2 = normals[1]
                    # Contact points
                    p_WC_c1 = unique_intersections[0]
                    p_WC_c2 = unique_intersections[1]

                    result.update({
                        'F_Ac_W_c1_x': F_Ac_W_c1[0],
                        'F_Ac_W_c1_y': F_Ac_W_c1[1],
                        'F_Ac_W_c1_z': F_Ac_W_c1[2],
                        'p_WC_c1_x': p_WC_c1[0],
                        'p_WC_c1_y': p_WC_c1[1],
                        'p_WC_c1_z': p_WC_c1[2],
                        'F_Ac_W_c2_x': F_Ac_W_c2[0],
                        'F_Ac_W_c2_y': F_Ac_W_c2[1],
                        'F_Ac_W_c2_z': F_Ac_W_c2[2],
                        'p_WC_c2_x': p_WC_c2[0],
                        'p_WC_c2_y': p_WC_c2[1],
                        'p_WC_c2_z': p_WC_c2[2],
                    })
                else:
                    # Only one unique intersection
                    print(f"Grasp ID {grasp_id}, Row {idx}: Grasp failed due to only one contact point.")
                    failed_grasps.append((grasp_id, idx))
                    result['status'] = 'FAILED'

                # Append the result to the list
                results.append(result)
            else:
                print(f"Missing position or orientation for row {idx}.")

    # Convert results to DataFrame and save to CSV
    results_df = pd.DataFrame(results)

    # Define the column order as specified
    columns = [
        'id', 'position_x', 'position_y', 'position_z',
        'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
        'gripper_opening', 'score',
        'F_Ac_W_c1_x', 'F_Ac_W_c1_y', 'F_Ac_W_c1_z',
        'p_WC_c1_x', 'p_WC_c1_y', 'p_WC_c1_z',
        'F_Ac_W_c2_x', 'F_Ac_W_c2_y', 'F_Ac_W_c2_z',
        'p_WC_c2_x', 'p_WC_c2_y', 'p_WC_c2_z',
        'object_com_x', 'object_com_y', 'object_com_z',
        'status'
    ]

    # Reorder columns and handle missing columns
    for col in columns:
        if col not in results_df.columns:
            results_df[col] = np.nan  # Fill missing columns with NaN

    results_df = results_df[columns]

    # Save to CSV
    results_df.to_csv(output_csv_path, sep=';', index=False)

    print(f"\nResults saved to {output_csv_path}")

    geometries.append(coordinate_frame)

    # Optionally, print or save failed grasps
    if failed_grasps:
        print("\nFailed Grasps:")
        for grasp_id, idx in failed_grasps:
            print(f"Grasp ID {grasp_id}, Row {idx}")

    # Visualization
    def custom_draw_geometries(geometries):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Gripper Visualization", width=1024, height=768)
        for geometry in geometries:
            vis.add_geometry(geometry)
        opt = vis.get_render_option()
        opt.background_color = np.array([1, 1, 1])  # White background
        vis.run()
        vis.destroy_window()

    custom_draw_geometries([mesh, *geometries])

if __name__ == "__main__":
    main()
