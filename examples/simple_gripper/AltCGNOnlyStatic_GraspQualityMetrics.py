import pandas as pd
import numpy as np
import argparse
import traceback
import os

# Import your DrakeGraspQualityMetrics class
from drake_grasp_quality_metrics import DrakeGraspQualityMetrics

# Function to prepare input data
def prepare_input_data(row):
    input_data = []
    # Contact 1
    try:
        F_c1_W = np.array([
            float(row['F_Ac_W_c1_x']),
            float(row['F_Ac_W_c1_y']),
            float(row['F_Ac_W_c1_z'])
        ])
        p_WC1_W = np.array([
            float(row['p_WC_c1_x']),
            float(row['p_WC_c1_y']),
            float(row['p_WC_c1_z'])
        ])
    except (ValueError, KeyError) as e:
        raise ValueError(f"Invalid or missing data for Contact 1: {e}")

    input_data.append({'F_c_W': F_c1_W, 'p_WC_W': p_WC1_W})

    # Contact 2
    try:
        F_c2_W = np.array([
            float(row['F_Ac_W_c2_x']),
            float(row['F_Ac_W_c2_y']),
            float(row['F_Ac_W_c2_z'])
        ])
        p_WC2_W = np.array([
            float(row['p_WC_c2_x']),
            float(row['p_WC_c2_y']),
            float(row['p_WC_c2_z'])
        ])
    except (ValueError, KeyError) as e:
        raise ValueError(f"Invalid or missing data for Contact 2: {e}")

    input_data.append({'F_c_W': F_c2_W, 'p_WC_W': p_WC2_W})

    # Object's COM
    try:
        p_WO_W = np.array([
            float(row['object_com_x']),
            float(row['object_com_y']),
            float(row['object_com_z'])
        ])
    except (ValueError, KeyError) as e:
        raise ValueError(f"Invalid or missing data for object COM: {e}")

    return input_data, p_WO_W

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Compute grasp quality metrics.')
    parser.add_argument('--ids', nargs='*', type=int, help='List of grasp IDs to process')
    parser.add_argument('--input_csv', type=str,
                        default='object_2_UOGPLog_heightCorrected_withCGNOnlyForceAnalysis.csv',
                        help='Path to input CSV file (default: object_2_UOGPLog_heightCorrected_withCGNOnlyForceAnalysis.csv)')
    args = parser.parse_args()

    # Input CSV file path
    input_csv_file = args.input_csv

    # Output CSV file path (append '_withGraspQuality' to the input filename)
    base_name, ext = os.path.splitext(input_csv_file)
    output_csv_file = f"{base_name}_withGraspQuality{ext}"

    # Read the CSV file
    df = pd.read_csv(input_csv_file, delimiter=';')

    # If IDs are specified, filter the DataFrame
    if args.ids:
        df = df[df['id'].isin(args.ids)].reset_index(drop=True)
        if df.empty:
            print(f"No data found for the specified IDs: {args.ids}")
            return
    else:
        print("No IDs specified. Processing all grasps.")

    # Initialize lists to store the computed metrics
    distance_centroid_COM_metrics = []
    grasp_matrix_min_singular_value_metrics = []
    epsilon_metric_linears = []
    epsilon_metric_rotationals = []

    # Iterate over each row in the DataFrame
    for index, row in df.iterrows():
        grasp_id = row['id']
        print(f"\nProcessing grasp ID {grasp_id}")
        try:
            # Check if the status is 'success'
            if row['status'].lower() != 'success':
                print(f"Skipping grasp ID {grasp_id} due to FAILED status.")
                # Append 'FAILED' to indicate failure
                distance_centroid_COM_metrics.append('FAILED')
                grasp_matrix_min_singular_value_metrics.append('FAILED')
                epsilon_metric_linears.append('FAILED')
                epsilon_metric_rotationals.append('FAILED')
                continue  # Skip to the next row

            # Prepare the input data
            input_data, p_WO_W = prepare_input_data(row)
            print(f"Input data prepared for grasp ID {grasp_id}")

            # Initialize the DrakeGraspQualityMetrics object
            drake_metrics = DrakeGraspQualityMetrics(
                input_data=input_data,
                p_WO_W=p_WO_W,
                mu=0.5,
                gamma=0.1,
                soft_finger_contact=True,
                plot_results=False
            )

            # Compute the metrics
            (epsilon_metric_linear, epsilon_metric_rotational,
             grasp_matrix_min_singular_value_metric,
             distance_centroid_COM_metric) = drake_metrics.compute_grasp_metrics()

            # Append the metrics to the lists
            distance_centroid_COM_metrics.append(distance_centroid_COM_metric)
            grasp_matrix_min_singular_value_metrics.append(grasp_matrix_min_singular_value_metric)
            epsilon_metric_linears.append(epsilon_metric_linear)
            epsilon_metric_rotationals.append(epsilon_metric_rotational)

            print(f"Computed metrics for grasp ID {grasp_id}")

        except Exception as e:
            print(f"Error computing metrics for grasp ID {grasp_id}: {e}")
            traceback.print_exc()
            # Append 'FAILED' to indicate failure
            distance_centroid_COM_metrics.append('FAILED')
            grasp_matrix_min_singular_value_metrics.append('FAILED')
            epsilon_metric_linears.append('FAILED')
            epsilon_metric_rotationals.append('FAILED')

    # Add the metrics to the DataFrame
    df['distance_centroid_COM_metric'] = distance_centroid_COM_metrics
    df['grasp_matrix_min_singular_value_metric'] = grasp_matrix_min_singular_value_metrics
    df['epsilon_metric_linear'] = epsilon_metric_linears
    df['epsilon_metric_rotational'] = epsilon_metric_rotationals

    # Save the updated DataFrame to a new CSV file
    df.to_csv(output_csv_file, sep=';', index=False)

    print(f"\nMetrics computed and saved to {output_csv_file}")

if __name__ == "__main__":
    main()
