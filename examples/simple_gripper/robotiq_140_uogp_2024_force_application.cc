#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/collision_filter_groups.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/contact_results.h"

#include <drake/multibody/tree/rigid_body.h>
#include <drake/multibody/tree/spatial_inertia.h>

#include "drake/geometry/proximity_properties.h"

#include <gflags/gflags.h>

// for the ExternalForceApplicator

#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/collision_filter_groups.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/math/rigid_transform.h"
#include <drake/multibody/tree/spatial_inertia.h>

void printVector(const std::string& name, const Eigen::Vector3d& vec) {
    // Print the formatted vector
    std::cout << name << ": ["
              << vec.x() << ", "
              << vec.y() << ", "
              << vec.z() << "]"
              << std::endl;
}

Eigen::Vector3d SelectUpwardAxis(const Eigen::Vector3d& axis) {
    // Define the upward direction in the world frame
    const Eigen::Vector3d upward_direction(0.0, 0.0, 1.0);

    // Normalize the axis vector to compute cosine similarity
    Eigen::Vector3d normalized_axis = axis.normalized();

    // Compute cosine similarity with the upward direction
    double cosine_similarity = normalized_axis.dot(upward_direction);

    // Debug: Print cosine similarity (optional)
    // std::cout << "Cosine Similarity with Upward Direction: " << cosine_similarity << std::endl;

    // If cosine similarity is negative, negate the axis to point upward
    if (cosine_similarity < 0.0) {
        return -axis;
    } else {
        return axis;
    }
}

// Function to select the predefined axis direction based on input string
Eigen::Vector3d GetAxisSelectionNormalized(const std::string& direction_flag,
                                           const Eigen::Vector3d& x_axis_normalized,
                                           const Eigen::Vector3d& y_axis_normalized,
                                           const Eigen::Vector3d& z_axis_normalized,
                                           const Eigen::Vector3d& xy_axis_normalized,
                                           const Eigen::Vector3d& yz_axis_normalized,
                                           const Eigen::Vector3d& xz_axis_normalized,
                                           const Eigen::Vector3d& xyz_axis_normalized) {
    if (direction_flag == "x") {
        return x_axis_normalized;
    } else if (direction_flag == "y") {
        return y_axis_normalized;
    } else if (direction_flag == "z") {
        return z_axis_normalized;
    } else if (direction_flag == "xy") {
        return xy_axis_normalized;
    } else if (direction_flag == "yz") {
        return yz_axis_normalized;
    } else if (direction_flag == "xz") {
        return xz_axis_normalized;
    } else if (direction_flag == "xyz") {
        return xyz_axis_normalized;
    } else {
        throw std::invalid_argument("Invalid axis selection: " + direction_flag);
    }
}



namespace drake {
    namespace multibody {

// Class definition of the LeafSystem which outputs multiple wrenches (forces and torques)
        class ExternalForceApplicator : public systems::LeafSystem<double> {
        public:
            explicit ExternalForceApplicator(const MultibodyPlant<double>* plant);

            // Existing method to add a force
            void AddForce(double start_time, double end_time, double force_magnitude, const Eigen::Vector3d& force_direction);

            // New method to add a wrench (force and torque)
            void AddWrench(double start_time, double end_time,
                           double force_magnitude, const Eigen::Vector3d& force_direction,
                           double torque_magnitude, const Eigen::Vector3d& torque_direction);

        private:
            void CalcSpatialForceOutput(
                    const systems::Context<double>& context,
                    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const;

            const MultibodyPlant<double>* plant_{nullptr};

            // Vectors to store start and end times for forces and wrenches
            std::vector<double> start_times_;
            std::vector<double> end_times_;

            // Vectors to store force magnitudes and directions
            std::vector<double> force_magnitudes_;
            std::vector<Eigen::Vector3d> force_directions_;

            // Vectors to store torque magnitudes and directions
            std::vector<double> torque_magnitudes_;
            std::vector<Eigen::Vector3d> torque_directions_;
        };

// Constructor where we define the callback function of the output port
        ExternalForceApplicator::ExternalForceApplicator(const MultibodyPlant<double>* plant)
                : plant_(plant) {
            this->DeclareAbstractOutputPort(
                    "spatial_force_output",
                    &ExternalForceApplicator::CalcSpatialForceOutput);
        }

// Method to add a force (unchanged)
        void ExternalForceApplicator::AddForce(double start_time, double end_time, double force_magnitude, const Eigen::Vector3d& force_direction) {
            start_times_.push_back(start_time);
            end_times_.push_back(end_time);
            force_magnitudes_.push_back(force_magnitude);
            force_directions_.push_back(force_direction);

            // For forces, we push back zero torque
            torque_magnitudes_.push_back(0.0);
            torque_directions_.push_back(Eigen::Vector3d::Zero());
        }

// New method to add a wrench (force and torque)
        void ExternalForceApplicator::AddWrench(double start_time, double end_time,
                                                double force_magnitude, const Eigen::Vector3d& force_direction,
                                                double torque_magnitude, const Eigen::Vector3d& torque_direction) {
            start_times_.push_back(start_time);
            end_times_.push_back(end_time);
            force_magnitudes_.push_back(force_magnitude);
            force_directions_.push_back(force_direction);
            torque_magnitudes_.push_back(torque_magnitude);
            torque_directions_.push_back(torque_direction);
        }

// This method specifies what is output from this LeafSystem continually
        void ExternalForceApplicator::CalcSpatialForceOutput(
                const systems::Context<double>& context,
                std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const {

            const double current_time = context.get_time();
            output->clear();

            const RigidBody<double>& object =
                    dynamic_cast<const RigidBody<double>&>(plant_->GetBodyByName("base_link"));
            // Valid names in model instance 'spam' (the object to be grasped) are: base_link

            const BodyIndex object_body_index = object.index();
            const Vector3<double> object_com = object.default_com();
            const double g = UniformGravityFieldElement<double>::kDefaultStrength;

            // Loop through all the time windows and output forces and torques at the given simulation time
            for (size_t i = 0; i < start_times_.size(); ++i) {
                if (current_time >= start_times_[i] && current_time <= end_times_[i]) {
                    // Calculate the force vector
                    Vector3<double> force_vector = force_magnitudes_[i] * force_directions_[i];

                    // Calculate the torque vector
                    Vector3<double> torque_vector = torque_magnitudes_[i] * torque_directions_[i];

                    // Create the spatial force (torque and force)
                    const SpatialForce<double> F_object_com_W(object.default_mass() * g * torque_vector,
                                                              object.default_mass() * g * force_vector);

                    output->emplace_back();
                    auto& force = output->back();
                    force.body_index = object_body_index;
                    force.p_BoBq_B = object_com;
                    force.F_Bq_W = F_object_com_W;
                }
            }
        }

    }  // namespace multibody
}  // namespace drake
DEFINE_string(position, "", "Position vector as comma-separated values, e.g., '1,2,3'");
DEFINE_string(orientation, "", "Orientation quaternion as comma-separated values, e.g., '0,0,0,1'");
DEFINE_double(gripper_opening, 0.0, "Gripper opening in meters");
DEFINE_double(manual_correction, 0.0, "Manual height correction on top of the predicted gripper height");
DEFINE_double(table_correction, 0.0, "Manual height correction for the table in meters");
DEFINE_double(force_magnitude, 1.0, "Magnitude of the applied force in Newtons.");
DEFINE_double(torque_magnitude, 1.0, "Magnitude of the applied torque in Newton-meters.");
DEFINE_bool(NoHeightCorrection, false, "If true, does not correct the height");
DEFINE_double(advanceSimTo, 5.0, "Time to advance the simulation to in seconds.");
DEFINE_string(uogp_object, "", "Name of the UOGP object to load, e.g., 'CheezItBox'");
DEFINE_double(force_start, 0.55, "Start of the application of the force.");
DEFINE_double(force_end, 0.56, "End of the application of the force.");
DEFINE_string(SelectForceDirection, "y", "Force direction selection: 'x', 'y', 'z', 'xy', 'yz', 'xz', 'xyz'");
DEFINE_string(SelectMomentDirection, "x", "Moment direction selection: 'x', 'y', 'z', 'xy', 'yz', 'xz', 'xyz'");
DEFINE_bool(useNegativeForceAxis, false, "If true, invert the selected force axis direction.");
DEFINE_bool(useNegativeMomentAxis, false, "If true, invert the selected moment axis direction.");



namespace drake {
    namespace examples {
        namespace simple_gripper {

            using Eigen::Vector3d;
            using multibody::ContactResults;
            using multibody::HydroelasticContactInfo;
            namespace {

                Eigen::Vector3d parse_position(const std::string& position_str) {
                    std::istringstream iss(position_str);
                    double x, y, z;
                    char comma;
                    if (!(iss >> x >> comma >> y >> comma >> z) || comma != ',') {
                        throw std::runtime_error("Invalid position format. Expected 'x,y,z'");
                    }
                    return Eigen::Vector3d(x, y, z);
                }

                Eigen::Quaterniond parse_orientation(const std::string& orientation_str) {
                    std::istringstream iss(orientation_str);
                    double x, y, z, w;
                    char comma;
                    if (!(iss >> x >> comma >> y >> comma >> z >> comma >> w) || comma != ',') {
                        throw std::runtime_error("Invalid orientation format. Expected 'w,x,y,z'");
                    }
                    return Eigen::Quaterniond(w, x, y, z).normalized();
                }

                // Function to predict gripper grasp point for the robotiq 140 for a given opening in meters
                double predict_robotiq140_gripper_grasp_point_height(double cgn_gripper_width) {
                    // cgn_gripper_width is in meters, so we need to convert it to mm for our original function
                    double x_mm = cgn_gripper_width * 1000;
                    double height_mm = 229.644116 + 0.004132*x_mm - 0.000725*std::pow(x_mm, 2) - 0.000004*std::pow(x_mm, 3);
                    // Convert the result back to meters
                    return height_mm / 1000;
                }

                int do_main(int argc, char* argv[]) {
                    gflags::ParseCommandLineFlags(&argc, &argv, true);
                    std::string position_str = FLAGS_position;
                    std::string orientation_str = FLAGS_orientation;
                    double gripper_opening = FLAGS_gripper_opening;
                    double manual_correction = FLAGS_manual_correction;
                    double table_correction = FLAGS_table_correction; // Parsed table_correction
                    bool no_height_correction = FLAGS_NoHeightCorrection;
                    double advance_sim_to = FLAGS_advanceSimTo;  // Retrieve the simulation time
                    double force_magnitude = FLAGS_force_magnitude;
                    double torque_magnitude = FLAGS_torque_magnitude;
                    std::string uogp_object = FLAGS_uogp_object;
                    std::string select_force_direction = FLAGS_SelectForceDirection;
                    std::string select_moment_direction = FLAGS_SelectMomentDirection;
                    bool use_negative_force_axis = FLAGS_useNegativeForceAxis;
                    bool use_negative_moment_axis = FLAGS_useNegativeMomentAxis;



                    std::cout << "Position: " << position_str << std::endl;
                    std::cout << "Orientation: " << orientation_str << std::endl;
                    std::cout << "Gripper Opening: " << gripper_opening << " meters" << std::endl;
                    std::cout << "Manual Correction: " << manual_correction << " meters" << std::endl;
                    std::cout << "Table Correction: " << table_correction << " meters" << std::endl; // Display table_correction
                    std::cout << "Table Correction: " << table_correction << " meters" << std::endl; // Display table_correction
                    std::cout << "No Height Correction: " << (no_height_correction ? "True" : "False") << std::endl;
                    std::cout << "Advancing simulation to: " << advance_sim_to << " seconds" << std::endl;  // Display simulation time
                    std::cout << "force magnitude: " << force_magnitude << std::endl;  // Display simulation time
                    std::cout << "torque magnitude: " << torque_magnitude << std::endl;  // Display simulation time
                    std::cout << "UOGP Object: " << uogp_object << std::endl;
                    std::cout << "SelectForceDirection: " << select_force_direction << std::endl;
                    std::cout << "SelectMomentDirection: " << select_moment_direction << std::endl;
                    std::cout << "Use Negative Force Axis: " << (use_negative_force_axis ? "True" : "False") << std::endl;
                    std::cout << "Use Negative Moment Axis: " << (use_negative_moment_axis ? "True" : "False") << std::endl;


                    // Parse position and orientation
                    Eigen::Vector3d parsed_position;
                    Eigen::Quaterniond parsed_orientation;

                    try {
                        parsed_position = parse_position(position_str);
                        parsed_orientation = parse_orientation(orientation_str);
                    } catch (const std::runtime_error& e) {
                        std::cerr << "Error parsing input: " << e.what() << std::endl;
                        return 1;
                    }

                    drake::math::RotationMatrix<double> orientation_matrix = drake::math::RotationMatrix<double>(parsed_orientation);

                    // Extract the z-axis (third column) from the orientation matrix
                    Eigen::Vector3d z_axis = orientation_matrix.matrix().col(2);

                    // Calculate the height correction based upon the predicted gripper height plus some manual correction (Magic Number)
                    double predicted_gripper_height = predict_robotiq140_gripper_grasp_point_height(gripper_opening);
                    std::cout << "Predicted Gripper Opening: " << predicted_gripper_height << " meters" << std::endl;

                    std::cout << "Predicted Gripper Opening + manual_correction: " << (predicted_gripper_height + manual_correction) << " meters" << std::endl;

                    double franka_panda_hand_height = 0.127;
                    double franka_panda_hand_fingertip = 0.018;

                    // We specify the location of the contact point on the Franka Panda hand from its height
                    double franka_panda_hand_contact_pt = franka_panda_hand_height - franka_panda_hand_fingertip / 2.0;

                    double robotiq_140_fingerpad_length = 0.0655;

                    // Similar to the calculation of the contact on the Franka Panda hand we calculate the contact point
                    // on the Robotiq 140 gripper from its predicted height
                    double robotiq_140_fingerpad_contact_pt = predicted_gripper_height - robotiq_140_fingerpad_length / 2.0;


                    // We finally specify the height correction to be the distance in grippper approach (z-axis of gripper)
                    // between contact point on the Robotiq 140 and the contact point on the Franka Panda + some manual correction
                    Eigen::Vector3d height_correction = (robotiq_140_fingerpad_contact_pt - franka_panda_hand_contact_pt + manual_correction)  * z_axis;

                    std::cout << "Final Height correction: robotiq_140_fingerpad_contact_pt - franka_panda_hand_contact_pt + manual_correction: " << (robotiq_140_fingerpad_contact_pt - franka_panda_hand_contact_pt + manual_correction) << " meters" << std::endl;

                    Eigen::Vector3d height_correct_parsed_position;

                    if (!no_height_correction) {
                        // Apply height correction
                        // Adjust the parsed position
                        height_correct_parsed_position = parsed_position - height_correction;

                        // Print the height-corrected position
                        std::cout << "Height Corrected Parsed Position: ["
                                  << height_correct_parsed_position.x() << ", "
                                  << height_correct_parsed_position.y() << ", "
                                  << height_correct_parsed_position.z() << "]"
                                  << std::endl;
                    } else {
                        // Do not apply height correction
                        std::cout << "No Height Correction applied." << std::endl;
                        height_correct_parsed_position = parsed_position;

                        // Print the uncorrected position
                        std::cout << "Parsed Position: ["
                                  << height_correct_parsed_position.x() << ", "
                                  << height_correct_parsed_position.y() << ", "
                                  << height_correct_parsed_position.z() << "]"
                                  << std::endl;
                    }

                    // Create a 90-degree rotation around the z-axis
                    drake::math::RotationMatrix<double> z_rotation = drake::math::RotationMatrix<double>::MakeZRotation(M_PI / 2.0);

                    // Combine default orientation with parsed orientation
                    drake::math::RotationMatrix<double> final_rotation = orientation_matrix * z_rotation;

                    // Output the final rotation matrix with determinant
                    // std::string final_matrix_str = matrix_to_string_with_det(final_rotation);
                    // drake::log()->info("Final Rotation matrix:\n{}", final_matrix_str);


                    auto meshcat = std::make_shared<geometry::Meshcat>();
                    systems::DiagramBuilder<double> builder;

                    auto [plant, scene_graph] =
                            multibody::AddMultibodyPlantSceneGraph(&builder, 0.002);
                    plant.set_discrete_contact_approximation( drake::multibody::DiscreteContactApproximation::kSimilar);

                    multibody::Parser parser(&plant);
                    multibody::PackageMap::RemoteParams params;
                    params.urls = {"https://github.com/RussTedrake/kinova-movo/archive/"
                                   "d94d1d7da7ff8fc71f2439bb0a8989f1e6fd79b4.tar.gz"};
                    params.sha256 =
                            "a9201477a23f410f10d00e86847de778c175d3d3c8971be52a9ac881194e4887";
                    params.strip_prefix = "kinova-movo-d94d1d7da7ff8fc71f2439bb0a8989f1e6fd79b4";
                    parser.package_map().AddRemote("kinova-movo", params);
                    parser.package_map().AddPackageXml(
                            parser.package_map().GetPath("kinova-movo") +
                            "/movo_common/movo_description/package.xml");

                    // Compute the new table height
                    double base_table_height = -0.7645;
                    double total_table_height = base_table_height + table_correction;

                    std::string object_file;
                    if (!uogp_object.empty()) {
                        // Use the specified UOGP object
                        object_file = fmt::format("package://drake/examples/simple_gripper/uogp_2024/{0}/{0}.sdf", uogp_object);
                    } else {
                        // Default object file
                        object_file = "package://drake/examples/simple_gripper/mesh.sdf";
                    }



                    // Update the translation in the YAML string
                    std::string with_mimic = fmt::format(R"""(
directives:
- add_model:
    name: spam
    file: {object_file}
    default_free_body_pose: {{ base_link: {{
        translation: [0.0, 0.00, 0.0],
        rotation: !Rpy {{ deg: [0.0, 0.0, 0.0 ]}}
    }} }}

- add_model:
    name: table
    file: package://drake/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table_surface_only_collision.sdf

- add_weld:
    parent: world
    child: table::table_link
    X_PC:
        translation: [0.0, 0.0, {:.5f}]
)""", total_table_height, fmt::arg("object_file", object_file));

                    // Explanation:
                    // The base table height is -0.768 meters.
                    // table_correction is added to this base height.
                    // The total_table_height is then inserted into the YAML string.

                    parser.AddModelsFromString(with_mimic, "dmd.yaml");
                    parser.AddModelsFromUrl(
                            "package://drake/examples/simple_gripper/robotiq_140_gripper.urdf");
                    plant.WeldFrames(
                            plant.world_frame(),
                            plant.GetBodyByName("robotiq_arg2f_base_link").body_frame(),
                            // math::RigidTransformd(math::RollPitchYawd(M_PI , 0, M_PI),
                            //                      Eigen::Vector3d(0.2, 0, 0.21)));
                            drake::math::RigidTransform<double>(final_rotation, height_correct_parsed_position));

                    plant.Finalize();

                    auto torque = builder.AddSystem<systems::ConstantVectorSource>(Vector1d(15));
                    builder.Connect(torque->get_output_port(), plant.get_actuation_input_port());

                    visualization::AddDefaultVisualization(&builder, meshcat);

                    // Create ExternalForceApplicator instance, a Leafsystem which can continually output a force
                    auto external_force_applicator =
                            builder.AddSystem<drake::multibody::ExternalForceApplicator>(&plant);

                    // Force to keep the object standing
                    external_force_applicator->AddForce(0.0, 0.2, 1.0, Vector3d(0, 0, 1));

                    // Compute gripper axes
                    Eigen::Vector3d gripper_x_axis = orientation_matrix.matrix().col(0);
                    Eigen::Vector3d gripper_y_axis = orientation_matrix.matrix().col(1);
                    Eigen::Vector3d gripper_z_axis = orientation_matrix.matrix().col(2);

                    printVector("gripper_x_axis", gripper_x_axis);
                    printVector("gripper_y_axis", gripper_y_axis);
                    printVector("gripper_z_axis", gripper_z_axis);

                    // Select the upward-pointing axes using the SelectUpwardAxis function
                    Eigen::Vector3d gripper_x_axis_selection = SelectUpwardAxis(gripper_x_axis);
                    Eigen::Vector3d gripper_y_axis_selection = SelectUpwardAxis(gripper_y_axis);
                    Eigen::Vector3d gripper_z_axis_selection = SelectUpwardAxis(gripper_z_axis);

                    printVector("gripper_x_axis_selection", gripper_x_axis_selection);
                    printVector("gripper_y_axis_selection", gripper_y_axis_selection);
                    printVector("gripper_z_axis_selection", gripper_z_axis_selection);

                    Eigen::Vector3d gripper_x_axis_selection_normalized = gripper_x_axis_selection.normalized();
                    Eigen::Vector3d gripper_y_axis_selection_normalized = gripper_y_axis_selection.normalized();
                    Eigen::Vector3d gripper_z_axis_selection_normalized = gripper_z_axis_selection.normalized();

                    Eigen::Vector3d gripper_xy_axis_selection_normalized = (gripper_x_axis_selection + gripper_y_axis_selection).normalized();
                    Eigen::Vector3d gripper_yz_axis_selection_normalized = (gripper_y_axis_selection + gripper_z_axis_selection).normalized();
                    Eigen::Vector3d gripper_xz_axis_selection_normalized = (gripper_x_axis_selection + gripper_z_axis_selection).normalized();

                    printVector("gripper_xy_axis_selection_normalized", gripper_xy_axis_selection_normalized);
                    printVector("gripper_yz_axis_selection_normalized", gripper_yz_axis_selection_normalized);
                    printVector("gripper_xz_axis_selection_normalized", gripper_xz_axis_selection_normalized);

                    Eigen::Vector3d gripper_xyz_axis_selection_normalized = (gripper_x_axis_selection + gripper_y_axis_selection + gripper_z_axis_selection).normalized();

                    printVector("gripper_xyz_axis_selection_normalized", gripper_xyz_axis_selection_normalized);

                    // Function to select axis direction based on the flags
                    Eigen::Vector3d force_direction;
                    Eigen::Vector3d torque_direction;

                    try {
                        force_direction = GetAxisSelectionNormalized(select_force_direction,
                                                                     gripper_x_axis_selection_normalized,
                                                                     gripper_y_axis_selection_normalized,
                                                                     gripper_z_axis_selection_normalized,
                                                                     gripper_xy_axis_selection_normalized,
                                                                     gripper_yz_axis_selection_normalized,
                                                                     gripper_xz_axis_selection_normalized,
                                                                     gripper_xyz_axis_selection_normalized);

                        torque_direction = GetAxisSelectionNormalized(select_moment_direction,
                                                                      gripper_x_axis_selection_normalized,
                                                                      gripper_y_axis_selection_normalized,
                                                                      gripper_z_axis_selection_normalized,
                                                                      gripper_xy_axis_selection_normalized,
                                                                      gripper_yz_axis_selection_normalized,
                                                                      gripper_xz_axis_selection_normalized,
                                                                      gripper_xyz_axis_selection_normalized);
                    } catch (const std::invalid_argument& e) {
                        std::cerr << "Error: " << e.what() << std::endl;
                        return 1;
                    }

                    // Apply negative axis flags
                    if (use_negative_force_axis) {
                        force_direction = -force_direction;
                    }

                    if (use_negative_moment_axis) {
                        torque_direction = -torque_direction;
                    }

                    printVector("Selected force direction", force_direction);
                    printVector("Selected torque direction", torque_direction);

                    // Add a wrench (force and torque)
                    /*
                    external_force_applicator->AddWrench(
                            0.5, 1.0,
                            500.0, gripper_y_axis.normalized(),   // Force magnitude and direction
                            0.0, gripper_x_axis);   // Torque magnitude and direction
                            */


                    /*
                    // Add a wrench (force and torque)
                    external_force_applicator->AddWrench(
                            0.5, 1.0,
                            500.0, good_vector_normalized,   // Force magnitude and normalized direction
                            0.0, gripper_x_axis);     // Torque magnitude and direction
                            */



                    // Add the wrench using the selected directions
                    external_force_applicator->AddWrench(
                            FLAGS_force_start, FLAGS_force_end,
                            force_magnitude, force_direction,   // Force magnitude and normalized direction
                            torque_magnitude, torque_direction); // Torque magnitude and direction



                    // Connect the external force applicator system to the MBP.
                    builder.Connect(external_force_applicator->get_output_port(0),
                                    plant.get_applied_spatial_force_input_port());

                    auto diagram = builder.Build();

                    // Set up simulator.
                    systems::Simulator simulator(*diagram);

                    meshcat->StartRecording(32.0, false);
                    simulator.AdvanceTo(advance_sim_to);  // Use the flag value here
                    meshcat->PublishRecording();

                    const auto& final_context = simulator.get_context();

                    const auto& plant_context = diagram->GetSubsystemContext(plant, final_context);

                    const ContactResults<double>& contact_results =
                            plant.get_contact_results_output_port().Eval<ContactResults<double>>(plant_context);

                    std::cout << "Contact forces and centroids at the end of the simulation:" << std::endl;
                    for (int i = 0; i < contact_results.num_hydroelastic_contacts(); ++i) {
                        const HydroelasticContactInfo<double>& info =
                                contact_results.hydroelastic_contact_info(i);

                        const Vector3d& F_Ac_W = info.F_Ac_W().translational();
                        const Vector3d& p_WC = info.contact_surface().centroid();
                        // const Vector3d& face_normal = info.contact_surface().face_normal();
                        const Vector3d& tau_Ac_W = info.F_Ac_W().rotational();

                        std::cout << "Contact " << i << ":" << std::endl;
                        // Force applied on body A, at the centroid point C, expressed in the world frame W
                        std::cout << "  F_Ac_W: [" << F_Ac_W.x() << ", " << F_Ac_W.y() << ", " << F_Ac_W.z() << "]" << std::endl;
                        // position p_WC of the centroid point C in the world frame W
                        std::cout << "  p_WC: [" << p_WC.x() << ", " << p_WC.y() << ", " << p_WC.z() << "]" << std::endl;
                        // face normal of contact point i
                        //std::cout << "  face_normal: [" << face_normal.x() << ", " << face_normal.y() << ", " << face_normal.z() << "]" << std::endl;
                        // Moment
                        std::cout << "  tau_Ac_W: [" << tau_Ac_W.x() << ", " << tau_Ac_W.y() << ", " << tau_Ac_W.z() << "]" << std::endl;
                    }


                    const drake::multibody::RigidBody<double>& object =
                            dynamic_cast<const drake::multibody::RigidBody<double>&>(plant.GetBodyByName("base_link"));

                    const drake::multibody::SpatialInertia<double>& spatial_inertia = object.default_spatial_inertia();
                    const Vector3<double> object_com = spatial_inertia.get_com();

                    const auto& X_WO = plant.EvalBodyPoseInWorld(plant_context, object);
                    const Vector3<double> object_com_W = X_WO * object_com;

                    // Center of Mass of the to be grasped object
                    std::cout << "  object_com: [" << object_com_W.x() << ", " << object_com_W.y() << ", " << object_com_W.z() << "]" << std::endl;


                    const drake::math::RotationMatrix<double>& R_WO = X_WO.rotation();


                    //std::cout << "Rotation Matrix R_WO:\n" << R_WO.matrix() << std::endl; // doesn't work


                    Eigen::Quaterniond quat = R_WO.ToQuaternion();
                    std::cout << "Quaternion (x, y, z, w): [" << quat.x() << ", "
                              << quat.y() << ", " << quat.z() << ", " << quat.w() << "]" << std::endl;


                    drake::math::RollPitchYaw<double> rpy(R_WO);

                    std::cout << "Euler angles (degrees): roll = " << rpy.roll_angle() * 180.0 / M_PI
                              << ", pitch = " << rpy.pitch_angle() * 180.0 / M_PI
                              << ", yaw = " << rpy.yaw_angle() * 180.0 / M_PI << std::endl;

                    /*
                    std::cout << "Euler angles (radians): roll = " << rpy.roll_angle()
                              << ", pitch = " << rpy.pitch_angle()
                              << ", yaw = " << rpy.yaw_angle() << std::endl;
                              */

                    // Pause so that you can see the meshcat output.
                    std::cout << "[Press Ctrl-C to finish]." << std::endl;
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                    return 0;
                }

            }  // namespace
        }  // namespace simple_gripper
    }  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
    return drake::examples::simple_gripper::do_main(argc, argv);
}
