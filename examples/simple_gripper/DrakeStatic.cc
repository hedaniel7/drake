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

// For collision info
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"

DEFINE_string(position, "", "Position vector as comma-separated values, e.g., '1,2,3'");
DEFINE_string(orientation, "", "Orientation quaternion as comma-separated values, e.g., 'w,x,y,z'");
DEFINE_double(gripper_opening, 0.0, "Gripper opening in meters");
DEFINE_double(manual_correction, 0.0, "Manual height correction on top of the predicted gripper height");
DEFINE_double(table_correction, 0.0, "Manual height correction for the table in meters");
DEFINE_bool(NoHeightCorrection, false, "If true, does not correct the height");
DEFINE_double(advanceSimTo, 5.0, "Time to advance the simulation to in seconds.");  // Added this line
DEFINE_string(uogp_object, "", "Name of the UOGP object to load, e.g., 'CheezItBox'");


namespace drake {
    namespace examples {
        namespace simple_gripper {

            using Eigen::Vector3d;
            using multibody::ContactResults;
            using multibody::HydroelasticContactInfo;
            using drake::multibody::BodyIndex;
            using drake::multibody::PointPairContactInfo;
            using drake::geometry::GeometryId;
            using drake::geometry::FrameId;
            using drake::geometry::QueryObject;
            using drake::geometry::PenetrationAsPointPair;
            using drake::multibody::ModelInstanceIndex;


            namespace {

                // This is the Drake Simulation module used in the Master Thesis. This particular file extends the base
                // simulation with additional contact collision pair infos between object, table and gripper parts
                // (left and right gripper finger pad and other parts).

                // Part 1: Parse flags set in the command which presumably calls the simulation binary
                // (This enables a script software for example to automatically run and evaluate simulation instances
                // for an array of grasp poses predicted by e.g. the Contact-GraspNet neural network. We also avoid
                // recompiling the simulation with usage of the flags in a precompiled binary).
                // The above defined flags define for example the position and orientation of the parallel jaw
                // gripper, its gripper opening, which object to grasp and how long to run the simulation.

                // Additionally, two distinctly different height correction are made possible with the flags:
                // 1) a correction for the gripper height in direction of the gripper z-axis to accomodate the fact
                // that Contact-GraspNet gripper width predictions are trained for another gripper model Franka Panda
                // than the gripper model Robotiq 140 we use in our simulation (and on the real robot at DLR).

                // 2) a table height correction due to 2cm offset from an edge bleeding removal which we use to clean
                // the partial view point clouds of the unknown objects (more details in the Master Thesis presentation:
                // https://www.youtube.com/watch?v=LdrG_YuUSac at 15:50)


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

                // Function to predict gripper grasp point for the Robotiq 140 for a given opening in meters
                // This is the result of System identification in which we fit a polynomial function to the
                // (gripper with, gripper height) measurements of the real Robotiq 140. This allows
                // us to predict the height of the gripper given its set gripper width.
                // This is later used in the gripper height correction from Franka Panda to Robotiq 140.
                double predict_robotiq140_gripper_grasp_point_height(double cgn_gripper_width) {
                    // gripper width predicted by neural network is in meters, so we need to convert it to mm for our original function
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
                    double table_correction = FLAGS_table_correction;
                    bool no_height_correction = FLAGS_NoHeightCorrection;
                    double advance_sim_to = FLAGS_advanceSimTo;
                    std::string uogp_object = FLAGS_uogp_object;


                    std::cout << "Position: " << position_str << std::endl;
                    std::cout << "Orientation: " << orientation_str << std::endl;
                    std::cout << "Gripper Opening: " << gripper_opening << " meters" << std::endl;
                    std::cout << "Manual Correction: " << manual_correction << " meters" << std::endl;
                    std::cout << "Table Correction: " << table_correction << " meters" << std::endl;
                    std::cout << "No Height Correction: " << (no_height_correction ? "True" : "False") << std::endl;
                    std::cout << "Advancing simulation to: " << advance_sim_to << " seconds" << std::endl;
                    std::cout << "UOGP Object: " << uogp_object << std::endl;

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

                    // We correct the height of the gripper in the gripper z direction to enable an
                    // adjustment of the predicted pose by the neural network Contact-GraspNet
                    // which was trained to predict gripper width for a different gripper
                    // model (Franka Panda)

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

                    // We finally specify the height correction to be the distance in gripper approach (z-axis of gripper)
                    // between contact point on the Robotiq 140 and the contact point on the Franka Panda + some manual correction
                    Eigen::Vector3d height_correction = (robotiq_140_fingerpad_contact_pt - franka_panda_hand_contact_pt + manual_correction)  * z_axis;

                    Eigen::Vector3d height_correct_parsed_position;

                    std::cout << "Final Height correction: robotiq_140_fingerpad_contact_pt - franka_panda_hand_contact_pt + manual_correction: " << (robotiq_140_fingerpad_contact_pt - franka_panda_hand_contact_pt + manual_correction) << " meters" << std::endl;

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


                    // Part 2: Simulation of the grasp.
                    // We load in the necessary files of the gripper, table and object and simulate the grasp process

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


                    parser.AddModelsFromString(with_mimic, "dmd.yaml");
                    parser.AddModelsFromUrl(
                            "package://drake/examples/simple_gripper/robotiq_140_gripper.urdf");
                    plant.WeldFrames(
                            plant.world_frame(),
                            plant.GetBodyByName("robotiq_arg2f_base_link").body_frame(),
                            drake::math::RigidTransform<double>(final_rotation, height_correct_parsed_position));

                    plant.Finalize();

                    auto torque = builder.AddSystem<systems::ConstantVectorSource>(Vector1d(15));
                    builder.Connect(torque->get_output_port(), plant.get_actuation_input_port());

                    visualization::AddDefaultVisualization(&builder, meshcat);

                    auto diagram = builder.Build();

                    // Simulation of the grasp
                    systems::Simulator simulator(*diagram);

                    meshcat->StartRecording(32.0, false);
                    simulator.AdvanceTo(advance_sim_to);  // Use the flag value here
                    meshcat->PublishRecording();

                    const auto& final_context = simulator.get_context();

                    const auto& plant_context = diagram->GetSubsystemContext(plant, final_context);

                    // Part 3: Output of the force (and moment) vectors and their locations, the location of the
                    // object COM and its orientation

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

                    const drake::math::RotationMatrix<double>& R_WO = X_WO.rotation();

                    Eigen::Quaterniond quat = R_WO.ToQuaternion();
                    std::cout << "object quaternion (x, y, z, w): [" << quat.x() << ", "
                              << quat.y() << ", " << quat.z() << ", " << quat.w() << "]" << std::endl;

                    // Center of Mass of the to-be-grasped object
                    std::cout << "  object_com: [" << object_com_W.x() << ", " << object_com_W.y() << ", " << object_com_W.z() << "]" << std::endl;

                    // Obtain the QueryObject from the plant's geometry query input port.
                    const auto& query_object = plant.get_geometry_query_input_port().Eval<QueryObject<double>>(plant_context);

                    // Access the SceneGraph's inspector.
                    const auto& inspector = query_object.inspector();

                    // Part 4: Additional part only in this file:
                    // Process and output information about the different types of collisions between object, table and gripper parts:

                    // Get the list of all penetrations.
                    std::vector<PenetrationAsPointPair<double>> penetration_pairs = query_object.ComputePointPairPenetration();

                    // Get the ModelInstanceIndex of the gripper, table, and object
                    ModelInstanceIndex gripper_model_instance = plant.GetModelInstanceByName("robotiq_arg2f_140_model");
                    ModelInstanceIndex table_model_instance = plant.GetModelInstanceByName("table");
                    ModelInstanceIndex object_model_instance = plant.GetModelInstanceByName("spam");

                    // Collect BodyIndices of the gripper
                    std::vector<BodyIndex> gripper_body_indices = plant.GetBodyIndices(gripper_model_instance);

                    // Get BodyIndices for the left and right finger pads
                    const multibody::Body<double>& left_inner_finger_pad = plant.GetBodyByName("left_inner_finger_pad");
                    const multibody::Body<double>& right_inner_finger_pad = plant.GetBodyByName("right_inner_finger_pad");

                    BodyIndex left_pad_index = left_inner_finger_pad.index();
                    BodyIndex right_pad_index = right_inner_finger_pad.index();

                    // Collect BodyIndices of other gripper parts (excluding the finger pads)
                    std::vector<BodyIndex> gripper_other_body_indices;
                    for (const auto& body_index : gripper_body_indices) {
                        if (body_index != left_pad_index && body_index != right_pad_index) {
                            gripper_other_body_indices.push_back(body_index);
                        }
                    }

                    // Collect BodyIndices of the table
                    std::vector<BodyIndex> table_body_indices = plant.GetBodyIndices(table_model_instance);

                    // Collect BodyIndices of the object
                    std::vector<BodyIndex> object_body_indices = plant.GetBodyIndices(object_model_instance);

                    // Initialize flags.
                    bool left_pad_in_contact_with_object = false;
                    bool right_pad_in_contact_with_object = false;
                    bool other_gripper_parts_in_contact_with_object = false;
                    bool object_in_contact_with_table = false;
                    bool gripper_in_contact_with_table = false;

                    bool finger_pads_in_contact_with_each_other = false;

                    // Process the penetration pairs.
                    for (const auto& penetration : penetration_pairs) {
                        GeometryId geometryA_id = penetration.id_A;
                        GeometryId geometryB_id = penetration.id_B;

                        // Map GeometryId to FrameId using the inspector.
                        FrameId frameA_id = inspector.GetFrameId(geometryA_id);
                        FrameId frameB_id = inspector.GetFrameId(geometryB_id);

                        // Get the Body associated with each FrameId.
                        const multibody::Body<double>* bodyA = plant.GetBodyFromFrameId(frameA_id);
                        const multibody::Body<double>* bodyB = plant.GetBodyFromFrameId(frameB_id);

                        // Ensure the pointers are valid.
                        DRAKE_DEMAND(bodyA != nullptr);
                        DRAKE_DEMAND(bodyB != nullptr);

                        // Get the BodyIndex for each body.
                        BodyIndex bodyA_index = bodyA->index();
                        BodyIndex bodyB_index = bodyB->index();

                        // Check if bodyA is the left or right finger pad.
                        bool bodyA_is_left_pad = (bodyA_index == left_pad_index);
                        bool bodyB_is_left_pad = (bodyB_index == left_pad_index);

                        bool bodyA_is_right_pad = (bodyA_index == right_pad_index);
                        bool bodyB_is_right_pad = (bodyB_index == right_pad_index);

                        // **Check if the penetration is between the left and right finger pads.**
                        if ((bodyA_is_left_pad && bodyB_is_right_pad) || (bodyA_is_right_pad && bodyB_is_left_pad)) {
                            finger_pads_in_contact_with_each_other = true;
                        }

                        // Existing collision checks...
                        // Check if bodyA is other gripper parts (excluding pads).
                        bool bodyA_is_other_gripper = std::find(gripper_other_body_indices.begin(),
                                                                gripper_other_body_indices.end(), bodyA_index) != gripper_other_body_indices.end();
                        bool bodyB_is_other_gripper = std::find(gripper_other_body_indices.begin(),
                                                                gripper_other_body_indices.end(), bodyB_index) != gripper_other_body_indices.end();

                        // Check if bodyA or bodyB is part of the object.
                        bool bodyA_is_object = std::find(object_body_indices.begin(),
                                                         object_body_indices.end(), bodyA_index) != object_body_indices.end();
                        bool bodyB_is_object = std::find(object_body_indices.begin(),
                                                         object_body_indices.end(), bodyB_index) != object_body_indices.end();

                        // Check if bodyA or bodyB is part of the table.
                        bool bodyA_is_table = std::find(table_body_indices.begin(),
                                                        table_body_indices.end(), bodyA_index) != table_body_indices.end();
                        bool bodyB_is_table = std::find(table_body_indices.begin(),
                                                        table_body_indices.end(), bodyB_index) != table_body_indices.end();

                        // Check if bodyA or bodyB is part of the gripper (any part).
                        bool bodyA_is_gripper = std::find(gripper_body_indices.begin(),
                                                          gripper_body_indices.end(), bodyA_index) != gripper_body_indices.end();
                        bool bodyB_is_gripper = std::find(gripper_body_indices.begin(),
                                                          gripper_body_indices.end(), bodyB_index) != gripper_body_indices.end();

                        // Check for collisions between finger pads and object.
                        if ((bodyA_is_left_pad && bodyB_is_object) || (bodyA_is_object && bodyB_is_left_pad)) {
                            left_pad_in_contact_with_object = true;
                        }
                        if ((bodyA_is_right_pad && bodyB_is_object) || (bodyA_is_object && bodyB_is_right_pad)) {
                            right_pad_in_contact_with_object = true;
                        }

                        // Check for collisions between other gripper parts and object.
                        if ((bodyA_is_other_gripper && bodyB_is_object) || (bodyA_is_object && bodyB_is_other_gripper)) {
                            other_gripper_parts_in_contact_with_object = true;
                        }

                        // Check for collisions between object and table.
                        if ((bodyA_is_object && bodyB_is_table) || (bodyA_is_table && bodyB_is_object)) {
                            object_in_contact_with_table = true;
                        }

                        // Check for collisions between gripper (any part) and table.
                        if ((bodyA_is_gripper && bodyB_is_table) || (bodyA_is_table && bodyB_is_gripper)) {
                            gripper_in_contact_with_table = true;
                        }
                    }

                    // Output the collision results.
                    std::cout << "Left finger pad in collision with object: "
                              << (left_pad_in_contact_with_object ? "Yes" : "No") << std::endl;

                    std::cout << "Right finger pad in collision with object: "
                              << (right_pad_in_contact_with_object ? "Yes" : "No") << std::endl;

                    std::cout << "Other gripper parts in collision with object: "
                              << (other_gripper_parts_in_contact_with_object ? "Yes" : "No") << std::endl;

                    std::cout << "Object in collision with table: "
                              << (object_in_contact_with_table ? "Yes" : "No") << std::endl;

                    std::cout << "Gripper in collision with table: "
                              << (gripper_in_contact_with_table ? "Yes" : "No") << std::endl;

                    std::cout << "Finger pads in collision with each other: "
                              << (finger_pads_in_contact_with_each_other ? "Yes" : "No") << std::endl;

                    // Pause so that you can see the Meshcat output.
                    std::cout << "[Press Enter to finish]." << std::endl;
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                    return 0;
                }

            }
        }  // namespace simple_gripper
    }  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
    return drake::examples::simple_gripper::do_main(argc, argv);
}
