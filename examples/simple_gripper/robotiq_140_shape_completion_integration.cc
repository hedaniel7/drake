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

DEFINE_string(position, "", "Position vector as comma-separated values, e.g., '1,2,3'");
DEFINE_string(orientation, "", "Orientation quaternion as comma-separated values, e.g., '0,0,0,1'");

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

                int do_main(int argc, char* argv[]) {
                    gflags::ParseCommandLineFlags(&argc, &argv, true);
                    std::string position_str = FLAGS_position;
                    std::string orientation_str = FLAGS_orientation;

                    std::cout << "Position: " << position_str << std::endl;
                    std::cout << "Orientation: " << orientation_str << std::endl;

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

                    // Print out all elements of the quaternion
                    std::cout << std::fixed << std::setprecision(6)
                              << "Parsed Orientation Eigen Quaternion:\n"
                              << "z: " << parsed_orientation.z() << "\n"
                              << "x: " << parsed_orientation.x() << "\n"
                              << "y: " << parsed_orientation.y() << "\n"
                              << "w: " << parsed_orientation.w() <<  std::endl;


                    // Create the default orientation
                    //drake::math::RotationMatrix<double> default_rotation =
                    //        drake::math::RotationMatrix<double>::MakeRotationMatrixFromRpy(M_PI, 0, M_PI);
                    //drake::math::RollPitchYaw<double> default_rotation_rpy(M_PI, 0, M_PI);
                    //drake::math::RollPitchYaw<double> default_rotation_rpy(0.0872665, 0, 0); // 5 deg x rotation
                    //drake::math::RollPitchYaw<double> default_rotation_rpy(0, 0.0872665, 0); // 5 deg y rotation
                    //drake::math::RollPitchYaw<double> default_rotation_rpy(0, 0.2617994, 0); // 15 deg y rotation
                    //drake::math::RollPitchYaw<double> default_rotation_rpy(0, 0, 0.0872665); // 5 deg z rotation
                    //drake::math::RollPitchYaw<double> default_rotation_rpy(0, 0, 0); // no rotation

                    drake::math::RollPitchYaw<double> default_rotation_rpy(0, M_PI, 0); // 180 deg y rotation
                    drake::math::RotationMatrix<double> default_rotation_matrix = default_rotation_rpy.ToRotationMatrix();

                    drake::math::RotationMatrix<double> orientation_matrix = drake::math::RotationMatrix<double>(parsed_orientation);

                    // Extract the z-axis (third column) from the orientation matrix
                    Eigen::Vector3d z_axis = orientation_matrix.matrix().col(2);

// Scale the z-axis to 10 cm (0.10 meters)
                    Eigen::Vector3d height_correction = 0.10 * z_axis;

// Add the translation to the parsed position
                    Eigen::Vector3d height_correct_parsed_position = parsed_position - height_correction;

                    // Function to convert RotationMatrix to string and calculate determinant
                    auto matrix_to_string_with_det = [](const drake::math::RotationMatrix<double>& rot_matrix) {
                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(6);  // Set precision for floating-point numbers
                        const auto& matrix = rot_matrix.matrix();
                        for (int i = 0; i < 3; ++i) {
                            for (int j = 0; j < 3; ++j) {
                                ss << matrix(i, j) << " ";
                            }
                            ss << "\n";
                        }
                        double det = matrix.determinant();
                        ss << "Determinant: " << det << "\n";
                        return ss.str();
                    };

                    // Output the default rotation matrix with determinant
                    std::string default_matrix_str = matrix_to_string_with_det(default_rotation_matrix);
                    drake::log()->info("Default Rotation matrix:\n{}", default_matrix_str);

                    // Output the parsed orientation matrix with determinant
                    std::string parsed_matrix_str = matrix_to_string_with_det(orientation_matrix);
                    drake::log()->info("Parsed Orientation matrix:\n{}", parsed_matrix_str);

                    // Create a 90-degree rotation around the z-axis
                    drake::math::RotationMatrix<double> z_rotation = drake::math::RotationMatrix<double>::MakeZRotation(M_PI / 2.0);

                    // Combine default orientation with parsed orientation
                    drake::math::RotationMatrix<double> final_rotation = orientation_matrix * z_rotation;

                    // Output the final rotation matrix with determinant
                    std::string final_matrix_str = matrix_to_string_with_det(final_rotation);
                    drake::log()->info("Final Rotation matrix:\n{}", final_matrix_str);


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

                    std::string with_mimic = R"""(
directives:
- add_model:
    name: spam
    file: package://drake/examples/simple_gripper/mesh.sdf
    default_free_body_pose: { base_link: {
        translation: [0.0, 0.00, 0.0],
        rotation: !Rpy { deg: [0.0, 0.0, 0.0 ]}
    } }

- add_model:
    name: table
    file: package://drake/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table_surface_only_collision.sdf

- add_weld:
    parent: world
    child: table::table_link
    X_PC:
        translation: [0.0, 0.0, -0.77]
)""";

                    parser.AddModelsFromString(with_mimic, "dmd.yaml");
                    parser.AddModelsFromUrl(
                            "package://drake/examples/simple_gripper/robotiq_140_gripper.urdf");
                    plant.WeldFrames(
                            plant.world_frame(),
                            plant.GetBodyByName("robotiq_arg2f_base_link").body_frame(),
                            //math::RigidTransformd(math::RollPitchYawd(M_PI , 0, M_PI),
                            //                      Eigen::Vector3d(0.2, 0, 0.21)));
                            drake::math::RigidTransform<double>(final_rotation, height_correct_parsed_position));

                    plant.Finalize();

                    auto torque = builder.AddSystem<systems::ConstantVectorSource>(Vector1d(15));
                    builder.Connect(torque->get_output_port(), plant.get_actuation_input_port());

                    visualization::AddDefaultVisualization(&builder, meshcat);

                    auto diagram = builder.Build();

                    // Set up simulator.
                    systems::Simulator simulator(*diagram);

                    meshcat->StartRecording(32.0, false);
                    simulator.AdvanceTo(5.0);
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
                        //const Vector3d& face_normal = info.contact_surface().face_normal();
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


                    /*
                    const drake::multibody::RigidBody<double>& object =
                            dynamic_cast<const drake::multibody::RigidBody<double>&>(plant.GetBodyByName("base_link"));
                    // valid names in model instance 'spam' (the to be grasped object) are: base_link;

                    //const drake::multibody::BodyIndex object_body_index = object.index();


                    //const auto& object = plant.GetBodyByName("base_link");
                    const Vector3<double> object_com = object.default_com();

                    // Center of Mass of the to be grasped object
                    std::cout << "  object_com: [" << object_com.x() << ", " << object_com.y() << ", " << object_com.z() << "]" << std::endl;

                    */



                    const drake::multibody::RigidBody<double>& object =
                            dynamic_cast<const drake::multibody::RigidBody<double>&>(plant.GetBodyByName("base_link"));
                    // valid names in model instance 'spam' (the to be grasped object) are: base_link;

                    const drake::multibody::SpatialInertia<double>& spatial_inertia = object.default_spatial_inertia();
                    const Vector3<double> object_com = spatial_inertia.get_com();

                    const auto& X_WO = plant.EvalBodyPoseInWorld(plant_context, object);
                    const Vector3<double> object_com_W = X_WO * object_com;


                    // Center of Mass of the to be grasped object
                    std::cout << "  object_com: [" << object_com_W.x() << ", " << object_com_W.y() << ", " << object_com_W.z() << "]" << std::endl;




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