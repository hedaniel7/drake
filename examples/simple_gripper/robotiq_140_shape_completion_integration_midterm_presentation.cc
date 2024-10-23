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


namespace drake {
    namespace multibody {

        // Class definition of the Leafsystem which should output multiple forces
        class ExternalForceApplicator : public systems::LeafSystem<double> {
        public:
            explicit ExternalForceApplicator(const MultibodyPlant<double>* plant);

            void AddForce(double start_time, double end_time, double force_magnitude, const Eigen::Vector3d& force_direction);

        private:
            void CalcSpatialForceOutput(
                    [[maybe_unused]] const systems::Context<double>& context,
                    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const;

            const MultibodyPlant<double>* plant_{nullptr};
            std::vector<double> start_times_;
            std::vector<double> end_times_;
            std::vector<double> force_magnitudes_;
            std::vector<Eigen::Vector3d> force_directions_;
        };

        // Constructor of class in which we define the callback function of the output port
        // which returns the output of our Leafsytem
        ExternalForceApplicator::ExternalForceApplicator(const MultibodyPlant<double>* plant)
                : plant_(plant) {
            this->DeclareAbstractOutputPort(
                    "spatial_force_output",
                    &ExternalForceApplicator::CalcSpatialForceOutput);
        }

        // Because - from my understanding now, I could be wrong here - Drake only allows connecting one system to
        // plant.get_applied_spatial_force_input_port()), we can't design the Leafsystem class here to be instantiated
        // multiple times for multiple forces. I tried it myself and got:
        /*
        // We changed the class and the constructor in this hypothetical failing example to take the arguments
        // in the constructor
        auto external_force_applicator_1 =
        builder.AddSystem<drake::multibody::ExternalForceApplicator>(&plant, 7.0, 12.0, 10.0, Vector3d(0, 0, 1));
        auto external_force_applicator_2 =
        builder.AddSystem<drake::multibody::ExternalForceApplicator>(&plant, 15.0, 20.0, 10.0, Vector3d(1, 0, 0));


        builder.Connect(external_force_applicator_1->get_output_port(0),
        plant.get_applied_spatial_force_input_port());
        builder.Connect(external_force_applicator_2->get_output_port(0),
        plant.get_applied_spatial_force_input_port());

        This would yield the following errro:
        abort: Failure at systems/framework/diagram_builder.cc:453 in ThrowIfInputAlreadyWired():
        condition 'iter != input_port_ids_.end()' failed.
        */
        // Because of that, I chose to only instantiate one Leafsystem class and add multiple forces to be output from
        // the output of this one Leafsystem class and specify their time window, magnitude and direction in this method
        void ExternalForceApplicator::AddForce(double start_time, double end_time, double force_magnitude, const Eigen::Vector3d& force_direction) {
            start_times_.push_back(start_time);
            end_times_.push_back(end_time);
            force_magnitudes_.push_back(force_magnitude);
            force_directions_.push_back(force_direction);
        }

        // This method specifies what is output from this Leafsystem continually
        //
        // Remark: The compiler throws and unused error if the context is not used. It is used here anyway, though.
        void ExternalForceApplicator::CalcSpatialForceOutput(
                [[maybe_unused]] const systems::Context<double>& context,
                std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const {

            const double current_time = context.get_time();
            output->clear();

            const RigidBody<double>& object =
                    dynamic_cast<const RigidBody<double>&>(plant_->GetBodyByName("base_link"));
            // valid names in model instance 'spam' (the to be grasped object) are: base_link;

            const BodyIndex object_body_index = object.index();
            const Vector3<double> object_com = object.default_com();
            const double g = UniformGravityFieldElement<double>::kDefaultStrength;

            // We look through all the time windows and see if we output a force at the given simulation time
            for (size_t i = 0; i < start_times_.size(); ++i) {
                if (current_time >= start_times_[i] && current_time <= end_times_[i]) {
                    const SpatialForce<double> F_object_com_W(Vector3<double>::Zero() /* no torque */,
                                                              object.default_mass() * g * force_magnitudes_[i] * force_directions_[i]);

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

                    std::cout << "Position: " << position_str << std::endl;
                    std::cout << "Orientation: " << orientation_str << std::endl;
                    std::cout << "Gripper Opening: " << gripper_opening << " meters" << std::endl;
                    std::cout << "Manual Correction: " << manual_correction << " meters" << std::endl;
                    std::cout << "Table Correction: " << table_correction << " meters" << std::endl; // Display table_correction

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

                    // Add the translation to the parsed position
                    Eigen::Vector3d height_correct_parsed_position = parsed_position - height_correction;

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
                    double base_table_height = -0.768;
                    double total_table_height = base_table_height + table_correction;
                    // Update the translation in the YAML string
                    std::string with_mimic = fmt::format(R"""(
directives:
- add_model:
    name: spam
    file: package://drake/examples/simple_gripper/midterm_presentation/obj2/obj2_mesh.sdf
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
)""", total_table_height);

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

                    // Add force to be output and specify time window, force multiplier and force direction
                    external_force_applicator->AddForce(0.0, 0.2, 1.0, Vector3d(0, 0, 1));

                    // Connect the external force applicator system to the MBP.
                    builder.Connect(external_force_applicator->get_output_port(0),
                                    plant.get_applied_spatial_force_input_port());

                    auto diagram = builder.Build();

                    // Set up simulator.
                    systems::Simulator simulator(*diagram);

                    meshcat->StartRecording(32.0, false);
                    simulator.AdvanceTo(1.0);
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
