#include <iostream>
#include <memory>
#include <string>

#include <fmt/format.h>

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

namespace drake {
    namespace multibody {

        class ExternalForceApplicator : public systems::LeafSystem<double> {
        public:
            explicit ExternalForceApplicator(const MultibodyPlant<double>* plant);

        private:
            void CalcSpatialForceOutput(
                    [[maybe_unused]] const systems::Context<double>& context,
                    std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const;

            const MultibodyPlant<double>* plant_{nullptr};
        };

        ExternalForceApplicator::ExternalForceApplicator(const MultibodyPlant<double>* plant)
                : plant_(plant) {
            this->DeclareAbstractOutputPort(
                    "spatial_force_output",
                    &ExternalForceApplicator::CalcSpatialForceOutput);
        }

        void ExternalForceApplicator::CalcSpatialForceOutput(
                [[maybe_unused]] const systems::Context<double>& context,
                std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>* output) const {

            const double g = UniformGravityFieldElement<double>::kDefaultStrength;

            const RigidBody<double>& object =
                    dynamic_cast<const RigidBody<double>&>(plant_->GetBodyByName("base_link"));

            const BodyIndex object_body_index = object.index();
            const Vector3<double> object_com = object.default_com();

            const Vector3<double> up_W(0, 0, 1);
            const SpatialForce<double> F_object_com_W(Vector3<double>::Zero() /* no torque */,
                                                      object.default_mass() * g * up_W);

            output->resize(1 /* number of forces */);
            (*output)[0].body_index = object_body_index;
            (*output)[0].p_BoBq_B = object_com;
            (*output)[0].F_Bq_W = F_object_com_W;
        }

    }  // namespace multibody
}  // namespace drake

namespace drake {
    namespace examples {
        namespace simple_gripper {

            using Eigen::Vector3d;
            using multibody::ContactResults;
            using multibody::HydroelasticContactInfo;
            namespace {

                int do_main() {
                    auto meshcat = std::make_shared<geometry::Meshcat>();
                    systems::DiagramBuilder<double> builder;

                    auto [plant, scene_graph] =
                            multibody::AddMultibodyPlantSceneGraph(&builder, 0.002);
                    plant.set_discrete_contact_approximation( drake::multibody::DiscreteContactApproximation::kLagged);

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
        translation: [-0.2, 0.05, 0.00],
        rotation: !Rpy { deg: [90.0, 0.0, 0.0 ]}
    } }

- add_model:
    name: table
    file: package://drake/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table_surface_only_collision.sdf

- add_weld:
    parent: world
    child: table::table_link
    X_PC:
        translation: [0.0, 0.0, -0.81]
)""";

                    parser.AddModelsFromString(with_mimic, "dmd.yaml");
                    parser.AddModelsFromUrl(
                            "package://drake/examples/simple_gripper/robotiq_140_gripper.urdf");
                    plant.WeldFrames(
                            plant.world_frame(),
                            plant.GetBodyByName("robotiq_arg2f_base_link").body_frame(),
                            math::RigidTransformd(math::RollPitchYawd(M_PI , 0, M_PI),
                                                  Eigen::Vector3d(0.2, 0, 0.21)));

                    plant.Finalize();

                    auto torque = builder.AddSystem<systems::ConstantVectorSource>(Vector1d(2));
                    builder.Connect(torque->get_output_port(), plant.get_actuation_input_port());

                    visualization::AddDefaultVisualization(&builder, meshcat);

                    // Apply a force to the grasped object

                    // Create an ExternalForceApplicator instance
                    auto external_force_applicator_ =
                            builder.AddSystem<drake::multibody::ExternalForceApplicator>(&plant);

                    // Connect the external force applicator system to the MBP.
                    builder.Connect(external_force_applicator_->get_output_port(0),
                                    plant.get_applied_spatial_force_input_port());

                    auto diagram = builder.Build();

                    // Set up simulator.
                    systems::Simulator simulator(*diagram);

                    meshcat->StartRecording(32.0, false);
                    simulator.AdvanceTo(20.0);
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
                        const Vector3d& tau_Ac_W = info.F_Ac_W().rotational();

                        std::cout << "Contact " << i << ":" << std::endl;
                        // Force applied on body A, at the centroid point C, expressed in the world frame W
                        std::cout << "  F_Ac_W: [" << F_Ac_W.x() << ", " << F_Ac_W.y() << ", " << F_Ac_W.z() << "]" << std::endl;
                        // Position p_WC of the centroid point C in the world frame W
                        std::cout << "  p_WC: [" << p_WC.x() << ", " << p_WC.y() << ", " << p_WC.z() << "]" << std::endl;
                        // Moment
                        std::cout << "  tau_Ac_W: [" << tau_Ac_W.x() << ", " << tau_Ac_W.y() << ", " << tau_Ac_W.z() << "]" << std::endl;
                    }

                    // Pause so that you can see the meshcat output.
                    std::cout << "[Press Ctrl-C to finish]." << std::endl;
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                    return 0;
                }

            }  // namespace
        }  // namespace simple_gripper
    }  // namespace examples
}  // namespace drake

int main(int, char*[]) {
    return drake::examples::simple_gripper::do_main();
}
