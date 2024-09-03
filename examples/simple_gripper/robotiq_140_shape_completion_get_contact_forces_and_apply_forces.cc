#include <iostream>
#include <memory>
#include <string>
#include <vector>

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
                    plant.set_discrete_contact_approximation(drake::multibody::DiscreteContactApproximation::kLagged);

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

                    // Create ExternalForceApplicator instance, a Leafsystem which can continually output a force
                    auto external_force_applicator =
                            builder.AddSystem<drake::multibody::ExternalForceApplicator>(&plant);

                    // Add multiple forces to be output and specify time window, force multiplier and force direction
                    external_force_applicator->AddForce(0.0, 20.0, 1.0, Vector3d(0, 0, 1));
                    external_force_applicator->AddForce(5.0, 20.0, 0.001, Vector3d(0, 0, 1));
                    external_force_applicator->AddForce(15.0, 20.0, 10.0, Vector3d(1, 0, 0));

                    // Connect the external force applicator system to the MBP.
                    builder.Connect(external_force_applicator->get_output_port(0),
                                    plant.get_applied_spatial_force_input_port());

                    auto diagram = builder.Build();

                    // Set up the MeshCat simulator in browser
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
