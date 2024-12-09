from static_experiment import run_static_experiment
from dynamic_experiment_joint_space import run_dynamic_experiment
from plot import plot_torques, plot_cheat_torques, plot_torques_analytic

model_id = 1

## Example launch of static experiments

# run_static_experiment(model_id=model_id)
# plot_torques(model_id=model_id, static=True)
# plot_cheat_torques(model_id=model_id, static=True)
# plot_torques_analytic(model_id=model_id, static=True)

run_dynamic_experiment(model_id=model_id)
plot_torques(model_id=model_id, static=False)
plot_cheat_torques(model_id=model_id, static=False)
plot_torques_analytic(model_id=model_id, static=False)