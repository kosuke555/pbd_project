import ClothModel from './ClothModel';
import { Stiffnesses } from './constraints';

export interface StepParams {
    step_iter: number,
    max_proj_iter: number,
    stiffnesses: Stiffnesses,
    time_step: number,
    gravity: number
}

export function step(model: ClothModel, params: StepParams) {
    const step_iter = params.step_iter;

    for (let i = 0; i < step_iter; ++i) {
        internal_step(model, params);
    }
}

function internal_step(model: ClothModel, params: StepParams) {
    const n_particle = model.particles.length;
    const position = model.particles.position;
    const old_position = model.particles.old_position;
    const velocity = model.particles.velocity;
    const mass = model.particles.mass;
    const inv_mass = model.particles.inv_mass;
    const constraints = model.constraints;
    const max_proj_iter = params.max_proj_iter;
    const stiffnesses = params.stiffnesses;
    const time_step = params.time_step / params.step_iter;
    const inv_time_step = 1.0 / time_step;
    const gravity = params.gravity;
    const gravity_speed = gravity * time_step;

    for (let i = 0, len = position.length; i < len; i += 3) {
        old_position[i] = position[i];
        old_position[i + 1] = position[i + 1];
        old_position[i + 2] = position[i + 2];
    }

    // position predictions
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            velocity[i * 3 + 1] += gravity_speed;
            const v_x = velocity[i * 3] * time_step;
            const v_y = velocity[i * 3 + 1] * time_step;
            const v_z = velocity[i * 3 + 2] * time_step;
            position[i * 3] += v_x;
            position[i * 3 + 1] += v_y;
            position[i * 3 + 2] += v_z;
        }
    }

    // project constraints
    for (let i = 0; i < max_proj_iter; ++i) {
        for (let j = 0, len = constraints.length; j < len; ++j) {
            constraints[j].solver(constraints[j], position, inv_mass, stiffnesses);
        }
    }

    // update velocities
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            let v_x = position[i * 3] - old_position[i * 3];
            let v_y = position[i * 3 + 1] - old_position[i * 3 + 1];
            let v_z = position[i * 3 + 2] - old_position[i * 3 + 2];
            v_x *= inv_time_step;
            v_y *= inv_time_step;
            v_z *= inv_time_step;
            velocity[i * 3] = v_x;
            velocity[i * 3 + 1] = v_y;
            velocity[i * 3 + 2] = v_z;
        }
    }
}
