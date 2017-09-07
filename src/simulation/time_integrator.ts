import * as three from 'three';
import { ParticlePositionType } from '../types';
import ClothModel from './ClothModel';
import { Stiffnesses, prestabilize_contact_constraint, solve_contact_constraint, ContactConstraint } from './constraints';
import { CollisionObjects, generate_collision_constraints, CollisionTolerances } from './collision_detection';

export interface SimulationConfiguration {
    step_iter: number,
    max_proj_iter: number,
    pre_stabilize_iter: number,
    stiffnesses: Stiffnesses,
    time_step: number,
    gravity: number,
    velocity_damp_factor: number,
    collision_tolerances: CollisionTolerances
}

let prev_coll_consts = [] as ContactConstraint[];
export function step(model: ClothModel, collision_objects: CollisionObjects, config: SimulationConfiguration) {
    const step_iter = config.step_iter;

    for (let i = 0; i < step_iter; ++i) {
        prev_coll_consts = internal_step(model, collision_objects, prev_coll_consts, config);
    }
}

function internal_step(model: ClothModel, collision_objects: CollisionObjects,
    prev_collision_constraints: ContactConstraint[], config: SimulationConfiguration) {

    const particles = model.particles;
    const n_particle = particles.length;
    const position = particles.position;
    const old_position = particles.old_position;
    const delta_position = particles.delta_position;
    const velocity = particles.velocity;
    const n_collision = particles.n_collision;
    const r_vec = particles.r_vec;
    const mass = particles.mass;
    const inv_mass = particles.inv_mass;
    const constraints = model.constraints;
    const max_proj_iter = config.max_proj_iter;
    const pre_stabilize_iter = config.pre_stabilize_iter;
    const stiffnesses = config.stiffnesses;
    const time_step = config.time_step / config.step_iter;
    const inv_time_step = 1.0 / time_step;
    const gravity = config.gravity;
    const gravity_speed = gravity * time_step;
    const velocity_damp_factor = config.velocity_damp_factor;
    const c_tolerances = config.collision_tolerances;

    // integrate velocity
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            const Y = i * 3 + 1;
            velocity[Y] += gravity_speed;
        }
    }

    velocity_damping(velocity, position, mass, r_vec, n_particle, velocity_damp_factor);

    // position predictions
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            old_position[X] = position[X];
            old_position[Y] = position[Y];
            old_position[Z] = position[Z];
            const d_x = velocity[X] * time_step;
            const d_y = velocity[Y] * time_step;
            const d_z = velocity[Z] * time_step;
            position[X] += d_x;
            position[Y] += d_y;
            position[Z] += d_z;
        }
    }

    // pre-stabilization
    if (prev_collision_constraints.length) {
        for (let i = 0; i < pre_stabilize_iter; ++i) {
            delta_position.fill(0);
            n_collision.fill(0);
            for (let j = 0, len = prev_collision_constraints.length; j < len; ++j) {
                prestabilize_contact_constraint(prev_collision_constraints[j],
                    old_position, delta_position, n_collision, inv_mass);
            }
            for (let j = 0; j < n_particle; ++j) {
                const X = j * 3, Y = X + 1, Z = X + 2;
                const n = n_collision[j];
                if (n === 0) continue;
                const inv_n = 1 / n;
                old_position[X] += delta_position[X] * inv_n;
                old_position[Y] += delta_position[Y] * inv_n;
                old_position[Z] += delta_position[Z] * inv_n;
                position[X] += delta_position[X] * inv_n;
                position[Y] += delta_position[Y] * inv_n;
                position[Z] += delta_position[Z] * inv_n;
            }
        }
    }

    // generate collision constraints
    const collision_constraints = generate_collision_constraints(particles, collision_objects, c_tolerances);

    // project constraints
    for (let i = 0; i < max_proj_iter; ++i) {
        for (let j = 0, len = collision_constraints.length; j < len; ++j) {
            solve_contact_constraint(collision_constraints[j], position, inv_mass);
        }
        for (let j = 0, len = constraints.length; j < len; ++j) {
            constraints[j].solver(constraints[j], position, inv_mass, stiffnesses);
        }
    }

    // update velocities
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            let v_x = position[X] - old_position[X];
            let v_y = position[Y] - old_position[Y];
            let v_z = position[Z] - old_position[Z];
            v_x *= inv_time_step;
            v_y *= inv_time_step;
            v_z *= inv_time_step;
            velocity[X] = v_x;
            velocity[Y] = v_y;
            velocity[Z] = v_z;
        }
    }

    return collision_constraints;
}

const velocity_damping = (() => {
    const l_vec = new three.Vector3();
    const r_mat = new three.Matrix3();
    const r_mat2 = new three.Matrix3();
    const i_mat = new three.Matrix3();
    const inv_mat = new three.Matrix3();
    return (velocities: Float32Array, positions: ParticlePositionType,
        masses: Float32Array, r_vec: Float32Array, n_particle: number, k_damping: number) => {

        let p_cm_x = 0;
        let p_cm_y = 0;
        let p_cm_z = 0;
        let v_cm_x = 0;
        let v_cm_y = 0;
        let v_cm_z = 0;
        let sum_mass = 0;
    
        for (let i = 0; i < n_particle; ++i) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            const mass = masses[i];
            p_cm_x += positions[X] * mass;
            p_cm_y += positions[Y] * mass;
            p_cm_z += positions[Z] * mass;
            v_cm_x += velocities[X] * mass;
            v_cm_y += velocities[Y] * mass;
            v_cm_z += velocities[Z] * mass;
            sum_mass += mass;
        }
    
        p_cm_x /= sum_mass;
        p_cm_y /= sum_mass;
        p_cm_z /= sum_mass;
        v_cm_x /= sum_mass;
        v_cm_y /= sum_mass;
        v_cm_z /= sum_mass;
    
        l_vec.set(0, 0, 0);
        i_mat.identity();
        const r_elm = r_mat.elements;
        const i_elm = i_mat.elements;
    
        for (let i = 0; i < n_particle; ++i) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            const r_x = r_vec[X] = positions[X] - p_cm_x;
            const r_y = r_vec[Y] = positions[Y] - p_cm_y;
            const r_z = r_vec[Z] = positions[Z] - p_cm_z;
            const mass = masses[i];
            const mv_x = velocities[X] * mass;
            const mv_y = velocities[Y] * mass;
            const mv_z = velocities[Z] * mass;
            l_vec.x += r_y * mv_z - r_z * mv_y;
            l_vec.y += r_z * mv_x - r_x * mv_z;
            l_vec.z += r_x * mv_y - r_y * mv_x;
            r_mat.set(0, -r_z, r_y, r_z, 0, -r_x, -r_y, r_x, 0);
            r_mat2.copy(r_mat).transpose();
            r_mat.multiply(r_mat2).multiplyScalar(mass);
            for (let k = 0; k < 9; ++k) i_elm[k] += r_elm[k];
        }
        
        inv_mat.getInverse(i_mat);
        l_vec.applyMatrix3(inv_mat);
    
        for (let i = 0; i < n_particle; ++i) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            const r_x = r_vec[X];
            const r_y = r_vec[Y];
            const r_z = r_vec[Z];
            const dv_x = v_cm_x + (l_vec.y * r_z + l_vec.z * r_y) - velocities[X];
            const dv_y = v_cm_y + (l_vec.z * r_x + l_vec.x * r_z) - velocities[Y];
            const dv_z = v_cm_z + (l_vec.x * r_y + l_vec.y * r_x) - velocities[Z];
            velocities[X] += k_damping * dv_x;
            velocities[Y] += k_damping * dv_y;
            velocities[Z] += k_damping * dv_z;
        }
    };
})();
