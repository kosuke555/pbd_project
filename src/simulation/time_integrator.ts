import * as three from 'three';
import { ParticlePositionType } from '../types';
import ClothModel from './ClothModel';
import { Constraint, Stiffnesses, ContactConstraint,
    prestabilize_contact_constraint, solve_contact_constraint } from './constraints';
import { CollisionObjects, CollisionDetector } from './collision_detection';
import { normalize_vec3_gen } from './math';

export interface SimulationConfiguration {
    step_iter: number;
    max_proj_iter: number;
    pre_stabilize_iter: number;
    stiffnesses: Stiffnesses;
    time_step: number;
    gravity: number;
    velocity_damp_factor: number;
    static_friction_coeff: number;
    kinetic_friction_coeff: number;
}

const sqrt = Math.sqrt;
const min = Math.min;

export const step = (() => {
    let prev_coll_consts = [] as ContactConstraint[];
    return (model: ClothModel, objects: CollisionObjects,
        detector: CollisionDetector, config: SimulationConfiguration) => {

        const step_iter = config.step_iter;
        for (let i = 0; i < step_iter; ++i) {
            prev_coll_consts = internal_step(model, objects, detector, prev_coll_consts, config);
        }
    };
})();

function internal_step(model: ClothModel, objects: CollisionObjects, detector: CollisionDetector,
    prev_collision_constraints: ContactConstraint[], config: SimulationConfiguration) {

    const particles = model.particles;
    const n_particle = particles.length;
    const position = particles.position;
    const old_position = particles.old_position;
    const velocity = particles.velocity;
    const mass = particles.mass;
    const inv_mass = particles.inv_mass;
    const time_step = config.time_step / config.step_iter;

    // integrate velocity
    const gravity = config.gravity;
    integrate_velocity(velocity, mass, n_particle, gravity, time_step);

    // velocity damping
    const r_vec = particles.extra_vec;
    const velocity_damp_factor = config.velocity_damp_factor;
    velocity_damping(velocity, position, mass, r_vec, n_particle, velocity_damp_factor);

    // position predictions
    position_predictions(position, old_position, mass, velocity, n_particle, time_step);

    // pre-stabilization
    if (prev_collision_constraints.length) {
        const delta_positions = particles.extra_vec;
        const n_collisions = particles.extra_uint8;
        for (let i = 0, len = config.pre_stabilize_iter; i < len; ++i) {
            pre_stabilization(position, old_position, prev_collision_constraints,
                inv_mass, n_particle, delta_positions, n_collisions);
        }
    }

    // generate collision constraints
    detector.release_collision_constraints();
    const collision_constraints = detector.generate_collision_constraints(particles, objects);

    // project constraints
    const contact_normals = particles.extra_vec;
    const depths = particles.extra_float;
    const static_friction_coeff = config.static_friction_coeff;
    const kinetic_friction_coeff = config.kinetic_friction_coeff;
    for (let i = 0, len = config.max_proj_iter; i < len; ++i) {
        project_constraint(model.constraints, collision_constraints,
            config.stiffnesses, position, inv_mass, contact_normals, depths);

        apply_friction(position, old_position, n_particle,
            contact_normals, depths, static_friction_coeff, kinetic_friction_coeff);
    }

    // update velocity
    update_velocity(velocity, position, old_position, mass, n_particle, time_step);

    return collision_constraints;
}

function integrate_velocity(velocity: Float32Array, mass: Float32Array,
    n_particle: number, gravity: number, time_step: number) {

    const gravity_speed = gravity * time_step;
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            const Y = i * 3 + 1;
            velocity[Y] += gravity_speed;
        }
    }
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

function position_predictions(position: ParticlePositionType, old_position: ParticlePositionType,
    mass: Float32Array, velocity: Float32Array, n_particle: number, time_step: number) {

    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            old_position[X] = position[X];
            old_position[Y] = position[Y];
            old_position[Z] = position[Z];
            position[X] += velocity[X] * time_step;
            position[Y] += velocity[Y] * time_step;
            position[Z] += velocity[Z] * time_step;
        }
    }
}

function pre_stabilization(position: ParticlePositionType, old_position: ParticlePositionType,
    prev_collision_constraints: ContactConstraint[], inv_mass: Float32Array, n_particle: number,
    delta_positions: ParticlePositionType, n_collisions: Uint8Array) {

    delta_positions.fill(0);
    n_collisions.fill(0);
    for (let j = 0, len = prev_collision_constraints.length; j < len; ++j) {
        prestabilize_contact_constraint(prev_collision_constraints[j],
            old_position, delta_positions, n_collisions, inv_mass);
    }
    for (let j = 0; j < n_particle; ++j) {
        const X = j * 3, Y = X + 1, Z = X + 2;
        const n = n_collisions[j];
        if (n === 0) continue;
        const inv_n = 1 / n;
        old_position[X] += delta_positions[X] * inv_n;
        old_position[Y] += delta_positions[Y] * inv_n;
        old_position[Z] += delta_positions[Z] * inv_n;
        position[X] += delta_positions[X] * inv_n;
        position[Y] += delta_positions[Y] * inv_n;
        position[Z] += delta_positions[Z] * inv_n;
    }
}

function project_constraint(constraints: Constraint[], collision_constraints: ContactConstraint[], stiffnesses: Stiffnesses,
    position: ParticlePositionType, inv_mass: Float32Array, contact_normals: Float32Array, depths: Float32Array) {

    contact_normals.fill(0);
    depths.fill(0);

    for (let j = 0, len = constraints.length; j < len; ++j) {
        constraints[j].solver(position, inv_mass, stiffnesses);
    }

    for (let j = 0, len = collision_constraints.length; j < len; ++j) {
        solve_contact_constraint(collision_constraints[j], position, inv_mass, contact_normals, depths);
    }
}

function apply_friction(position: ParticlePositionType, old_position: ParticlePositionType, n_particle: number,
    contact_normals: Float32Array, depths: Float32Array, static_friction_coeff: number, kinetic_friction_coeff: number) {

    for (let j = 0; j < n_particle; ++j) {
        if (depths[j] > 0) {
            normalize_vec3_gen(contact_normals, j);
            const X = j * 3, Y = X + 1, Z = X + 2;
            const cn_x = contact_normals[X];
            const cn_y = contact_normals[Y];
            const cn_z = contact_normals[Z];
            const d_x = position[X] - old_position[X];
            const d_y = position[Y] - old_position[Y];
            const d_z = position[Z] - old_position[Z];
            const dot = d_x * cn_x + d_y * cn_y + d_z * cn_z;
            const dis_x = -d_x + (cn_x * dot);
            const dis_y = -d_y + (cn_y * dot);
            const dis_z = -d_z + (cn_z * dot);
            const dis_len = sqrt((dis_x * dis_x) + (dis_y * dis_y) + (dis_z * dis_z));
            const depth = depths[j];
            if (static_friction_coeff * depth > dis_len) {
                position[X] += dis_x;
                position[Y] += dis_y;
                position[Z] += dis_z;
            } else {
                const coeff = min((kinetic_friction_coeff * depth) / dis_len, 1.0);
                position[X] += dis_x * coeff;
                position[Y] += dis_y * coeff;
                position[Z] += dis_z * coeff;
            }
        }
    }
}

function update_velocity(velocity: Float32Array,
    position: ParticlePositionType, old_position: ParticlePositionType, mass: Float32Array,
    n_particle: number, time_step: number) {

    const inv_time_step = 1.0 / time_step;
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
}
