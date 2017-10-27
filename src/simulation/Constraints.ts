import { ParticlePositionType } from '../types';
import { ParticleData } from './ParticleData';
import { distance_vectors_vec3 } from './math';

export type Constraint = DistanceConstraint;

export interface Stiffnesses {
    compression: number;
    stretch: number;
}

export type SolverFunction = (constraint: Constraint, positions: ParticlePositionType,
    inv_mass: Float32Array, stiffnesses: Stiffnesses) => boolean;

/*
 * Distance Constraint
 */

export interface DistanceConstraint {
    rest_len: number;
    particle1: number;
    particle2: number;
    solver: SolverFunction;
}

export function create_distance_constraint(particles: ParticleData, particle1: number, particle2: number): DistanceConstraint {
    const rest_len = distance_vectors_vec3(particles.position, particle1, particle2);
    return { rest_len, particle1, particle2, solver: solve_distance_constraint };
}

const sqrt = Math.sqrt;
export function solve_distance_constraint(constraint: DistanceConstraint, positions: ParticlePositionType,
    inv_mass: Float32Array, stiffnesses: { compression: number, stretch: number }) {

    const p1 = constraint.particle1;
    const p2 = constraint.particle2;
    const rest_len = constraint.rest_len;
    const imass1 = inv_mass[p1];
    const imass2 = inv_mass[p2];
    const sum_imass = imass1 + imass2;

    if (sum_imass == 0.0) return false;

    const p1_x = positions[p1 * 3];
    const p1_y = positions[p1 * 3 + 1];
    const p1_z = positions[p1 * 3 + 2];
    const p2_x = positions[p2 * 3];
    const p2_y = positions[p2 * 3 + 1];
    const p2_z = positions[p2 * 3 + 2];
    const x = p2_x - p1_x;
    const y = p2_y - p1_y;
    const z = p2_z - p1_z;
    const d = sqrt(x * x + y * y + z * z);

    const nx = x / d;
    const ny = y / d;
    const nz = z / d;

    const stiff = d < rest_len ? stiffnesses.compression : stiffnesses.stretch;
    const a = stiff * (d - rest_len) / sum_imass;
    const cx = a * nx;
    const cy = a * ny;
    const cz = a * nz;

    if (imass1 !== 0.0) {
        positions[p1 * 3] = p1_x + imass1 * cx;
        positions[p1 * 3 + 1] = p1_y + imass1 * cy;
        positions[p1 * 3 + 2] = p1_z + imass1 * cz;
    }
    if (imass2 !== 0.0) {
        positions[p2 * 3] = p2_x + -imass2 * cx;
        positions[p2 * 3 + 1] = p2_y + -imass2 * cy;
        positions[p2 * 3 + 2] = p2_z + -imass2 * cz;
    }

    return true;
}

/*
 * Contact Constraint
 */

export interface ContactConstraint {
    particle: number;
    normal: Float32Array;
    contact_point: Float32Array;
    contact_normal: Float32Array;
}

export function create_contact_constraint(particle: number, norm: number[], c_point: number[], c_norm: number[]): ContactConstraint {
    const normal = new Float32Array(norm);
    const contact_point = new Float32Array(c_point);
    const contact_normal = new Float32Array(c_norm);

    return { particle, normal, contact_point, contact_normal };
}

export function solve_contact_constraint(constraint: ContactConstraint,
    positions: ParticlePositionType, inv_mass: Float32Array) {

    const p = constraint.particle;
    if (inv_mass[p] === 0.0) return;

    const C = contact_constraint_func(constraint, positions);
    if (C >= 0.0) return;

    const p_x = positions[p * 3];
    const p_y = positions[p * 3 + 1];
    const p_z = positions[p * 3 + 2];
    const normals = constraint.normal;
    const nx = normals[0];
    const ny = normals[1];
    const nz = normals[2];
    positions[p * 3] = p_x - C * nx;
    positions[p * 3 + 1] = p_y - C * ny;
    positions[p * 3 + 2] = p_z - C * nz;
}

export function prestabilize_contact_constraint(constraint: ContactConstraint, positions: ParticlePositionType,
    delta_positions: ParticlePositionType, n_collisions: Uint8Array, inv_mass: Float32Array) {

    const p = constraint.particle;
    if (inv_mass[p] === 0.0) return;

    const C = contact_constraint_func(constraint, positions);
    if (C >= 0.0) return;

    const normals = constraint.normal;
    const nx = normals[0];
    const ny = normals[1];
    const nz = normals[2];
    delta_positions[p * 3] += -C * nx;
    delta_positions[p * 3 + 1] += -C * ny;
    delta_positions[p * 3 + 2] += -C * nz;
    ++n_collisions[p];
}

function contact_constraint_func(constraint: ContactConstraint, positions: ParticlePositionType) {
    const p = constraint.particle;
    const p_x = positions[p * 3];
    const p_y = positions[p * 3 + 1];
    const p_z = positions[p * 3 + 2];
    const contact_point = constraint.contact_point;
    const cp_x = contact_point[0];
    const cp_y = contact_point[1];
    const cp_z = contact_point[2];
    const contact_normal = constraint.contact_normal;
    const cn_x = contact_normal[0];
    const cn_y = contact_normal[1];
    const cn_z = contact_normal[2];

    return (p_x - cp_x) * cn_x + (p_y - cp_y) * cn_y + (p_z - cp_z) * cn_z;
}

// export interface ContactConstraints {
//     particles: IndexArrayType;
//     normals: Float32Array;
//     contact_points: Float32Array;
//     contact_normals: Float32Array;
//     length: number;
// }

// export function create_contact_constraints(n_constraints: number, indices_ctor: TypedArrayConstructor<IndexArrayType>): ContactConstraints {
//     const particles = new indices_ctor(n_constraints);
//     const normals = new Float32Array(n_constraints * 3);
//     const contact_points = new Float32Array(n_constraints * 3);
//     const contact_normals = new Float32Array(n_constraints * 3);

//     return { particles, normals, contact_points, contact_normals, length: n_constraints };
// }

// export function project_contact_constraint(constraints: ContactConstraints, positions: ParticlePositionType, inv_mass: Float32Array) {
//     for (let i = 0, len = constraints.length; i < len; ++i) {
//         const pi = constraints.particles[i];
//         if (inv_mass[pi] === 0.0) return;
//         const pi_x = positions[pi * 3];
//         const pi_y = positions[pi * 3 + 1];
//         const pi_z = positions[pi * 3 + 2];
//         const normals = constraints.normals;
//         const nx = normals[i * 3];
//         const ny = normals[i * 3 + 1];
//         const nz = normals[i * 3 + 2];
//         const contact_points = constraints.contact_points;
//         const cp_x = contact_points[i * 3];
//         const cp_y = contact_points[i * 3 + 1];
//         const cp_z = contact_points[i * 3 + 2];
//         const contact_normals = constraints.contact_normals;
//         const cn_x = contact_normals[i * 3];
//         const cn_y = contact_normals[i * 3 + 1];
//         const cn_z = contact_normals[i * 3 + 2];
//         const dot = (pi_x - cp_x) * cn_x + (pi_y - cp_y) * cn_y + (pi_z - cp_z) * cn_z;

//         positions[pi * 3] = pi_x - dot * nx;
//         positions[pi * 3 + 1] = pi_y - dot * ny;
//         positions[pi * 3 + 2] = pi_z - dot * nz;
//     }
// }
