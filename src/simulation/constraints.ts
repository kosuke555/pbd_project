import * as three from 'three';
import { ParticlePositionType } from '../types';
import { ParticleData, ParticleAccessor } from './ParticleData';
import { distance_vectors_vec3, cot_theta, EPSILON } from './math';
import { SimpleAllocator } from '../utils/memory';

export type Constraint = DistanceConstraint | IsometricBendingConstraint;

export interface Stiffnesses {
    compression: number;
    stretch: number;
    bending: number;
}

export type SolverFunction<C extends Constraint> = (this: C, positions: ParticlePositionType,
    inv_mass: Float32Array, stiffnesses: Stiffnesses) => void;

const sqrt = Math.sqrt;

/*
 * Distance Constraint
 */

export interface DistanceConstraint {
    rest_len: number;
    particle1: number;
    particle2: number;
    solver: SolverFunction<DistanceConstraint>;
}

export function create_distance_constraint(particles: ParticleData, particle1: number, particle2: number): DistanceConstraint {
    const rest_len = distance_vectors_vec3(particles.position, particle1, particle2);
    return { rest_len, particle1, particle2, solver: solve_distance_constraint };
}

export function solve_distance_constraint(this: DistanceConstraint, positions: ParticlePositionType,
    inv_mass: Float32Array, stiffnesses: { compression: number, stretch: number }) {

    const constraint = this;
    const p1 = constraint.particle1;
    const p2 = constraint.particle2;
    const rest_len = constraint.rest_len;
    const imass1 = inv_mass[p1];
    const imass2 = inv_mass[p2];
    const sum_imass = imass1 + imass2;

    if (sum_imass == 0.0) return;

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
}

/*
 * Isometric Bending Constraint
 */

export interface IsometricBendingConstraint {
    p_s: number[];
    Q_s: Float32Array;
    solver: SolverFunction<IsometricBendingConstraint>;
}

export const create_isometric_bending_constraint = (() => {
    const vec = new three.Vector3();
    return (particles: ArrayLike<ParticleAccessor>, p1: number, p2: number, p3: number, p4: number): IsometricBendingConstraint => {
        const x_s = [
            new three.Vector3().fromArray(particles[p1].get_position()),
            new three.Vector3().fromArray(particles[p2].get_position()),
            new three.Vector3().fromArray(particles[p3].get_position()),
            new three.Vector3().fromArray(particles[p4].get_position())
        ];

        const e_s = [
            new three.Vector3().subVectors(x_s[0], x_s[1]),
            new three.Vector3().subVectors(x_s[2], x_s[1]),
            new three.Vector3().subVectors(x_s[0], x_s[2]),
            new three.Vector3().subVectors(x_s[0], x_s[3]),
            new three.Vector3().subVectors(x_s[3], x_s[1])
        ];

        const c01 = cot_theta(e_s[0], e_s[1]);
        const c02 = cot_theta(e_s[0], e_s[2]);
        const c03 = cot_theta(e_s[0], e_s[3]);
        const c04 = cot_theta(e_s[0], e_s[4]);

        const K = [c01 + c04, c02 + c03, -c01 - c02, -c03 - c04];
        const Q_s = new Float32Array(16);
        for (let i = 0; i < 4; ++i) {
            for (let j = 0; j < 4; ++j) {
                Q_s[i * 4 + j] = K[i] * K[j];
            }
        }

        const A0 = vec.crossVectors(e_s[0], e_s[1]).length() * 0.5;
        const A1 = vec.crossVectors(e_s[0], e_s[4]).length() * 0.5;
        const coef = 3 / (A0 + A1);
        Q_s.forEach((_, i, Q_s) => Q_s[i] *= coef);

        return { p_s: [p1, p2, p3, p4], Q_s, solver: solve_isometric_bending_constraint };
    };
})();

export const solve_isometric_bending_constraint = (() => {
    const grad_C = new Float32Array(12);
    return function (this: IsometricBendingConstraint, positions: ParticlePositionType,
        inv_mass: Float32Array, stiffnesses: { bending: number }) {

        const constraint = this;
        const p_s = constraint.p_s;
        const Q_s = constraint.Q_s;
        const x = positions;

        let energy = 0;
        for (let i = 0; i < 4; ++i) {
            const i_X = p_s[i] * 3, i_Y = i_X + 1, i_Z = i_X + 2;
            for (let j = 0; j < 4; ++j) {
                const j_X = p_s[j] * 3, j_Y = j_X + 1, j_Z = j_X + 2;
                energy += Q_s[i * 4 + j] * (x[i_X] * x[j_X] + x[i_Y] * x[j_Y] + x[i_Z] * x[j_Z]);
            }
        }
        energy *= 0.5;

        grad_C.fill(0);
        for (let i = 0; i < 4; ++i) {
            const i_X = i * 3, i_Y = i_X + 1, i_Z = i_X + 2;
            for (let j = 0; j < 4; ++j) {
                const j_X = p_s[j] * 3, j_Y = j_X + 1, j_Z = j_X + 2;
                grad_C[i_X] += Q_s[i * 4 + j] * x[j_X];
                grad_C[i_Y] += Q_s[i * 4 + j] * x[j_Y];
                grad_C[i_Z] += Q_s[i * 4 + j] * x[j_Z];
            }
        }

        let grad_C_sum = 0;
        for (let i = 0; i < 4; ++i) {
            const i_X = i * 3, i_Y = i_X + 1, i_Z = i_X + 2;
            const grad_C_x = grad_C[i_X], grad_C_y = grad_C[i_Y], grad_C_z = grad_C[i_Z];
            grad_C_sum += inv_mass[p_s[i]] * (grad_C_x * grad_C_x + grad_C_y * grad_C_y + grad_C_z * grad_C_z);
        }

        if (grad_C_sum < EPSILON) return;

        const s = energy / grad_C_sum;
        const stiff = stiffnesses.bending;
        for (let i = 0; i < 4; ++i) {
            const w_i = inv_mass[p_s[i]];
            if (w_i !== 0.0) {
                const p_i_X = p_s[i] * 3, p_i_Y = p_i_X + 1, p_i_Z = p_i_X + 2;
                const i_X = i * 3, i_Y = i_X + 1, i_Z = i_X + 2;
                positions[p_i_X] += -stiff * s * w_i * grad_C[i_X];
                positions[p_i_Y] += -stiff * s * w_i * grad_C[i_Y];
                positions[p_i_Z] += -stiff * s * w_i * grad_C[i_Z];
            }
        }
    };
})();

/*
 * Contact Constraint
 */

export interface ContactConstraint {
    particle: number;
    normal: Float32Array;
    contact_point: Float32Array;
    contact_normal: Float32Array;
}

export class ContactConstraintAllocator extends SimpleAllocator<ContactConstraint, () => ContactConstraint> {
    constructor() {
        super(() => {
            const buffer = new ArrayBuffer(36);  // 3 float arrays, each has 3 elements
            return {
                particle: 0,
                normal: new Float32Array(buffer, 0, 3),
                contact_point: new Float32Array(buffer, 12, 3),
                contact_normal: new Float32Array(buffer, 24, 3)
            };
        });
    }
}

export function create_contact_constraint(particle: number, n_x: number, n_y: number, n_z: number,
    cp_x: number, cp_y: number, cp_z: number, cn_x: number, cn_y: number, cn_z: number, allocator: ContactConstraintAllocator): ContactConstraint {

    const constraint = allocator.get();
    constraint.particle = particle;
    const normal = constraint.normal;
    normal[0] = n_x, normal[1] = n_y, normal[2] = n_z;
    const contact_point = constraint.contact_point;
    contact_point[0] = cp_x, contact_point[1] = cp_y, contact_point[2] = cp_z;
    const contact_normal = constraint.contact_normal;
    contact_normal[0] = cn_x, contact_normal[1] = cn_y, contact_normal[2] = cn_z;

    return constraint;
}

export function solve_contact_constraint(constraint: ContactConstraint,
    positions: ParticlePositionType, inv_mass: Float32Array,
    contact_normals: Float32Array, depths: Float32Array) {

    const p = constraint.particle;
    if (inv_mass[p] === 0.0) return;

    const C = contact_constraint_func(constraint, positions);
    if (C >= 0.0) return;

    const X = p * 3, Y = X + 1, Z = X + 2;
    const p_x = positions[X];
    const p_y = positions[Y];
    const p_z = positions[Z];
    const normals = constraint.normal;
    const nx = normals[0];
    const ny = normals[1];
    const nz = normals[2];
    positions[X] = p_x - C * nx;
    positions[Y] = p_y - C * ny;
    positions[Z] = p_z - C * nz;
    const cn = constraint.contact_normal;
    contact_normals[X] += cn[0];
    contact_normals[Y] += cn[1];
    contact_normals[Z] += cn[2];
    depths[p] -= C;
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
