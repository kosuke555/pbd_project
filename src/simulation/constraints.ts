import { ParticleData } from './ParticleData';
import { distance_vectors_vec3, copy_vec3_gen, sub_vec3, length_vec3,
    set_vec3, mul_scalar_to_vec3, mul_scalar_vec3, add_vec3 } from './math';

export type Constraint = DistanceConstraint;

export interface Stiffnesses {
    compression: number;
    stretch: number;
}

export type SolverFunction = (constraint: Constraint, positions: Float32Array,
    inv_mass: Float32Array, stiffnesses: Stiffnesses) => boolean;

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
export function solve_distance_constraint(constraint: DistanceConstraint, positions: Float32Array,
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

const p1 = new Float32Array(3);
const p2 = new Float32Array(3);
const d_vec = new Float32Array(3);
const n_vec = new Float32Array(3);
const c_vec = new Float32Array(3);
export function solve_distance_constraint2(constraint: DistanceConstraint, positions: Float32Array,
    inv_mass: Float32Array, stiffnesses: { compression: number, stretch: number }) {

    const p1_index = constraint.particle1;
    const p2_index = constraint.particle2;
    const rest_len = constraint.rest_len;
    const imass1 = inv_mass[p1_index];
    const imass2 = inv_mass[p2_index];
    const sum_imass = imass1 + imass2;

    if (sum_imass == 0.0) return false;

    copy_vec3_gen(p1, 0, positions, p1_index);
    copy_vec3_gen(p2, 0, positions, p2_index);
    sub_vec3(p2, p1, d_vec, 0);
    const d = length_vec3(d_vec);

    set_vec3(n_vec, d_vec[0] / d, d_vec[1] / d, d_vec[2] / d);

    const stiff = d < rest_len ? stiffnesses.compression : stiffnesses.stretch;
    const a = stiff * (d - rest_len) / sum_imass;
    mul_scalar_to_vec3(n_vec, a);

    if (imass1 !== 0.0) {
        mul_scalar_vec3(n_vec, imass1, c_vec);
        add_vec3(p1, c_vec, positions, p1_index);
    }
    if (imass2 !== 0.0) {
        mul_scalar_vec3(n_vec, -imass2, c_vec);
        add_vec3(p2, c_vec, positions, p2_index);
    }

    return true;
}
