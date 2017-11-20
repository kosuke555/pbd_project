import * as three from 'three';
import * as ammo from 'ammo.js';
import { get_vec3_array_gen, set_vec3_gen } from './math';

const min = Math.min;
const max = Math.max;

export interface SphereRigidBodies {
    bodies: ammo.btRigidBody[];
    radiuses: Float32Array;
    positions: Float32Array;
    AABBs: Float32Array;  // [min,max], [min,max], ...
    tolerances: Float32Array;
    length: number;
}

export interface SphereRigidBodyAccessor {
    readonly body: ammo.btRigidBody;
    radius: number;
    tolerance: number;
    get_position(): [number, number, number];
    set_position(x: number, y: number, z: number): void;
    get_AABB(): [[number, number, number], [number, number, number]];
    calc_AABB(): void;
}

export function create_sphere_rigid_bodies(n_bodies: number): Readonly<SphereRigidBodies> {
    const radiuses = new Float32Array(n_bodies);
    const positions = new Float32Array(n_bodies * 3);
    const AABBs = new Float32Array(n_bodies * 3 * 2);
    const tolerances = new Float32Array(n_bodies);
    return { bodies: [], radiuses, positions, AABBs, tolerances, length: n_bodies };
}

export function sphere_rigid_body_accessor(target: Readonly<SphereRigidBodies>, index: number): SphereRigidBodyAccessor {
    return {
        get body() { return target.bodies[index]; },
        get radius() { return target.radiuses[index]; },
        set radius(rad: number) { target.radiuses[index] = rad; },
        get tolerance() { return target.tolerances[index]; },
        set tolerance(tol: number) { target.tolerances[index] = tol; },
        get_position() {
            return get_vec3_array_gen(target.positions, index);
        },
        set_position(x: number, y: number, z: number) {
            set_vec3_gen(target.positions, index, x, y, z);
        },
        get_AABB() {
            return get_AABB(target.AABBs, index);
        },
        calc_AABB() {
            const AABBs = target.AABBs;
            const positions = target.positions;
            const radius = target.radiuses[index] + target.tolerances[index];
            const min_index = index * 3 * 2;
            const max_index = index * 3 * 2 + 3;
            const X = index * 3, Y = X + 1, Z = X + 2;
            AABBs[min_index] = positions[X] - radius;
            AABBs[min_index + 1] = positions[Y] - radius;
            AABBs[min_index + 2] = positions[Z] - radius;
            AABBs[max_index] = positions[X] + radius;
            AABBs[max_index + 1] = positions[Y] + radius;
            AABBs[max_index + 2] = positions[Z] + radius;
        }
    };
}

// Represents a box primitive around the origin
export interface BoxRigidBodies {
    bodies: ammo.btRigidBody[];
    half_widths: Float32Array;  // half length of side vector along X axis
    half_heights: Float32Array;  // half length of side vector along Y axis
    half_depths: Float32Array;  // half length of side vector along Z axis
    positions: Float32Array;  // origin position
    basis_matrices: Float32Array;
    inv_basis_mats: Float32Array;
    AABBs: Float32Array;  // [min,max], [min,max], ...
    tolerances: Float32Array;
    length: number;
}

export interface BoxRigidBodyAccessor {
    readonly body: ammo.btRigidBody;
    half_width: number;
    half_height: number;
    half_depth: number;
    tolerance: number;
    get_position(): [number, number, number];
    set_position(x: number, y: number, z: number): void;
    get_basis_vector(basis_index: number): [number, number, number];
    set_basis_vector(basis_index: number, x: number, y: number, z: number): void;
    get_inv_basis_vector(basis_index: number): [number, number, number];
    calc_inv_basis_matrix(): void;
    get_AABB(): [[number, number, number], [number, number, number]];
    calc_AABB(): void;
}

export function create_box_rigid_bodies(n_bodies: number): Readonly<BoxRigidBodies> {
    const half_widths = new Float32Array(n_bodies);
    const half_heights = new Float32Array(n_bodies);
    const half_depths = new Float32Array(n_bodies);
    const positions = new Float32Array(n_bodies * 3);
    const basis_matrices = new Float32Array(n_bodies * 9);
    const inv_basis_mats = new Float32Array(n_bodies * 9);
    const AABBs = new Float32Array(n_bodies * 3 * 2);
    const tolerances = new Float32Array(n_bodies);
    return { bodies: [], half_widths, half_heights, half_depths, positions, basis_matrices, inv_basis_mats, AABBs, tolerances, length: n_bodies };
}

export function box_rigid_body_accessor(target: Readonly<BoxRigidBodies>, index: number): BoxRigidBodyAccessor {
    return {
        get body() { return target.bodies[index]; },
        get half_width() { return target.half_widths[index]; },
        set half_width(width: number) { target.half_widths[index] = width; },
        get half_height() { return target.half_heights[index]; },
        set half_height(height: number) { target.half_heights[index] = height; },
        get half_depth() { return target.half_depths[index]; },
        set half_depth(depth: number) { target.half_depths[index] = depth; },
        get tolerance() { return target.tolerances[index]; },
        set tolerance(tol: number) { target.tolerances[index] = tol; },
        get_position() {
            return get_vec3_array_gen(target.positions, index);
        },
        set_position(x: number, y: number, z: number) {
            set_vec3_gen(target.positions, index, x, y, z);
        },
        get_basis_vector(basis_index: number): [number, number, number] {
            const basis_matrices = target.basis_matrices;
            const idx = index * 9;
            return [
                basis_matrices[idx + basis_index],
                basis_matrices[idx + basis_index + 3],
                basis_matrices[idx + basis_index + 6]
            ];
        },
        set_basis_vector(basis_index: number, x: number, y: number, z: number) {
            const basis_matrices = target.basis_matrices;
            const idx = index * 9;
            basis_matrices[idx + basis_index] = x;
            basis_matrices[idx + basis_index + 3] = y;
            basis_matrices[idx + basis_index + 6] = z;
        },
        get_inv_basis_vector(basis_index: number): [number, number, number] {
            const inv_basis_matrices = target.inv_basis_mats;
            const idx = index * 9;
            return [
                inv_basis_matrices[idx + basis_index],
                inv_basis_matrices[idx + basis_index + 3],
                inv_basis_matrices[idx + basis_index + 6]
            ];
        },
        calc_inv_basis_matrix: (() => {
            const mat_a = new three.Matrix3();
            const mat_b = new three.Matrix3();
            const byte_offset = index * 9 * 4;
            mat_a.elements = new Float32Array(target.inv_basis_mats.buffer, byte_offset, 9);
            mat_b.elements = new Float32Array(target.basis_matrices.buffer, byte_offset, 9);
            return () => {
                mat_a.getInverse(mat_b);
            };
        })(),
        get_AABB() {
            return get_AABB(target.AABBs, index);
        },
        calc_AABB: (() => {
            const min_vec = new Float32Array(3);
            const max_vec = new Float32Array(3);
            const AABBs = target.AABBs;
            const positions = target.positions;
            const half_widths = target.half_widths;
            const half_heights = target.half_heights;
            const half_depths = target.half_depths;
            const tolerances = target.tolerances;
            const basis_matrices = target.basis_matrices;
            const min_index = index * 3 * 2;
            const max_index = index * 3 * 2 + 3;
            const idx = index * 9;
            return () => {
                const tolerance = tolerances[index];
                min_vec[0] = -half_widths[index] - tolerance;
                min_vec[1] = -half_heights[index] - tolerance;
                min_vec[2] = -half_depths[index] - tolerance;
                max_vec[0] = half_widths[index] + tolerance;
                max_vec[1] = half_heights[index] + tolerance;
                max_vec[2] = half_depths[index] + tolerance;
                // http://dev.theomader.com/transform-bounding-boxes/
                for (let j = 0; j < 3; ++j) {
                    AABBs[min_index + j] = positions[index * 3 + j];
                    AABBs[max_index + j] = positions[index * 3 + j];
                    for (let i = 0; i < 3; ++i) {
                        const a = basis_matrices[idx + i * 3 + j] * min_vec[i];
                        const b = basis_matrices[idx + i * 3 + j] * max_vec[i];
                        if (a < b) {
                            AABBs[min_index + j] += a;
                            AABBs[max_index + j] += b;
                        } else {
                            AABBs[min_index + j] += b;
                            AABBs[max_index + j] += a;
                        }
                    }
                }
            };
        })()
    };
}

// Represents a capsule around the Y axis
export interface CapsuleRigidBodies {
    bodies: ammo.btRigidBody[];
    radiuses: Float32Array;
    lengths: Float32Array;  // length between the two spheres
    half_lengths: Float32Array;
    positions: Float32Array;  // center of mass position
    directions: Float32Array;  // direction of the Y axis, normalized
    AABBs: Float32Array;  // [min,max], [min,max], ...
    tolerances: Float32Array;
    length: number;
}

export interface CapsuleRigidBodyAccessor {
    readonly body: ammo.btRigidBody;
    radius: number;
    length: number;
    readonly half_length: number;
    tolerance: number;
    get_position(): [number, number, number];
    set_position(x: number, y: number, z: number): void;
    get_direction(): [number, number, number];
    set_direction(x: number, y: number, z: number): void;
    get_AABB(): [[number, number, number], [number, number, number]];
    calc_AABB(): void;
}

export function create_capsule_rigid_bodies(n_bodies: number): Readonly<CapsuleRigidBodies> {
    const radiuses = new Float32Array(n_bodies);
    const lengths = new Float32Array(n_bodies);
    const half_lengths = new Float32Array(n_bodies);
    const positions = new Float32Array(n_bodies * 3);
    const directions = new Float32Array(n_bodies * 3);
    const AABBs = new Float32Array(n_bodies * 3 * 2);
    const tolerances = new Float32Array(n_bodies);
    return { bodies: [], radiuses, lengths, half_lengths, positions, directions, AABBs, tolerances, length: n_bodies };
}

export function capsule_rigid_body_accessor(target: Readonly<CapsuleRigidBodies>, index: number): CapsuleRigidBodyAccessor {
    return {
        get body() { return target.bodies[index]; },
        get radius() { return target.radiuses[index]; },
        set radius(radius: number) { target.radiuses[index] = radius; },
        get length() { return target.lengths[index]; },
        set length(length: number) {
            target.lengths[index] = length;
            target.half_lengths[index] = length * 0.5;
        },
        get half_length() { return target.half_lengths[index]; },
        get tolerance() { return target.tolerances[index]; },
        set tolerance(tol: number) { target.tolerances[index] = tol; },
        get_position() {
            return get_vec3_array_gen(target.positions, index);
        },
        set_position(x: number, y: number, z: number) {
            set_vec3_gen(target.positions, index, x, y, z);
        },
        get_direction() {
            return get_vec3_array_gen(target.directions, index);
        },
        set_direction(x: number, y: number, z: number) {
            set_vec3_gen(target.directions, index, x, y, z);
        },
        get_AABB() {
            return get_AABB(target.AABBs, index);
        },
        calc_AABB() {
            const positions = target.positions;
            const directions = target.directions;
            const half_length = target.half_lengths[index];
            const radius = target.radiuses[index] + target.tolerances[index];
            const X = index * 3, Y = X + 1, Z = X + 2;
            const p0_x = positions[X] + directions[X] * half_length;
            const p0_y = positions[Y] + directions[Y] * half_length;
            const p0_z = positions[Z] + directions[Z] * half_length;
            const p1_x = positions[X] - directions[X] * half_length;
            const p1_y = positions[Y] - directions[Y] * half_length;
            const p1_z = positions[Z] - directions[Z] * half_length;
            const s0_min_x = p0_x - radius;
            const s0_min_y = p0_y - radius;
            const s0_min_z = p0_z - radius;
            const s0_max_x = p0_x + radius;
            const s0_max_y = p0_y + radius;
            const s0_max_z = p0_z + radius;
            const s1_min_x = p1_x - radius;
            const s1_min_y = p1_y - radius;
            const s1_min_z = p1_z - radius;
            const s1_max_x = p1_x + radius;
            const s1_max_y = p1_y + radius;
            const s1_max_z = p1_z + radius;
            const AABBs = target.AABBs;
            const min_index = index * 3 * 2;
            const max_index = index * 3 * 2 + 3;
            AABBs[min_index] = min(s0_min_x, s1_min_x);
            AABBs[min_index + 1] = min(s0_min_y, s1_min_y);
            AABBs[min_index + 2] = min(s0_min_z, s1_min_z);
            AABBs[max_index] = max(s0_max_x, s1_max_x);
            AABBs[max_index + 1] = max(s0_max_y, s1_max_y);
            AABBs[max_index + 2] = max(s0_max_z, s1_max_z);
        }
    };
}

// Represents a infinite plane
export interface PlaneRigidBodies {
    constants: Float32Array;
    normals: Float32Array;
    tolerances: Float32Array;
    length: number;
}

export interface PlaneRigidBodyAccessor {
    constant: number;
    tolerance: number;
    get_normal(): [number, number, number];
    set_normal(x: number, y: number, z: number): void;
}

export function create_plane_rigid_bodies(n_bodies: number): Readonly<PlaneRigidBodies> {
    const constants = new Float32Array(n_bodies);
    const normals = new Float32Array(n_bodies * 3);
    const tolerances = new Float32Array(n_bodies);
    return { constants, normals, tolerances, length: n_bodies };
}

export function plane_rigid_body_accessor(target: Readonly<PlaneRigidBodies>, index: number): PlaneRigidBodyAccessor {
    return {
        get constant() { return target.constants[index]; },
        set constant(constant: number) { target.constants[index] = constant; },
        get tolerance() { return target.tolerances[index]; },
        set tolerance(tol: number) { target.tolerances[index] = tol; },
        get_normal() {
            return get_vec3_array_gen(target.normals, index);
        },
        set_normal(x: number, y: number, z: number) {
            set_vec3_gen(target.normals, index, x, y, z);
        }
    };
}

function get_AABB(AABBs: Float32Array, index: number): [[number, number, number], [number, number, number]] {
    return [
        get_vec3_array_gen(AABBs, index * 2),
        get_vec3_array_gen(AABBs, (index * 2) + 1)
    ];
}
