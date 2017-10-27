import * as three from 'three';
import * as ammo from 'ammo.js';
import { get_vec3_array_gen, set_vec3_gen } from './math';

export interface SphereRigidBodies {
    bodies: ammo.btRigidBody[];
    radiuses: Float32Array;
    positions: Float32Array;
    length: number;
}

export interface SphereRigidBodyAccessor {
    readonly body: ammo.btRigidBody;
    radius: number;
    get_position(): [number, number, number];
    set_position(x: number, y: number, z: number): void;
}

export function create_sphere_rigid_bodies(n_bodies: number): Readonly<SphereRigidBodies> {
    const radiuses = new Float32Array(n_bodies);
    const positions = new Float32Array(n_bodies * 3);
    return { bodies: [], radiuses, positions, length: n_bodies };
}

export function sphere_rigid_body_accessor(target: Readonly<SphereRigidBodies>, index: number): SphereRigidBodyAccessor {
    return {
        get body() { return target.bodies[index]; },
        get radius() { return target.radiuses[index]; },
        set radius(rad: number) {
            target.radiuses[index] = rad;
        },
        get_position() {
            return get_vec3_array_gen(target.positions, index);
        },
        set_position(x: number, y: number, z: number) {
            set_vec3_gen(target.positions, index, x, y, z);
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
    length: number;
}

export interface BoxRigidBodyAccessor {
    readonly body: ammo.btRigidBody;
    half_width: number;
    half_height: number;
    half_depth: number;
    get_position(): [number, number, number];
    set_position(x: number, y: number, z: number): void;
    get_basis_vector(basis_index: number): [number, number, number];
    set_basis_vector(basis_index: number, x: number, y: number, z: number): void;
    get_inv_basis_vector(basis_index: number): [number, number, number];
    calc_inv_basis_matrix(): void;
}

export function create_box_rigid_bodies(n_bodies: number): Readonly<BoxRigidBodies> {
    const half_widths = new Float32Array(n_bodies);
    const half_heights = new Float32Array(n_bodies);
    const half_depths = new Float32Array(n_bodies);
    const positions = new Float32Array(n_bodies * 3);
    const basis_matrices = new Float32Array(n_bodies * 9);
    const inv_basis_mats = new Float32Array(n_bodies * 9);
    return { bodies: [], half_widths, half_heights, half_depths, positions, basis_matrices, inv_basis_mats, length: n_bodies };
}

export function box_rigid_body_accessor(target: Readonly<BoxRigidBodies>, index: number): BoxRigidBodyAccessor {
    const mat_a = new three.Matrix3();
    const mat_b = new three.Matrix3();
    return {
        get body() { return target.bodies[index]; },
        get half_width() { return target.half_widths[index]; },
        set half_width(width: number) { target.half_widths[index] = width; },
        get half_height() { return target.half_heights[index]; },
        set half_height(height: number) { target.half_heights[index] = height; },
        get half_depth() { return target.half_depths[index]; },
        set half_depth(depth: number) { target.half_depths[index] = depth; },
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
        calc_inv_basis_matrix() {
            const inv_basis_mats = target.inv_basis_mats;
            const basis_matrices = target.basis_matrices;
            const byte_offset = index * 9 * 4;
            mat_a.elements = new Float32Array(inv_basis_mats.buffer, byte_offset, 9);
            mat_b.elements = new Float32Array(basis_matrices.buffer, byte_offset, 9);
            mat_a.getInverse(mat_b);
        }
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
    length: number;
}

export interface CapsuleRigidBodyAccessor {
    readonly body: ammo.btRigidBody;
    radius: number;
    length: number;
    readonly half_length: number;
    get_position(): [number, number, number];
    set_position(x: number, y: number, z: number): void;
    get_direction(): [number, number, number];
    set_direction(x: number, y: number, z: number): void;
}

export function create_capsule_rigid_bodies(n_bodies: number): Readonly<CapsuleRigidBodies> {
    const radiuses = new Float32Array(n_bodies);
    const lengths = new Float32Array(n_bodies);
    const half_lengths = new Float32Array(n_bodies);
    const positions = new Float32Array(n_bodies * 3);
    const directions = new Float32Array(n_bodies * 3);
    return { bodies: [], radiuses, lengths, half_lengths, positions, directions, length: n_bodies };
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
        }
    };
}

// Represents a infinite plane
export interface PlaneRigidBodies {
    constants: Float32Array;
    normals: Float32Array;
    length: number;
}

export interface PlaneRigidBodyAccessor {
    constant: number;
    get_normal(): [number, number, number];
    set_normal(x: number, y: number, z: number): void;
}

export function create_plane_rigid_bodies(n_bodies: number): Readonly<PlaneRigidBodies> {
    const constants = new Float32Array(n_bodies);
    const normals = new Float32Array(n_bodies * 3);
    return { constants, normals, length: n_bodies };
}

export function plane_rigid_body_accessor(target: Readonly<PlaneRigidBodies>, index: number) {
    return {
        get constant() { return target.constants[index]; },
        set constant(constant: number) { target.constants[index] = constant; },
        get_normal() {
            return get_vec3_array_gen(target.normals, index);
        },
        set_normal(x: number, y: number, z: number) {
            set_vec3_gen(target.normals, index, x, y, z);
        }
    };
}
