import { ParticlePositionType } from '../types';
import { get_vec3_array_gen, set_vec3_gen } from './math';

export interface ParticleData {
    mass: Float32Array;
    inv_mass: Float32Array;
    velocity: Float32Array;
    position: ParticlePositionType;
    old_position: ParticlePositionType;
    extra_vec: Float32Array;  // for pre-stabilization, velocity damping and friction
    extra_uint8: Uint8Array;  // for pre-stabilization
    extra_float: Float32Array;  // for friction
    readonly length: number;
}

export interface ParticleAccessor {
    mass: number;
    readonly inv_mass: number;
    get_velocity(): [number, number, number];
    set_velocity(x: number, y: number, z: number): void;
    get_position(): [number, number, number];
    set_position(x: number, y: number, z: number): void;
    get_old_position(): [number, number, number];
    set_old_position(x: number, y: number, z: number): void;
    init_position(x: number, y: number, z: number): void;
    init_position(vectors: ParticlePositionType, v_index: number): void;
}

export function create_particle_data(n_particle: number): Readonly<ParticleData> {
    const mass = new Float32Array(n_particle);
    const inv_mass = new Float32Array(n_particle);
    const velocity = new Float32Array(n_particle * 3);
    const position = new Float32Array(n_particle * 3);
    const old_position = new Float32Array(n_particle * 3);
    const extra_vec = new Float32Array(n_particle * 3);
    const extra_uint8 = new Uint8Array(n_particle);
    const extra_float = new Float32Array(n_particle);
    const length = n_particle;
    return { mass, inv_mass, velocity, position, old_position, extra_vec, extra_uint8, extra_float, length };
}

export function reset_particle_data(particles: ParticleData, initial_positions: ParticlePositionType) {
    particles.position.set(initial_positions);
    particles.old_position.set(initial_positions);
    particles.velocity.fill(0);
    particles.extra_vec.fill(0);
    particles.extra_uint8.fill(0);
    particles.extra_float.fill(0);
}

export function particle_accessor(target: Readonly<ParticleData>, index: number): ParticleAccessor {
    return {
        get mass() { return target.mass[index]; },
        set mass(mass: number) { set_mass(target, index, mass); },
        get inv_mass() { return target.inv_mass[index]; },
        get_velocity() {
            return get_vec3_array_gen(target.velocity, index);
        },
        set_velocity(x: number, y: number, z: number) {
            set_vec3_gen(target.velocity, index, x, y, z);
        },
        get_position() {
            return get_vec3_array_gen(target.position, index);
        },
        set_position(x: number, y: number, z: number) {
            set_vec3_gen(target.position, index, x, y, z);
        },
        get_old_position() {
            return get_vec3_array_gen(target.old_position, index);
        },
        set_old_position(x: number, y: number, z: number) {
            set_vec3_gen(target.old_position, index, x, y, z);
        },
        init_position(x_or_vec: number|ParticlePositionType, y_or_v_idx: number, z?: number) {
            init_position(target, index, x_or_vec, y_or_v_idx, z);
        }
    };
}

/**
 * initialize particle position.
 * this method initialize the old_position simultaneously.
 * @param particle_data target particle data
 * @param p_index particle index
 * @param vectors copy a position from one of this vectors
 * @param v_index vector index
 */
export function init_position(particle_data: ParticleData, p_index: number, vectors: ParticlePositionType, v_index: number): void;
/**
 * initialize particle position.
 * this method initialize the old_position simultaneously.
 * @param particle_data target particle data
 * @param p_index particle index
 * @param x coordinate x
 * @param y coordinate y
 * @param z coordinate z
 */
export function init_position(particle_data: ParticleData, p_index: number, x: number, y: number, z: number): void;
export function init_position(particle_data: ParticleData, p_index: number, x_or_vec: number|ParticlePositionType, y_or_v_idx: number, _z?: number): void;
export function init_position(particle_data: ParticleData, p_index: number, x_or_vec: number|ParticlePositionType, y_or_v_idx: number, _z?: number) {
    const pos = particle_data.position;
    const old_pos = particle_data.old_position;
    if (typeof x_or_vec === 'number') {
        const x = x_or_vec, y = y_or_v_idx, z = _z!;
        pos[p_index * 3] = x;
        pos[p_index * 3 + 1] = y;
        pos[p_index * 3 + 2] = z;
        old_pos[p_index * 3] = x;
        old_pos[p_index * 3 + 1] = y;
        old_pos[p_index * 3 + 2] = z;
    } else {
        const vectors = x_or_vec;
        const v_index = y_or_v_idx;
        pos[p_index * 3] = vectors[v_index * 3];
        pos[p_index * 3 + 1] = vectors[v_index * 3 + 1];
        pos[p_index * 3 + 2] = vectors[v_index * 3 + 2];
        old_pos[p_index * 3] = vectors[v_index * 3];
        old_pos[p_index * 3 + 1] = vectors[v_index * 3 + 1];
        old_pos[p_index * 3 + 2] = vectors[v_index * 3 + 2];
    }
}

/**
 * set particle mass.
 * mass == 0.0 is static particle.
 * @param particle_data target particle data
 * @param index particle index
 * @param mass particle mass
 */
export function set_mass(particle_data: ParticleData, index: number, mass: number) {
    particle_data.mass[index] = mass;
    particle_data.inv_mass[index] = mass !== 0.0 ? 1.0 / mass : 0.0;
}
