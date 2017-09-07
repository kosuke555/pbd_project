import { ParticlePositionType } from '../types';

export interface ParticleData {
    mass: Float32Array;
    inv_mass: Float32Array;
    velocity: Float32Array;
    position: ParticlePositionType;
    old_position: ParticlePositionType;
    delta_position: ParticlePositionType;  // for pre-stabilization
    n_collision: Uint8Array;  // for pre-stabilization
    r_vec: Float32Array;  // for velocity damping
    readonly length: number;
}

export function create_particle_data(n_particle: number): Readonly<ParticleData> {
    const mass = new Float32Array(n_particle);
    const inv_mass = new Float32Array(n_particle);
    const velocity = new Float32Array(n_particle * 3);
    const position = new Float32Array(n_particle * 3);
    const old_position = new Float32Array(n_particle * 3);
    const delta_position = new Float32Array(n_particle * 3);
    const n_collision = new Uint8Array(n_particle);
    const r_vec = new Float32Array(n_particle * 3);
    const length = n_particle;
    return { mass, inv_mass, velocity, position, old_position, delta_position, n_collision, r_vec, length };
}

export function reset_particle_data(particles: ParticleData, initial_positions: ParticlePositionType) {
    particles.position.set(initial_positions);
    particles.old_position.set(initial_positions);
    particles.delta_position.fill(0);
    particles.velocity.fill(0);
    particles.n_collision.fill(0);
    particles.r_vec.fill(0);
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
export function init_position(particle_data: ParticleData, p_index: number, x_or_vec: number|ParticlePositionType, y_or_v_idx: number, _z?: number) {
    const pos = particle_data.position;
    const old_pos = particle_data.old_position;
    const d_pos = particle_data.delta_position;
    if (typeof x_or_vec === 'number') {
        const x = x_or_vec, y = y_or_v_idx, z = _z!;
        pos[p_index * 3] = x;
        pos[p_index * 3 + 1] = y;
        pos[p_index * 3 + 2] = z;
        old_pos[p_index * 3] = x;
        old_pos[p_index * 3 + 1] = y;
        old_pos[p_index * 3 + 2] = z;
        d_pos[p_index * 3] = x;
        d_pos[p_index * 3 + 1] = y;
        d_pos[p_index * 3 + 2] = z;
    } else {
        const vectors = x_or_vec;
        const v_index = y_or_v_idx;
        pos[p_index * 3] = vectors[v_index * 3];
        pos[p_index * 3 + 1] = vectors[v_index * 3 + 1];
        pos[p_index * 3 + 2] = vectors[v_index * 3 + 2];
        old_pos[p_index * 3] = vectors[v_index * 3];
        old_pos[p_index * 3 + 1] = vectors[v_index * 3 + 1];
        old_pos[p_index * 3 + 2] = vectors[v_index * 3 + 2];
        d_pos[p_index * 3] = vectors[v_index * 3];
        d_pos[p_index * 3 + 1] = vectors[v_index * 3 + 1];
        d_pos[p_index * 3 + 2] = vectors[v_index * 3 + 2];
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
