export class ParticleData {
    mass: Float32Array;
    inv_mass: Float32Array;
    position: Float32Array;
    old_position: Float32Array;
    velocity: Float32Array;
    readonly length: number;

    constructor(n_particle: number) {
        this.mass = new Float32Array(n_particle);
        this.inv_mass = new Float32Array(n_particle);
        this.position = new Float32Array(n_particle * 3);
        this.old_position = new Float32Array(n_particle * 3);
        this.velocity = new Float32Array(n_particle * 3);
        this.length = n_particle;
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
