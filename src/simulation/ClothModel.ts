import { ParticleData } from './ParticleData';
import { ParticleMesh } from './ParticleMesh';
import { Constraint } from './constraints';

export default interface ClothModel {
    particles: ParticleData;
    mesh: ParticleMesh;
    constraints: Constraint[];
}
