import * as three from 'three';
import * as Stats from 'stats.js';
import { ParticleData, set_mass } from './simulation/ParticleData';
import { ParticleMesh, build_particle_mesh, update_face_normals, update_vertex_normals } from './simulation/ParticleMesh';
import { Constraint, create_distance_constraint } from './simulation/Constraints';
import { set_vec3_gen, copy_vec3_gen, add_to_vec3_gen,
    mul_scalar_vec3_gen, sub_vec3_gen, mul_scalar_to_vec3 } from './simulation/math';
import CanvasRecorder from './utils/CanvasRecorder';

const ClothWidth = 5;
const ClothHeight = 5;
const ClothParticleCols = 50;
const ClothParticleRows = 50;
const ClothCompressionStiffness = 1.0;
const ClothStretchStiffness = 1.0;

const TimeStepSize = 0.004;
const StepIteration = 4;
const Gravity = -9.81;
const MaxProjectionIteration = 5;

const WireframeRendering = false;

interface ClothModel {
    particles: ParticleData;
    mesh: ParticleMesh;
    constraints: Constraint[];
}

document.addEventListener('DOMContentLoaded', () => {
    const width = window.innerWidth;
    const height = window.innerHeight;

    const stats = new Stats();
    document.body.appendChild(stats.dom);

    const renderer = new three.WebGLRenderer();
    renderer.setClearColor(new three.Color(0xfcfcfc))
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(width, height);
    document.body.appendChild(renderer.domElement);

    const recorder = CanvasRecorder.createFromUrlParams(renderer.domElement, document.location.href);
    
    const camera = new three.PerspectiveCamera(45, width / height, 1, 1000);
    camera.position.set(1, 0.5, 1).setLength(15);
    camera.lookAt(new three.Vector3(ClothWidth * 0.5, -ClothHeight * 0.5, 0));

    const model = build_model(ClothParticleCols, ClothParticleRows, ClothWidth, ClothHeight);
    const positions = model.particles.position;
    const indices = model.mesh.indices;
    const face_normals = model.mesh.face_normals;
    const vertex_normals = model.mesh.vertex_normals;

    const scene = build_scene(positions, vertex_normals, indices);
    const geometry = (scene.children[0] as three.Mesh).geometry as three.BufferGeometry;
    const posAttr = geometry.getAttribute('position') as three.BufferAttribute;
    const normalAttr = geometry.getAttribute('normal') as three.BufferAttribute;

    const render = () => {
        stats.begin();

        step(model);

        update_face_normals(positions, indices, face_normals);
        update_vertex_normals(face_normals, indices, vertex_normals);
        posAttr.needsUpdate = true;
        normalAttr.needsUpdate = true;

        renderer.render(scene, camera);

        stats.end();

        requestAnimationFrame(render);
    };

    window.addEventListener('resize', () => {
        handle_resize(camera, renderer);
    });

    recorder && recorder.start();
    render();
});

function handle_resize(camera: three.PerspectiveCamera, renderer: three.WebGLRenderer) {
    const width = window.innerWidth;
    const height = window.innerHeight;
    renderer.setSize(width, height);
    camera.aspect = width / height;
}

function build_scene(vertices: Float32Array, normals: Float32Array, indices: Uint32Array): three.Scene {
    const geometry = new three.BufferGeometry();
    geometry.addAttribute('position', new three.BufferAttribute(vertices, 3));
    geometry.addAttribute('normal', new three.BufferAttribute(normals, 3));
    geometry.setIndex(new three.BufferAttribute(indices, 1));
    const material = new three.MeshStandardMaterial({
        color: 0x820202, emissive: new three.Color(0x7c0808),
        roughness: 0.88, metalness: 0.9,
        side: three.DoubleSide,
        wireframe: WireframeRendering
    });
    const cloth = new three.Mesh(geometry, material);
    cloth.frustumCulled = false;
    cloth.matrixAutoUpdate = false;

    const lights = [
        new three.PointLight(0xffffff, 0.9, 0),
        new three.PointLight(0xffffff, 0.9, 0),
        new three.PointLight(0xffffff, 0.9, 0),
    ];
    lights[0].position.set(0, 20, 0);
    lights[1].position.set(10, 20, 10);
    lights[2].position.set(-10, -20, -10);

    const scene = new three.Scene();
    scene.add(cloth);
    scene.add(lights[0]);
    scene.add(lights[1]);
    scene.add(lights[2]);

    return scene;
}

function build_model(n_cols: number, n_rows: number, width: number, height: number): ClothModel {
    const particles = new ParticleData(n_cols * n_rows);
    const dx = width / (n_cols - 1);
    const dy = height / (n_rows - 1);

    for (let i = 0; i < n_rows; ++i) {
        for (let j = 0; j < n_cols; ++j) {
            const index = i * n_cols + j;
            set_vec3_gen(particles.position, index, dx * j, 0.0, dy * i);
            copy_vec3_gen(particles.old_position, index, particles.position, index);
            set_mass(particles, index, 1.0);
        }
    }

    // pined cloth top corner
    set_mass(particles, 0, 0.0);
    set_mass(particles, n_cols - 1, 0.0);

    const indexArray = [...Array((n_cols - 1) * (n_rows - 1)).keys()]  // 0..(num of triangle pairs - 1)
    .map(index => [
        index + Math.floor(index / (n_cols - 1)), n_cols + index + Math.floor(index / (n_cols - 1)), index + Math.floor(index / (n_cols - 1)) + 1,  // triangle0 indices
        index + Math.floor(index / (n_cols - 1)) + 1, n_cols + index + Math.floor(index / (n_cols - 1)), n_cols + index + Math.floor(index / (n_cols - 1)) + 1  // triangle1 indices
    ])
    .reduce((a, b) => a.concat(b));  // flatten

    const indices = new Uint32Array(indexArray);

    const mesh = build_particle_mesh(particles, indices);

    const constraints = mesh.edges.map(e => create_distance_constraint(particles, e.vertices[0], e.vertices[1]));

    return { particles, mesh, constraints };
}

function step(model: ClothModel) {
    const n_particle = model.particles.length;
    const position = model.particles.position;
    const old_position = model.particles.old_position;
    const velocity = model.particles.velocity;
    const mass = model.particles.mass;
    const inv_mass = model.particles.inv_mass;

    for (let i = 0; i < StepIteration; ++i) {
        internal_step(n_particle, position, old_position, velocity, mass, inv_mass, model.constraints);
    }
}

const d_vec = new Float32Array(3);
const stiffness = { compression: ClothCompressionStiffness, stretch: ClothStretchStiffness };
function internal_step(n_particle: number, position: Float32Array, old_position: Float32Array,
    velocity: Float32Array, mass: Float32Array, inv_mass: Float32Array, constraints: Constraint[]) {

    for (let i = 0, len = position.length; i < len; ++i) {
        old_position[i] = position[i];
    }

    // position predictions
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            velocity[i * 3 + 1] += Gravity * TimeStepSize;
            mul_scalar_vec3_gen(velocity, i, TimeStepSize, d_vec, 0);
            add_to_vec3_gen(position, i, d_vec, 0);
        }
    }

    // project constraints
    for (let i = 0; i < MaxProjectionIteration; ++i) {
        for (let j = 0, len = constraints.length; j < len; ++j) {
            constraints[j].solver(constraints[j], position, inv_mass, stiffness);
        }
    }

    // update velocities
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            sub_vec3_gen(position, i, old_position, i, d_vec, 0);
            mul_scalar_to_vec3(d_vec, 1.0 / TimeStepSize);
            copy_vec3_gen(velocity, i, d_vec, 0);
        }
    }
}
