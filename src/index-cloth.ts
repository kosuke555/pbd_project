import * as three from 'three';
import Stats from 'stats.js';
import { VertexArrayType, IndexArrayType } from './types';
import ClothModel from './simulation/ClothModel';
import { create_particle_data, init_position, set_mass } from './simulation/ParticleData';
import { build_particle_mesh, update_face_normals, update_vertex_normals } from './simulation/ParticleMesh';
import { create_distance_constraint } from './simulation/constraints';
import { step } from './simulation/time_integrator';
import CanvasRecorder from './utils/CanvasRecorder';

const ClothWidth = 5;
const ClothHeight = 5;
const ClothParticleCols = 50;
const ClothParticleRows = 50;

const TimeStep = 1/60;
const StepIteration = 4;
const MaxProjectionIteration = 5;

const Gravity = -9.81;
const ClothCompressionStiffness = 1.0;
const ClothStretchStiffness = 1.0;

const WireframeRendering = false;

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
    const pos_attr = geometry.getAttribute('position') as three.BufferAttribute;
    const normal_attr = geometry.getAttribute('normal') as three.BufferAttribute;

    const render = () => {
        stats.begin();

        step(model, {
            step_iter: StepIteration,
            max_proj_iter: MaxProjectionIteration,
            time_step: TimeStep,
            gravity: Gravity,
            stiffnesses: {
                compression: ClothCompressionStiffness,
                stretch: ClothStretchStiffness
            }
        });

        update_face_normals(positions, indices, face_normals);
        update_vertex_normals(face_normals, indices, vertex_normals);
        pos_attr.needsUpdate = true;
        normal_attr.needsUpdate = true;

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

function build_scene(vertices: VertexArrayType, normals: Float32Array, indices: IndexArrayType): three.Scene {
    const geometry = new three.BufferGeometry();
    const pos_attr = new three.BufferAttribute(vertices, 3);
    pos_attr.setDynamic(true);
    geometry.addAttribute('position', pos_attr);
    const normal_attr = new three.BufferAttribute(normals, 3);
    normal_attr.setDynamic(true);
    geometry.addAttribute('normal', normal_attr);
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

function build_model(n_cols: number, n_rows: number, width: number, height: number): Readonly<ClothModel> {
    const particles = create_particle_data(n_cols * n_rows);
    const dx = width / (n_cols - 1);
    const dy = height / (n_rows - 1);

    for (let i = 0; i < n_rows; ++i) {
        for (let j = 0; j < n_cols; ++j) {
            const index = i * n_cols + j;
            init_position(particles, index, dx * j, 0.0, dy * i);
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

    const constraints = mesh.edges.map(e => create_distance_constraint(particles, e.vertex_pair[0], e.vertex_pair[1]));

    return { particles, mesh, constraints };
}
