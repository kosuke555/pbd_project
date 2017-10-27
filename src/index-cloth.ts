import * as three from 'three';
import Stats from 'stats.js';
import { VertexArrayType, IndexArrayType } from './types';
import ClothModel from './simulation/ClothModel';
import { create_particle_data, init_position, set_mass } from './simulation/ParticleData';
import { build_particle_mesh, update_face_normals, update_vertex_normals } from './simulation/ParticleMesh';
import { create_distance_constraint } from './simulation/constraints';
import { create_sphere_rigid_bodies, sphere_rigid_body_accessor, create_box_rigid_bodies,
    create_plane_rigid_bodies, box_rigid_body_accessor, create_capsule_rigid_bodies,
    capsule_rigid_body_accessor, plane_rigid_body_accessor } from './simulation/rigid_bodies';
import { CollisionObjects } from './simulation/collision_detection';
import { step } from './simulation/time_integrator';
import CanvasRecorder from './utils/CanvasRecorder';
import { particle_helper, rigid_body_helper } from './utils/debug';

const ClothWidth = 5;
const ClothHeight = 5;
const ClothParticleCols = 50;
const ClothParticleRows = 50;

const RigidBodyType = 'capsule';

const TimeStep = 1/60;
const StepIteration = 4;
const MaxProjectionIteration = 5;
const PreStabilizationIteration = 1;

const Gravity = -9.81;
const VelocityDampingFactor = 0.00125;
const ClothCompressionStiffness = 1.0;
const ClothStretchStiffness = 1.0;
const ClothMass = 1 / (ClothParticleCols * ClothParticleRows);

const SphereCollisionTolerance = 0.005;
const BoxCollisionTolerance = 0.055;
const CapsuleCollisionTolerance = 0.01;
const PlaneCollisionTolerance = 0;

const WireframeRendering = false;
const RigidBodyWireframeRendering = false;

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
    
    const camera = new three.PerspectiveCamera(40, width / height, 1, 1000);
    camera.position.set(1, 0.5, 1).setLength(15);
    camera.lookAt(new three.Vector3(ClothWidth * 0.5, -ClothHeight * 0.5, 0));

    const model = build_model(ClothParticleCols, ClothParticleRows, ClothWidth, ClothHeight);
    const particles = model.particles;
    const positions = particles.position;
    const indices = model.mesh.indices;
    const face_normals = model.mesh.face_normals;
    const vertex_normals = model.mesh.vertex_normals;

    const scene = build_scene(positions, vertex_normals, indices);
    const geometry = (scene.children[0] as three.Mesh).geometry as three.BufferGeometry;
    const pos_attr = geometry.getAttribute('position') as three.BufferAttribute;
    const normal_attr = geometry.getAttribute('normal') as three.BufferAttribute;

    const rigid_bodies = create_rigid_bodies(RigidBodyType);
    const rigid_body_meshes = rigid_body_helper(rigid_bodies, RigidBodyWireframeRendering);
    rigid_body_meshes.forEach(mesh => scene.add(mesh));

    const simulation_config = {
        step_iter: StepIteration,
        max_proj_iter: MaxProjectionIteration,
        pre_stabilize_iter: PreStabilizationIteration,
        time_step: TimeStep,
        gravity: Gravity,
        velocity_damp_factor: VelocityDampingFactor,
        stiffnesses: {
            compression: ClothCompressionStiffness,
            stretch: ClothStretchStiffness
        },
        collision_tolerances: {
            sphere: SphereCollisionTolerance,
            box: BoxCollisionTolerance,
            capsule: CapsuleCollisionTolerance,
            plane: PlaneCollisionTolerance
        }
    };

    const debug_points = particle_helper(particles);
    const debug_points_geo = debug_points.geometry as three.BufferGeometry;
    const debug_points_attr = debug_points_geo.getAttribute('position') as three.BufferAttribute;
    // scene.add(debug_points);

    const render = () => {
        stats.begin();

        step(model, rigid_bodies, simulation_config);

        update_face_normals(positions, indices, face_normals);
        update_vertex_normals(face_normals, indices, vertex_normals);
        pos_attr.needsUpdate = true;
        normal_attr.needsUpdate = true;

        debug_points_attr.needsUpdate = true;

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
            set_mass(particles, index, ClothMass);
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

function create_rigid_bodies(type: 'sphere' | 'box' | 'capsule' | 'plane'): CollisionObjects {
    switch (type) {
        case 'sphere':
            const spheres = create_sphere_rigid_bodies(1);
            const sphere = sphere_rigid_body_accessor(spheres, 0);
            sphere.set_position(ClothWidth * 0.5, -ClothHeight * 0.5, 0);
            sphere.radius = ClothWidth * 0.25;
            return { spheres };

        case 'box':
            const boxes = create_box_rigid_bodies(1);
            const box = box_rigid_body_accessor(boxes, 0);
            box.half_width = ClothWidth * 0.25;
            box.half_height = ClothWidth * 0.25;
            box.half_depth = ClothWidth * 0.25;
            box.set_position(ClothWidth * 0.5, -ClothHeight * 0.5, 0);
            const rot_mat = new three.Matrix4().makeRotationY(Math.PI / 4).elements;
            box.set_basis_vector(0, rot_mat[0], rot_mat[4], rot_mat[8]);
            box.set_basis_vector(1, rot_mat[1], rot_mat[5], rot_mat[9]);
            box.set_basis_vector(2, rot_mat[2], rot_mat[6], rot_mat[10]);
            box.calc_inv_basis_matrix();
            return { boxes };

        case 'capsule':
            const capsules = create_capsule_rigid_bodies(1);
            const capsule = capsule_rigid_body_accessor(capsules, 0);
            capsule.radius = ClothWidth * 0.125;
            capsule.length = ClothWidth * 0.25;
            capsule.set_position(ClothWidth * 0.5, -ClothHeight * 0.5, 0);
            capsule.set_direction(0, 0, 1);
            return { capsules };

        case 'plane':
            const planes = create_plane_rigid_bodies(1);
            const plane = plane_rigid_body_accessor(planes, 0);
            plane.constant = ClothHeight * 0.5;
            plane.set_normal(0, 1, 0);
            return { planes };
    }
}
