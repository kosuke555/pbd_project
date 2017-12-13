import * as three from 'three';
import Stats from 'stats.js';
import * as ammo from 'ammo.js';
import { VertexArrayType, IndexArrayType, NormalArrayType, SkinIndexArrayType, WeightArrayType, ParticlePositionType } from './types';
import ClothModel from './simulation/ClothModel';
import { ParticleData, create_particle_data, reset_particle_data, init_position, set_mass } from './simulation/ParticleData';
import { build_particle_mesh, update_face_normals, update_vertex_normals } from './simulation/ParticleMesh';
import { create_distance_constraint } from './simulation/constraints';
import { SphereRigidBodies, BoxRigidBodies, CapsuleRigidBodies, PlaneRigidBodies,
    SphereRigidBodyAccessor, BoxRigidBodyAccessor, CapsuleRigidBodyAccessor,
    create_sphere_rigid_bodies, create_box_rigid_bodies, create_capsule_rigid_bodies, create_plane_rigid_bodies,
    sphere_rigid_body_accessor, box_rigid_body_accessor, capsule_rigid_body_accessor, plane_rigid_body_accessor } from './simulation/rigid_bodies';
import { CollisionDetector } from './simulation/collision_detection';
import { step, SimulationConfiguration } from './simulation/time_integrator';
import { to_enumerable } from './utils/transformers';
import { sub_typed_array, get_constructor } from './utils/typedarray_utils';
import CanvasRecorder from './utils/CanvasRecorder';
import { particle_helper, rigid_body_helper } from './utils/debug';

const TimeStep = 1/60;
const StepIteration = 4;
const MaxProjectionIteration = 5;
const PreStabilizationIteration = 1;
const PhysicsStabilizationTime = 2;

const Gravity = -9.81 * 14;
const VelocityDampingFactor = 0.00125;
const ClothCompressionStiffness = 1;
const ClothStretchStiffness = 1;
const ClothMass = 0.001;
const StaticFrictionCoefficient = 0.61;
const KineticFrictionCoefficient = 0.52;

const DefaultMMDModel = 'assets/sylvie/sylvie.pmx';
const DefaultMMDMotion = 'assets/vmd/motion.vmd';
// const DefaultMMDMotion = 'assets/vmd/walk.vmd';
// const DefaultMMDMotion = 'assets/vmd/agura.vmd';
// const DefaultMMDMotion = 'assets/vmd/turn.vmd';
// const DefaultMMDMotion = 'assets/vmd/dance.vmd';
// const DefaultMMDMotion = 'assets/vmd/dance2.vmd';
// const DefaultMMDMotion = 'assets/vmd/Skip.vmd';
// const DefaultMMDMotion = 'assets/vmd/Running.vmd';
// const DefaultMMDMotion = 'assets/vmd/AzatokawaiiTurn.vmd';
const TargetMaterialName = 'スカート';

const WireframeRendering = false;
const ShowParticlePoint = false;
const ShowRigidBody = false;
const ResetParticlesWhenLoopingAnimation = true;

interface AttachedParticles {
    particle_indices: IndexArrayType;
    bones_indices: Uint32Array;
    bones_weights: WeightArrayType;
    initial_positions: ParticlePositionType;
    length: number;
}

interface VertexParticlePairs {
    vertex_idx_list: IndexArrayType;
    particle_idx_list: IndexArrayType;
    length: number;
}

interface MappingInfo {
    attachments: AttachedParticles;
    pairs: VertexParticlePairs;
}

interface RigidBodySet {
    spheres: SphereRigidBodies;
    sphere_accessors: SphereRigidBodyAccessor[];
    boxes: BoxRigidBodies;
    box_accessors: BoxRigidBodyAccessor[];
    capsules: CapsuleRigidBodies;
    capsule_accessors: CapsuleRigidBodyAccessor[];
    planes: PlaneRigidBodies;
}

type LoadedMMDMesh = three.MMDMesh & three.AdditionalMMDAnimationData & three.AdditionalMMDPhysicsData;

document.addEventListener('DOMContentLoaded', async () => {
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
    camera.position.set(1, 1, 1).setLength(30);
    camera.lookAt(new three.Vector3(0, 10, 0));

    const scene = build_scene();

    const mmd_helper = new three.MMDHelper();
    const mmd_mesh = await load_mmd_mesh(DefaultMMDModel, DefaultMMDMotion, mmd_helper);
    if (WireframeRendering) {
        const material_index = get_material_index_by_name(mmd_mesh.material, TargetMaterialName);
        mmd_mesh.material[material_index].wireframe = true;
    }
    mmd_mesh.add(camera);
    scene.add(mmd_mesh);

    const rigid_bodies = setup_rigid_bodies(mmd_mesh.physics);

    const model = build_model(mmd_mesh);
    const particles = model.particles;

    const detector = new CollisionDetector({
        table_size: 997,
        grid_cell_size: 5,
        position_offset: 100
    });

    const simulation_config = {
        step_iter: StepIteration,
        max_proj_iter: MaxProjectionIteration,
        pre_stabilize_iter: PreStabilizationIteration,
        time_step: TimeStep,
        gravity: Gravity,
        velocity_damp_factor: VelocityDampingFactor,
        static_friction_coeff: StaticFrictionCoefficient,
        kinetic_friction_coeff: KineticFrictionCoefficient,
        stiffnesses: {
            compression: ClothCompressionStiffness,
            stretch: ClothStretchStiffness
        }
    };

    stabilize_physics(mmd_helper, mmd_mesh, model,
        rigid_bodies, detector, simulation_config, PhysicsStabilizationTime);

    if (ResetParticlesWhenLoopingAnimation) {
        const initial_positions = model.particles.position.slice();
        mmd_mesh.mixer.addEventListener('loop', _ => {
            reset_particle_data(particles, initial_positions);
        });
    }

    const rigid_body_meshes = rigid_body_helper(rigid_bodies);
    ShowRigidBody && rigid_body_meshes.forEach(mesh => scene.add(mesh));

    const debug_points = particle_helper(particles);
    ShowParticlePoint && scene.add(debug_points);

    window.addEventListener('resize', () => {
        handle_resize(camera, renderer);
    });

    create_loop_runner({ stats, renderer, scene, camera, mmd_helper, mmd_mesh,
        model, rigid_bodies, detector, simulation_config, debug_points, rigid_body_meshes })();

    recorder && recorder.start();
});

function handle_resize(camera: three.PerspectiveCamera, renderer: three.WebGLRenderer) {
    const width = window.innerWidth;
    const height = window.innerHeight;
    renderer.setSize(width, height);
    camera.aspect = width / height;
}

function build_scene() {
    const scene = new three.Scene();
    
    const light = new three.DirectionalLight(0x887766);
    light.position.set(-1, 1, 1).normalize();
    scene.add(light);

    const ambient = new three.AmbientLight(0x666666);
    scene.add(ambient);

    return scene;
}

function load_mmd_mesh(model_path: string, motion_path: string, helper: three.MMDHelper) {
    return new Promise<LoadedMMDMesh>((resolve, reject) => {
        const loader = new three.MMDLoader();
        loader.load(model_path, [motion_path], mesh => {

            const geometry = mesh.geometry;
            (geometry.getAttribute('position') as three.BufferAttribute).setDynamic(true);
            (geometry.getAttribute('normal') as three.BufferAttribute).setDynamic(true);

            // Since the result of the simulation is written directly to the vertex buffer,
            // it conflicts with morphing animation.
            // TODO: Find a solution other than deleting morphing animation.
            geometry.animations = geometry.animations.filter(c => !is_morph_animation_clip(c));

            helper.add(mesh);
            helper.setAnimation(mesh);
            helper.setPhysics(mesh, { world: build_physics_world() });
            helper.unifyAnimationDuration({ afterglow: 0.0 });

            resolve(mesh as LoadedMMDMesh);

        }, undefined, reject);
    });
}

function build_model(mmd_mesh: three.MMDMesh): Readonly<ClothModel> & Readonly<MappingInfo> {
    const geometry = mmd_mesh.geometry as three.BufferGeometry;
    const vertices = geometry.getAttribute('position').array as VertexArrayType;
    const skin_indices = geometry.getAttribute('skinIndex').array as SkinIndexArrayType;
    const skin_weights = geometry.getAttribute('skinWeight').array as WeightArrayType;
    const material_index = get_material_index_by_name(mmd_mesh.material, TargetMaterialName);
    const geo_group = get_buffer_geometry_group_by_material_id(geometry, material_index)!;
    const geo_indices = sub_typed_array(geometry.index.array as IndexArrayType, geo_group.start, geo_group.count);
    const [mesh_indices, vertex_particle_map] = create_particle_mesh_indices(geo_indices);
    const n_particle = vertex_particle_map.size;
    
    const indices_ctor = get_constructor(geo_indices);
    const vertex_idx_list = new indices_ctor(vertex_particle_map.keys());
    const particle_idx_list = new indices_ctor(vertex_particle_map.values());
    const pairs = { vertex_idx_list, particle_idx_list, length: n_particle };

    const particles = create_particle_data(n_particle);
    copy_particle_positions_from_vertices(particles, vertices, pairs, mmd_mesh.matrixWorld);

    const attachments = find_attached_particles(vertices, skin_indices, skin_weights, pairs);

    for (let i = 0; i < n_particle; ++i) {
        set_mass(particles, i, ClothMass);
    }
    for (let i = 0, len = attachments.length; i < len; ++i) {
        set_mass(particles, attachments.particle_indices[i], 0.0);
    }

    const particle_mesh = build_particle_mesh(particles, mesh_indices);

    const constraints = particle_mesh.edges.map(e => create_distance_constraint(particles, e.vertex_pair[0], e.vertex_pair[1]));

    return { particles, mesh: particle_mesh, constraints, attachments, pairs };
}

function setup_rigid_bodies(physics: three.MMDPhysics): RigidBodySet {
    const mmd_rigid_bodies = physics.bodies;

    const mmd_sphere_bodies = mmd_rigid_bodies.filter(body => body.params.shapeType === 0);
    const mmd_box_bodies = mmd_rigid_bodies.filter(body => body.params.shapeType === 1);
    const mmd_capsule_bodies = mmd_rigid_bodies.filter(body => body.params.shapeType === 2);

    const sphere_bodies = create_sphere_rigid_bodies(mmd_sphere_bodies.length);
    const box_bodies = create_box_rigid_bodies(mmd_box_bodies.length);
    const capsule_bodies = create_capsule_rigid_bodies(mmd_capsule_bodies.length);

    mmd_sphere_bodies.forEach((mmd_body, i) => {
        const params = mmd_body.params;
        const body = sphere_rigid_body_accessor(sphere_bodies, i);
        body.radius = params.width;
        sphere_bodies.bodies.push(mmd_body.body);
    });

    mmd_box_bodies.forEach((mmd_body, i) => {
        const params = mmd_body.params;
        const body = box_rigid_body_accessor(box_bodies, i);
        body.half_width = params.width;
        body.half_height = params.height;
        body.half_depth = params.depth;
        box_bodies.bodies.push(mmd_body.body);
    });

    mmd_capsule_bodies.forEach((mmd_body, i) => {
        const params = mmd_body.params;
        const body = capsule_rigid_body_accessor(capsule_bodies, i);
        body.radius = params.width;
        body.length = params.height;
        capsule_bodies.bodies.push(mmd_body.body);
    });

    // ground
    const plane_bodies = create_plane_rigid_bodies(1);
    const plane_body = plane_rigid_body_accessor(plane_bodies, 0);
    plane_body.constant = 0;
    plane_body.set_normal(0, 1, 0);

    const rigid_bodies = {
        spheres: sphere_bodies,
        sphere_accessors: [...to_enumerable(sphere_bodies, sphere_rigid_body_accessor)],
        boxes: box_bodies,
        box_accessors: [...to_enumerable(box_bodies, box_rigid_body_accessor)],
        capsules: capsule_bodies,
        capsule_accessors: [...to_enumerable(capsule_bodies, capsule_rigid_body_accessor)],
        planes: plane_bodies
    };
    update_rigid_body_transform(rigid_bodies);

    return rigid_bodies;
}

function stabilize_physics(mmd_helper: three.MMDHelper, mmd_mesh: three.MMDMesh,
    model: Readonly<ClothModel> & Readonly<MappingInfo>, rigid_bodies: RigidBodySet,
    detector: CollisionDetector, simulation_config: SimulationConfiguration, stabilize_time: number) {

    const particles = model.particles;
    const attachments = model.attachments;
    const time_step = simulation_config.time_step;

    mmd_helper.animate(time_step);
    update_rigid_body_transform(rigid_bodies);
    mmd_mesh.updateMatrixWorld(true);
    update_attached_particle(particles, attachments, mmd_mesh.skeleton);

    for (let i = 0, len = (1 / time_step) * stabilize_time; i < len; ++i) {
        step(model, rigid_bodies, detector, simulation_config);
    }
}

function update_rigid_body_transform(rigid_bodies: RigidBodySet) {
    const sphere_accessors = rigid_bodies.sphere_accessors;
    for (let i = 0, len = sphere_accessors.length; i < len; ++i) {
        const sphere = sphere_accessors[i];
        const origin = sphere.body.getCenterOfMassTransform().getOrigin();
        sphere.set_position(origin.x(), origin.y(), origin.z());
        sphere.calc_AABB();
    }
    const box_accessors = rigid_bodies.box_accessors;
    for (let i = 0, len = box_accessors.length; i < len; ++i) {
        const box = box_accessors[i];
        const transform = box.body.getCenterOfMassTransform();
        const origin = transform.getOrigin();
        const basis = transform.getBasis();
        box.set_position(origin.x(), origin.y(), origin.z());
        const x_vec = basis.getRow(0);
        box.set_basis_vector(0, x_vec.x(), x_vec.y(), x_vec.z());
        const y_vec = basis.getRow(1);
        box.set_basis_vector(1, y_vec.x(), y_vec.y(), y_vec.z());
        const z_vec = basis.getRow(2);
        box.set_basis_vector(2, z_vec.x(), z_vec.y(), z_vec.z());
        box.calc_inv_basis_matrix();
        box.calc_AABB();
    }
    const capsule_accessors = rigid_bodies.capsule_accessors;
    for (let i = 0, len = capsule_accessors.length; i < len; ++i) {
        const capsule = capsule_accessors[i];
        const transform = capsule.body.getCenterOfMassTransform();
        const origin = transform.getOrigin();
        const basis = transform.getBasis();
        capsule.set_position(origin.x(), origin.y(), origin.z());
        const x_vec = basis.getRow(0);
        const m11 = x_vec.x(), m12 = x_vec.y(), m13 = x_vec.z();
        const y_vec = basis.getRow(1);
        const m21 = y_vec.x(), m22 = y_vec.y(), m23 = y_vec.z();
        const z_vec = basis.getRow(2);
        const m31 = z_vec.x(), m32 = z_vec.y(), m33 = z_vec.z();
        const t11 = m33 * m22 - m32 * m23;
        const t12 = m32 * m13 - m33 * m12
        const t13 = m23 * m12 - m22 * m13;
        const inv_det = 1 / (m11 * t11 + m21 * t12 + m31 * t13);
        const dir_x = (m23 * m31 - m21 * m33) * inv_det;
        const dir_y = (m11 * m33 - m13 * m31 * inv_det);
        const dir_z = (m13 * m21 - m11 * m23) * inv_det;
        capsule.set_direction(dir_x, dir_y, dir_z);
        capsule.calc_AABB();
    }
}

function create_loop_runner(environment: {
        stats: Stats,
        renderer: three.WebGLRenderer,
        scene: three.Scene,
        camera: three.Camera,
        mmd_helper: three.MMDHelper,
        mmd_mesh: three.MMDMesh,
        model: Readonly<ClothModel> & Readonly<MappingInfo>,
        rigid_bodies: RigidBodySet,
        detector: CollisionDetector,
        simulation_config: SimulationConfiguration,
        debug_points: three.Points,
        rigid_body_meshes: three.Object3D[]
    }) {

    const stats = environment.stats;
    const renderer = environment.renderer;
    const scene = environment.scene;
    const camera = environment.camera;
    const mmd_helper = environment.mmd_helper;
    const mmd_mesh = environment.mmd_mesh;
    const geometry = mmd_mesh.geometry;
    const pos_attr = geometry.getAttribute('position') as three.BufferAttribute;
    const pos_buffer = pos_attr.array as VertexArrayType;
    const normal_attr = geometry.getAttribute('normal') as three.BufferAttribute;
    const normal_buffer = normal_attr.array as NormalArrayType;

    const model = environment.model;
    const particles = model.particles;
    const positions = model.particles.position;
    const masses = model.particles.mass;
    const mesh_indices = model.mesh.indices;
    const face_normals = model.mesh.face_normals;
    const vertex_normals = model.mesh.vertex_normals;
    const attachments = model.attachments;
    const vertex_particle_pairs = model.pairs;

    const rigid_bodies = environment.rigid_bodies;

    const detector = environment.detector;

    const simulation_config = environment.simulation_config;

    const debug_points_geo = environment.debug_points.geometry as three.BufferGeometry;
    const debug_points_attr = debug_points_geo.getAttribute('position') as three.BufferAttribute;

    const rigid_body_meshes = environment.rigid_body_meshes;

    const inv_transform = new three.Matrix4();

    const runner = () => {
        stats.begin();

        mmd_helper.animate(TimeStep);

        update_rigid_body_transform(rigid_bodies);

        // Force update the matrixWorld of the individual bones,
        // for updating position of attached particles.
        mmd_mesh.updateMatrixWorld(true);
        update_attached_particle(particles, attachments, mmd_mesh.skeleton);

        step(model, rigid_bodies, detector, simulation_config);

        update_face_normals(positions, mesh_indices, face_normals);
        update_vertex_normals(face_normals, mesh_indices, vertex_normals);

        update_vertex_attributes(pos_buffer, positions, normal_buffer, vertex_normals,
            masses, vertex_particle_pairs, inv_transform.getInverse(mmd_mesh.matrixWorld));

        pos_attr.needsUpdate = true;
        normal_attr.needsUpdate = true;

        if (ShowParticlePoint) {
            debug_points_attr.needsUpdate = true;
        }

        if (ShowRigidBody) {
            for (let i = 0, len = rigid_body_meshes.length; i < len; ++i) {
                rigid_body_meshes[i].userData.update();
            }
        }

        renderer.render(scene, camera);

        stats.end();

        requestAnimationFrame(runner);
    };

    return runner;
}

function is_morph_animation_clip(clip: three.AnimationClip) {
    return ( clip.tracks.length > 0 && clip.tracks[ 0 ].name.indexOf( '.morphTargetInfluences' ) === 0 );
}

function build_physics_world(): ammo.btDiscreteDynamicsWorld {
    const config = new ammo.btDefaultCollisionConfiguration();
    const dispatcher = new ammo.btCollisionDispatcher(config);
    const cache = new ammo.btDbvtBroadphase();
    const solver = new ammo.btSequentialImpulseConstraintSolver();
    const world = new ammo.btDiscreteDynamicsWorld(dispatcher, cache, solver, config);
    world.setGravity( new ammo.btVector3( 0, -9.8 * 10, 0 ) );

    const form = new ammo.btTransform();
    form.setIdentity();
    form.setOrigin(new ammo.btVector3(0, -1, 0));
    const ground = new ammo.btRigidBody(
        new ammo.btRigidBodyConstructionInfo(
            0,
            new ammo.btDefaultMotionState(form),
            new ammo.btBoxShape(new ammo.btVector3(10, 1, 10)),
            new ammo.btVector3(0, 0, 0)
        )
    );
    world.addRigidBody(ground);

    return world;
}

function get_material_index_by_name(materials: three.Material[], name: string): number {
    for (let i = 0, len = materials.length; i < len; ++i) {
        if (materials[i].name === name) return i;
    }
    return -1;
}

function get_buffer_geometry_group_by_material_id(geometry: three.BufferGeometry, material_index: number) {
    const groups = geometry.groups;
    for (const group of groups) {
        if (group.materialIndex === material_index) return group;
    }
    return undefined;
}

function create_particle_mesh_indices(geo_indices: IndexArrayType): [IndexArrayType, Map<number, number>] {
    const indices_ctor = get_constructor(geo_indices);
    const vertex_particle_map = new Map<number, number>();
    const mesh_indices = [] as number[];
    let mesh_index = 0;

    for (const geo_index of geo_indices) {
        if (vertex_particle_map.has(geo_index)) {
            mesh_indices.push(vertex_particle_map.get(geo_index)!);            
        } else {
            mesh_indices.push(mesh_index);
            vertex_particle_map.set(geo_index, mesh_index++);
        }
    }

    return [new indices_ctor(mesh_indices), vertex_particle_map] as [IndexArrayType, Map<number, number>];
}

const copy_particle_positions_from_vertices = (() => {
    const vert = new three.Vector3();
    return (particles: ParticleData, vertices: VertexArrayType, pairs: VertexParticlePairs, transform: three.Matrix4) => {
        const vert_ids = pairs.vertex_idx_list;
        const particle_ids = pairs.particle_idx_list;

        for (let i = 0, len = particles.length; i < len; ++i) {
            const X = vert_ids[i] * 3, Y = X + 1, Z = X + 2;
            vert.set(vertices[X], vertices[Y], vertices[Z]).applyMatrix4(transform);
            init_position(particles, particle_ids[i], vert.x, vert.y, vert.z);
        }
    };
})();

function find_attached_particles(vertices: VertexArrayType, skin_indices: SkinIndexArrayType, skin_weights: WeightArrayType, pairs: VertexParticlePairs): AttachedParticles {
    const attachments = [] as { particle_idx: number, bone_indices: number[], weights: number[], initial_pos: number[] }[];
    const vert_ids = pairs.vertex_idx_list;
    const particle_ids = pairs.particle_idx_list;

    for (let i = 0, len = pairs.length; i < len; ++i) {
        const vert_offset = vert_ids[i] * 3;
        const skin_offset = vert_ids[i] * 4;

        // if the vertex has been bind to bone, it is regarded as an attached particle.
        if ( ! (skin_indices[skin_offset] === 0.0 && skin_weights[skin_offset] === 1.0) ) {
            attachments.push({
                particle_idx: particle_ids[i],
                bone_indices: Array.from(skin_indices.subarray(skin_offset, skin_offset + 4)),
                weights: Array.from(skin_weights.subarray(skin_offset, skin_offset + 4)),
                initial_pos: Array.from(vertices.subarray(vert_offset, vert_offset + 3))
            });
        }
    }

    const indices_ctor = get_constructor(particle_ids);
    const length = attachments.length;
    const particle_indices = new indices_ctor(length);
    const bones_indices = new Uint32Array(length * 4);
    const bones_weights = new Float32Array(length * 4);
    const initial_positions = new Float32Array(length * 3);

    attachments.forEach((a, i) => {
        particle_indices[i] = a.particle_idx;
        bones_indices.set(a.bone_indices, i * 4);
        bones_weights.set(a.weights, i * 4);
        initial_positions.set(a.initial_pos, i * 3);
    });

    return { particle_indices, bones_indices, bones_weights, initial_positions, length };
}

const update_attached_particle = (() => {
    const transformed = new three.Vector3();
    const offset_mat = new three.Matrix4();
    const vertex = new three.Vector3();

    return (particles: ParticleData, attachments: AttachedParticles, skeleton: three.Skeleton) => {

        if (attachments.length === 0) return;

        const positions = particles.position;
        const particle_indices = attachments.particle_indices;
        const bone_indices = attachments.bones_indices;
        const bone_weights = attachments.bones_weights;
        const init_pos = attachments.initial_positions;
        const bones = skeleton.bones;
        const inv_bones = skeleton.boneInverses;

        for (let i = 0, len = attachments.length; i < len; ++i) {
            transformed.set(0, 0, 0);

            for (let j = 0; j < 4; ++j) {
                const bone_idx = bone_indices[i * 4 + j];
                const weight = bone_weights[i * 4 + j];
                if (weight === 0.0) continue;
                offset_mat.multiplyMatrices(bones[bone_idx].matrixWorld, inv_bones[bone_idx]);
                vertex.set(init_pos[i * 3], init_pos[i * 3 + 1], init_pos[i * 3 + 2]);
                vertex.applyMatrix4(offset_mat).multiplyScalar(weight);
                transformed.add(vertex);
            }

            const particle_off = particle_indices[i] * 3;
            positions[particle_off] = transformed.x;
            positions[particle_off + 1] = transformed.y;
            positions[particle_off + 2] = transformed.z;
        }
    };
})();

const update_vertex_attributes = (() => {
    const vec = new three.Vector3();
    const normal_mat = new three.Matrix3();
    return (pos_buff: VertexArrayType, positions: ParticlePositionType,
        normal_buff: NormalArrayType, normals: NormalArrayType,
        masses: Float32Array, pairs: VertexParticlePairs, inv_transform: three.Matrix4 ) => {

        const vert_ids = pairs.vertex_idx_list;
        const particle_ids = pairs.particle_idx_list;
        normal_mat.getNormalMatrix(inv_transform);

        for (let i = 0, len = pairs.length; i < len; ++i) {
            const v_i = vert_ids[i];
            const p_i = particle_ids[i];
            if (masses[p_i] !== 0.0) {
                const p_X = p_i * 3, p_Y = p_X + 1, p_Z = p_X + 2;
                const v_X = v_i * 3, v_Y = v_X + 1, v_Z = v_X + 2;
                vec.set(positions[p_X], positions[p_Y], positions[p_Z]).applyMatrix4(inv_transform);
                pos_buff[v_X] = vec.x;
                pos_buff[v_Y] = vec.y;
                pos_buff[v_Z] = vec.z;
                vec.set(normals[p_X], normals[p_Y], normals[p_Z]).applyMatrix3(normal_mat);
                normal_buff[v_X] = vec.x;
                normal_buff[v_Y] = vec.y;
                normal_buff[v_Z] = vec.z;
            }
        }
    };
})();
