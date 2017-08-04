import * as three from 'three';
import Stats from 'stats.js';
import Ammo from 'ammo.js';
import { VertexArrayType, IndexArrayType, NormalArrayType, SkinIndexArrayType, WeightArrayType, ParticlePositionType } from './types';
import ClothModel from './simulation/ClothModel';
import { ParticleData, create_particle_data, reset_particle_data, init_position, set_mass } from './simulation/ParticleData';
import { build_particle_mesh, update_face_normals, update_vertex_normals } from './simulation/ParticleMesh';
import { create_distance_constraint } from './simulation/constraints';
import { step } from './simulation/time_integrator';
import { sub_typed_array, get_constructor } from './utils/typedarray_utils';
import CanvasRecorder from './utils/CanvasRecorder';

const TimeStep = 1/60;
const StepIteration = 4;
const MaxProjectionIteration = 5;

const Gravity = -9.81;
const ClothCompressionStiffness = 1.0;
const ClothStretchStiffness = 1.0;

const DefaultMMDModel = 'assets/sylvie/sylvie_merged.pmx';
const DefaultMMDMotion = 'assets/vmd/agura.vmd';
const TargetMaterialName = 'スカート';

const MMDModelMeshName = 'mmd_model';

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

    const mmd_helper = new three.MMDHelper();

    const scene = await build_scene(mmd_helper);
    const mmd_mesh = scene.getObjectByName(MMDModelMeshName) as three.SkinnedMesh;
    const geometry = mmd_mesh.geometry as three.BufferGeometry;
    const pos_attr = geometry.getAttribute('position') as three.BufferAttribute;
    const pos_buffer = pos_attr.array as VertexArrayType;
    const normal_attr = geometry.getAttribute('normal') as three.BufferAttribute;
    const normal_buffer = normal_attr.array as NormalArrayType;
    
    const model = build_model(mmd_mesh);
    const particles = model.particles;
    const positions = model.particles.position;
    const masses = model.particles.mass;
    const mesh_indices = model.mesh.indices;
    const face_normals = model.mesh.face_normals;
    const vertex_normals = model.mesh.vertex_normals;
    const attachments = model.attachments;
    const vertex_particle_pairs = model.pairs;

    const initial_positions = positions.slice();
    ((<any>mmd_mesh).mixer as three.AnimationMixer).addEventListener('loop', _ => {
        reset_particle_data(particles, initial_positions);
    });

    const points = particle_helper(particles);
    scene.add(points);
    const points_geo = points.geometry as three.BufferGeometry;
    const points_attr = points_geo.getAttribute('position') as three.BufferAttribute;

    const render = () => {
        stats.begin();

        mmd_helper.animate(TimeStep);

        // Force update the matrixWorld of the individual bones,
        // for updating position of attached particles.
        mmd_mesh.updateMatrixWorld(true);
        update_attached_particle(particles, attachments, mmd_mesh.skeleton);

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

        update_face_normals(positions, mesh_indices, face_normals);
        update_vertex_normals(face_normals, mesh_indices, vertex_normals);

        update_vertex_attributes(pos_buffer, positions, normal_buffer, vertex_normals, masses, vertex_particle_pairs);
        pos_attr.needsUpdate = true;
        normal_attr.needsUpdate = true;

        points_attr.needsUpdate = true;

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

function build_scene(helper: three.MMDHelper) {
    return new Promise<three.Scene>((resolve, reject) => {
        const scene = new three.Scene();
        
        const light = new three.DirectionalLight(0x887766);
        light.position.set(-1, 1, 1).normalize();
        const ambient = new three.AmbientLight(0x666666);
        scene.add(light);
        scene.add(ambient);

        const loader = new three.MMDLoader();
        loader.load(DefaultMMDModel, [DefaultMMDMotion], mesh => {
            mesh.name = MMDModelMeshName;
            mesh.matrixAutoUpdate = false;
            
            const geometry = mesh.geometry as three.BufferGeometry;
            (geometry.getAttribute('position') as three.BufferAttribute).setDynamic(true);
            (geometry.getAttribute('normal') as three.BufferAttribute).setDynamic(true);

            // Since the result of the simulation is written directly to the vertex buffer,
            // it conflicts with morphing animation.
            // TODO: Find a solution other than deleting morphing animation.
            const animations = (<any>geometry).animations as three.AnimationClip[];
            (<any>geometry).animations = animations.filter(c => !is_morph_animation_clip(c));

            scene.add(mesh);
            helper.add(mesh);
            helper.setAnimation(mesh);
            helper.setPhysics(mesh, { world: build_physics_world() });
            helper.unifyAnimationDuration({ afterglow: 0.0 });

            resolve(scene);

        }, undefined, reject);
    });
}

function build_model(mmd_mesh: three.SkinnedMesh): Readonly<ClothModel> & Readonly<MappingInfo> {
    const geometry = mmd_mesh.geometry as three.BufferGeometry;
    const vertices = geometry.getAttribute('position').array as VertexArrayType;
    const skin_indices = geometry.getAttribute('skinIndex').array as SkinIndexArrayType;
    const skin_weights = geometry.getAttribute('skinWeight').array as WeightArrayType;
    const materials = (<any>mmd_mesh.material) as three.Material[];
    const material_index = get_material_index_by_name(materials, TargetMaterialName);
    // (materials[material_index] as three.MeshBasicMaterial).wireframe = true;
    const geo_group = get_buffer_geometry_group_by_material_id(geometry, material_index)!;
    const geo_indices = sub_typed_array(geometry.index.array as IndexArrayType, geo_group.start, geo_group.count);
    const [mesh_indices, vertex_particle_map] = create_particle_mesh_indices(geo_indices);
    const n_particle = vertex_particle_map.size;
    
    const indices_ctor = get_constructor(geo_indices);
    const vertex_idx_list = new indices_ctor(vertex_particle_map.keys());
    const particle_idx_list = new indices_ctor(vertex_particle_map.values());
    const pairs = { vertex_idx_list, particle_idx_list, length: n_particle };

    const particles = create_particle_data(n_particle);
    copy_particle_positions_from_vertices(particles, vertices, pairs);

    const attachments = find_attached_particles(vertices, skin_indices, skin_weights, pairs);

    for (let i = 0; i < n_particle; ++i) {
        set_mass(particles, i, 1.0);
    }
    for (let i = 0, len = attachments.length; i < len; ++i) {
        set_mass(particles, attachments.particle_indices[i], 0.0);
    }

    const particle_mesh = build_particle_mesh(particles, mesh_indices);

    const constraints = particle_mesh.edges.map(e => create_distance_constraint(particles, e.vertex_pair[0], e.vertex_pair[1]));

    return { particles, mesh: particle_mesh, constraints, attachments, pairs };
}

function particle_helper(particles: ParticleData): three.Points {
    const geometry = new three.BufferGeometry();
    const pos_attr = new three.BufferAttribute(particles.position, 3);
    pos_attr.setDynamic(true);
    geometry.addAttribute('position', pos_attr);
    const colors = particles.mass.reduce((acc, m) => acc.concat(m !== 0.0 ? [1, 0, 0] : [0, 0, 1]), [] as number[]);
    geometry.addAttribute('color', new three.Float32BufferAttribute(colors, 3));
    const material = new three.PointsMaterial({ vertexColors: three.VertexColors, size: 0.3 });
    return new three.Points(geometry, material);
}

function is_morph_animation_clip(clip: three.AnimationClip) {
    return ( clip.tracks.length > 0 && clip.tracks[ 0 ].name.indexOf( '.morphTargetInfluences' ) === 0 );
}

function build_physics_world() {
    const config = new Ammo.btDefaultCollisionConfiguration();
    const dispatcher = new Ammo.btCollisionDispatcher( config );
    const cache = new Ammo.btDbvtBroadphase();
    const solver = new Ammo.btSequentialImpulseConstraintSolver();
    const world = new Ammo.btDiscreteDynamicsWorld( dispatcher, cache, solver, config );
    world.setGravity( new Ammo.btVector3( 0, -9.8 * 10, 0 ) );

    const form = new Ammo.btTransform();
    form.setIdentity();
    form.setOrigin(new Ammo.btVector3(0, -1, 0));
    const ground = new Ammo.btRigidBody(
        new Ammo.btRigidBodyConstructionInfo(
            0,
            new Ammo.btDefaultMotionState(form),
            new Ammo.btBoxShape(new Ammo.btVector3(10, 1, 10)),
            new Ammo.btVector3(0, 0, 0)
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

function copy_particle_positions_from_vertices(particles: ParticleData, vertices: VertexArrayType, pairs: VertexParticlePairs) {
    const vert_ids = pairs.vertex_idx_list;
    const particle_ids = pairs.particle_idx_list;

    for (let i = 0, len = particles.length; i < len; ++i) {
        init_position(particles, particle_ids[i], vertices, vert_ids[i]);
    }
}

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

function update_vertex_attributes(
    pos_buff: VertexArrayType, positions: ParticlePositionType,
    normal_buff: NormalArrayType, normals: NormalArrayType,
    masses: Float32Array, pairs: VertexParticlePairs ) {
    
    const vert_ids = pairs.vertex_idx_list;
    const particle_ids = pairs.particle_idx_list;
    for (let i = 0, len = pairs.length; i < len; ++i) {
        const v_i = vert_ids[i];
        const p_i = particle_ids[i];
        if (masses[p_i] !== 0.0) {
            pos_buff[v_i * 3] = positions[p_i * 3];
            pos_buff[v_i * 3 + 1] = positions[p_i * 3 + 1];
            pos_buff[v_i * 3 + 2] = positions[p_i * 3 + 2];
            normal_buff[v_i * 3] = normals[p_i * 3];
            normal_buff[v_i * 3 + 1] = normals[p_i * 3 + 1];
            normal_buff[v_i * 3 + 2] = normals[p_i * 3 + 2];
        }
    }
}
