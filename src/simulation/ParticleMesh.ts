import { IndexArrayType, NormalArrayType, ParticlePositionType } from '../types';
import { ParticleData } from './ParticleData';
import { copy_vec3_gen, sub_vec3, cross_vec3, normalize_vec3_gen, add_to_vec3_gen } from './math';

export interface Edge {
    vertex_pair: number[];
    face_pair: number[];
}

export interface Face {
    vertices: number[];
}

export interface ParticleMesh {
    indices: IndexArrayType;
    edges: Readonly<Edge>[];
    faces: Readonly<Face>[];
    face_normals: NormalArrayType;
    vertex_normals: NormalArrayType;
}

export const NullFaceID = 0xffffffff;

export function build_particle_mesh(particles: ParticleData, indices: IndexArrayType): Readonly<ParticleMesh> {
    const num_faces = indices.length / 3;

    const faces = [...Array(num_faces).keys()]  // 0..(num of faces - 1)
    .map(index => ({
        vertices: [indices[index * 3], indices[index * 3 + 1], indices[index * 3 + 2]]
    }));

    const overlapped_edges = faces
    .map(face => [
        [face.vertices[0], face.vertices[1]],  // edge 0-1
        [face.vertices[1], face.vertices[2]],  // edge 1-2
        [face.vertices[2], face.vertices[0]]  // edge 2-0
    ])
    .reduce((a, b) => a.concat(b));  // flatten

    const p_edges = new Map<number, Edge[]>();
    const edges = [] as Edge[];
    for (let i = 0, len = overlapped_edges.length; i < len; ++i) {
        const a = overlapped_edges[i][0];
        const b = overlapped_edges[i][1];
        const face_id = Math.floor(i / 3);
        const edge = find_edge(p_edges, a, b);
        if (!edge) {
            const new_edge = {
                vertex_pair: [a, b],
                face_pair: [face_id, NullFaceID]
            };
            edges.push(new_edge);
            add_edge_to_map(p_edges, a, b, new_edge);
        } else {
            edge.face_pair[1] = face_id;
        }
    }

    const face_normals = new Float32Array(num_faces * 3);
    const vertex_normals = new Float32Array(particles.length * 3);

    update_face_normals(particles.position, indices, face_normals);
    update_vertex_normals(face_normals, indices, vertex_normals);

    return { indices, edges, faces, face_normals, vertex_normals };
}

const p_a = new Float32Array(3);
const p_b = new Float32Array(3);
const p_c = new Float32Array(3);
const v1 = new Float32Array(3);
const v2 = new Float32Array(3);
export function update_face_normals(positions: ParticlePositionType, indices: IndexArrayType, face_normals: NormalArrayType) {
    const num_faces = indices.length / 3;
    for (let i = 0; i < num_faces; ++i) {
        copy_vec3_gen(p_a, 0, positions, indices[i * 3]);
        copy_vec3_gen(p_b, 0, positions, indices[i * 3 + 1]);
        copy_vec3_gen(p_c, 0, positions, indices[i * 3 + 2]);
        sub_vec3(p_b, p_a, v1, 0);
        sub_vec3(p_c, p_a, v2, 0);
        cross_vec3(v1, v2, face_normals, i);
        normalize_vec3_gen(face_normals, i);
    }
}

const n = new Float32Array(3);
export function update_vertex_normals(face_normals: NormalArrayType, indices: IndexArrayType, vertex_normals: NormalArrayType) {
    for (let i = 0, len = vertex_normals.length; i < len; ++i) {
        vertex_normals[i] = 0;
    }

    const num_faces = indices.length / 3;
    for (let i = 0; i < num_faces; ++i) {
        copy_vec3_gen(n, 0, face_normals, i);
        add_to_vec3_gen(vertex_normals, indices[i * 3], n, 0);
        add_to_vec3_gen(vertex_normals, indices[i * 3 + 1], n, 0);
        add_to_vec3_gen(vertex_normals, indices[i * 3 + 2], n, 0);
    }

    const num_vertices = vertex_normals.length / 3;
    for (let i = 0; i < num_vertices; ++i) {
        normalize_vec3_gen(vertex_normals, i);
    }
}

function find_edge(vertex_edge_map: Map<number, Edge[]>, a: number, b: number) {
    if (vertex_edge_map.has(a)) {
        const edges = vertex_edge_map.get(a)!;
        for (let i = 0, len = edges.length; i < len; ++i) {
            if ((edges[i].vertex_pair[0] === a && edges[i].vertex_pair[1] === b) ||
            (edges[i].vertex_pair[0] === b && edges[i].vertex_pair[1] === a)) {
                return edges[i];
            }
        }
    }
    return undefined;
}

function add_edge_to_map(vertex_edge_map: Map<number, Edge[]>, key_a: number, key_b: number, edge: Edge) {
    const edge_list_a = vertex_edge_map.get(key_a) || [];
    edge_list_a.push(edge);
    vertex_edge_map.set(key_a, edge_list_a);

    const edge_list_b = vertex_edge_map.get(key_b) || [];
    edge_list_b.push(edge);
    vertex_edge_map.set(key_b, edge_list_b);
}
