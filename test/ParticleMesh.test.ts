import 'mocha';
import * as assert from 'power-assert';
import { create_particle_data } from '../src/simulation/ParticleData';
import { ParticleMesh, Edge, Face, build_particle_mesh } from '../src/simulation/ParticleMesh';

describe('ParticleMesh', () => {
    describe('#build_particle_mesh', () => {
        /**
         * Test triangle mesh
         *  Vertex: 9
         *  Edge: 16
         *  Face: 8
         *  v0   v1   v2
         *    *----*----*
         *    |  ／|  ／|
         *  v3|／v4|／v5|
         *    *----*----*
         *    |  ／|  ／|
         *  v6|／v7|／v8|
         *    *----*----*
         */
        const particles = create_particle_data(3 * 3);
        const indices = [0, 3, 1, 1, 3, 4, 1, 4, 2, 2, 4, 5, 3, 6, 4, 4, 6, 7, 4, 7, 5, 5, 7, 8];
        const edges = [
            [0, 3], [3, 1], [1, 0], [3, 4],
            [4, 1], [4, 2], [2, 1], [4, 5],
            [5, 2], [3, 6], [6, 4], [6, 7],
            [7, 4], [7, 5], [7, 8], [8, 5]
        ];
        let mesh: ParticleMesh;
        before(() => {
            mesh = build_particle_mesh(particles, new Uint16Array(indices));
        });
        it(`should return a mesh that has 16 edges.`, () => assert.equal(mesh.edges.length, 16));
        it('should return a mesh that has 8 faces.', () => assert.equal(mesh.faces.length, 8));
        it('should not return a mesh that has duplication edge.', () => {
            assert(edges.every(e => count_if(mesh.edges, edge_consist_of_vertices(e[0], e[1])) === 1));
        });
        it('should return a mesh with a edge that has adjacent faces.', () => {
            const edges = mesh.edges;
            assert([
                find_edge(edges, [3, 1], [0, 1]),
                find_edge(edges, [4, 1], [1, 2]),
                find_edge(edges, [4, 2], [2, 3]),
                find_edge(edges, [4, 3], [1, 4]),
                find_edge(edges, [6, 4], [4, 5]),
                find_edge(edges, [7, 4], [5, 6]),
                find_edge(edges, [5, 4], [6, 3]),
                find_edge(edges, [7, 5], [6, 7])
            ].every(r => r));
        });
        it('should return a mesh with a face with vertices that make up the face.', () => {
            const faces = mesh.faces;
            assert([
                find_face(faces, [1, 0, 3]),
                find_face(faces, [1, 3, 4]),
                find_face(faces, [2, 1, 4]),
                find_face(faces, [2, 4, 5]),
                find_face(faces, [4, 3, 6]),
                find_face(faces, [4, 6, 7]),
                find_face(faces, [5, 4, 7]),
                find_face(faces, [5, 7, 8])
            ].every(r => r));
        });
    });
});

function count_if<T>(array: T[], pred: (e: T) => boolean) {
    let cnt = 0;
    array.forEach(e => { if (pred(e)) ++cnt; });
    return cnt;
}

function edge_consist_of_vertices(vertex_a: number, vertex_b: number) {
    return (edge: Edge) => (edge.vertex_pair[0] === vertex_a && edge.vertex_pair[1] === vertex_b) ||
        (edge.vertex_pair[0] === vertex_b && edge.vertex_pair[1] === vertex_a);
}

function face_consist_of_vertices(vertices: number[]) {
    return (face: Face) =>
        (face.vertices[0] === vertices[0] && face.vertices[1] === vertices[1] && face.vertices[2] === vertices[2])
        || (face.vertices[0] === vertices[1] && face.vertices[1] === vertices[2] && face.vertices[2] === vertices[0])
        || (face.vertices[0] === vertices[2] && face.vertices[1] === vertices[0] && face.vertices[2] === vertices[1]);
}

function edge_has_faces(edge: Edge, face_a: number, face_b: number) {
    return (edge.face_pair[0] === face_a && edge.face_pair[1] === face_b) ||
        (edge.face_pair[0] === face_b && edge.face_pair[1] === face_a);
}

function find_edge(edges: Edge[], vertices: number[], faces: number[]) {
    const edge = edges.find(edge_consist_of_vertices(vertices[0], vertices[1]));
    if (edge) return edge_has_faces(edge, faces[0], faces[1]);
    else return false;
}

function find_face(faces: Face[], vertices: number[]) {
    return faces.find(face_consist_of_vertices(vertices)) !== undefined;
}
