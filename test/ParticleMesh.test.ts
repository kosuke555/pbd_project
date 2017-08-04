import 'mocha';
import * as assert from 'power-assert';
import { create_particle_data } from '../src/simulation/ParticleData';
import { ParticleMesh, Edge, build_particle_mesh } from '../src/simulation/ParticleMesh';

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
            mesh = build_particle_mesh(particles, new Uint32Array(indices));
        });
        it(`should return the mesh that has 16 edges`, () => assert.equal(mesh.edges.length, 16));
        it('should return the mesh that has 8 faces', () => {
            const valid_face_ids = [0xffffffff, 0, 1, 2, 3, 4, 5, 6, 7];
            mesh.edges.forEach(edge => {
                assert(valid_face_ids.findIndex(id => id === edge.face_pair[0]) !== -1);
                assert(valid_face_ids.findIndex(id => id === edge.face_pair[1]) !== -1);
            });
        });
        it('should not return the mesh that has duplication edge', () => {
            edges.forEach(e => assert(count(mesh.edges, has_vertices(e[0], e[1])) === 1));
        });
        it('should return the mesh that has edges that have adjacent faces', () => {
            assert(has_faces(0, 1)(find_edge(mesh.edges, 3, 1)!));
            assert(has_faces(1, 2)(find_edge(mesh.edges, 4, 1)!));
            assert(has_faces(2, 3)(find_edge(mesh.edges, 4, 2)!));
            assert(has_faces(1, 4)(find_edge(mesh.edges, 4, 3)!));
            assert(has_faces(4, 5)(find_edge(mesh.edges, 6, 4)!));
            assert(has_faces(5, 6)(find_edge(mesh.edges, 7, 4)!));
            assert(has_faces(6, 3)(find_edge(mesh.edges, 5, 4)!));
            assert(has_faces(6, 7)(find_edge(mesh.edges, 7, 5)!));
        });
    });
});

function count(edges: Edge[], pred: (e: Edge) => boolean) {
    let cnt = 0;
    edges.forEach(e => { if (pred(e)) ++cnt; });
    return cnt;
}

function has_vertices(vertex_a: number, vertex_b: number) {
    return (edge: Edge) => (edge.vertex_pair[0] === vertex_a && edge.vertex_pair[1] === vertex_b) ||
        (edge.vertex_pair[0] === vertex_b && edge.vertex_pair[1] === vertex_a);
}

function has_faces(face_a: number, face_b: number) {
    return (edge: Edge) => (edge.face_pair[0] === face_a && edge.face_pair[1] === face_b) ||
        (edge.face_pair[0] === face_b && edge.face_pair[1] === face_a);
}

function find_edge(edges: Edge[], vertex_a: number, vertex_b: number) {
    return edges.find(has_vertices(vertex_a, vertex_b));
}
