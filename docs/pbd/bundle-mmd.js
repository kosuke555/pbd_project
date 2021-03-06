(function (three,Stats,ammo) {
'use strict';

Stats = Stats && Stats.hasOwnProperty('default') ? Stats['default'] : Stats;

const EPSILON = 0.0000001;

function set_vec3_gen(dst, dst_index, x, y, z) {
    dst[dst_index * 3] = x;
    dst[dst_index * 3 + 1] = y;
    dst[dst_index * 3 + 2] = z;
    return dst;
}
function get_vec3_array_gen(src, src_index) {
    return [
        src[src_index * 3],
        src[src_index * 3 + 1],
        src[src_index * 3 + 2]
    ];
}
function copy_vec3_gen(dst, dst_index, src, src_index) {
    dst[dst_index * 3] = src[src_index * 3];
    dst[dst_index * 3 + 1] = src[src_index * 3 + 1];
    dst[dst_index * 3 + 2] = src[src_index * 3 + 2];
    return dst;
}

function add_to_vec3_gen(dst, dst_index, b, b_index) {
    dst[dst_index * 3] += b[b_index * 3];
    dst[dst_index * 3 + 1] += b[b_index * 3 + 1];
    dst[dst_index * 3 + 2] += b[b_index * 3 + 2];
    return dst;
}
function sub_vec3(a, b, dst, dst_index) {
    dst[dst_index * 3] = a[0] - b[0];
    dst[dst_index * 3 + 1] = a[1] - b[1];
    dst[dst_index * 3 + 2] = a[2] - b[2];
    return dst;
}





function cross_vec3(a, b, dst, dst_index) {
    dst[dst_index * 3] = a[1] * b[2] - a[2] * b[1];
    dst[dst_index * 3 + 1] = a[2] * b[0] - a[0] * b[2];
    dst[dst_index * 3 + 2] = a[0] * b[1] - a[1] * b[0];
    return dst;
}
function distance_vectors_vec3(vectors, p1, p2) {
    const x = vectors[p1 * 3] - vectors[p2 * 3];
    const y = vectors[p1 * 3 + 1] - vectors[p2 * 3 + 1];
    const z = vectors[p1 * 3 + 2] - vectors[p2 * 3 + 2];
    return Math.sqrt(x * x + y * y + z * z);
}

function length_vec3_gen(vectors, index) {
    const x = vectors[index * 3];
    const y = vectors[index * 3 + 1];
    const z = vectors[index * 3 + 2];
    return Math.sqrt(x * x + y * y + z * z);
}
function normalize_vec3_gen(vectors, index) {
    const l = 1 / length_vec3_gen(vectors, index);
    vectors[index * 3] *= l;
    vectors[index * 3 + 1] *= l;
    vectors[index * 3 + 2] *= l;
    return vectors;
}
const cot_theta = (() => {
    const vec = new three.Vector3();
    return (v, w) => {
        const cos = v.dot(w);
        const sin = vec.crossVectors(v, w).length();
        return cos / sin;
    };
})();

function create_particle_data(n_particle) {
    const mass = new Float32Array(n_particle);
    const inv_mass = new Float32Array(n_particle);
    const velocity = new Float32Array(n_particle * 3);
    const position = new Float32Array(n_particle * 3);
    const old_position = new Float32Array(n_particle * 3);
    const extra_vec = new Float32Array(n_particle * 3);
    const extra_uint8 = new Uint8Array(n_particle);
    const extra_float = new Float32Array(n_particle);
    const length = n_particle;
    return { mass, inv_mass, velocity, position, old_position, extra_vec, extra_uint8, extra_float, length };
}
function reset_particle_data(particles, initial_positions) {
    particles.position.set(initial_positions);
    particles.old_position.set(initial_positions);
    particles.velocity.fill(0);
    particles.extra_vec.fill(0);
    particles.extra_uint8.fill(0);
    particles.extra_float.fill(0);
}
function particle_accessor(target, index) {
    return {
        get mass() { return target.mass[index]; },
        set mass(mass) { set_mass(target, index, mass); },
        get inv_mass() { return target.inv_mass[index]; },
        get_velocity() {
            return get_vec3_array_gen(target.velocity, index);
        },
        set_velocity(x, y, z) {
            set_vec3_gen(target.velocity, index, x, y, z);
        },
        get_position() {
            return get_vec3_array_gen(target.position, index);
        },
        set_position(x, y, z) {
            set_vec3_gen(target.position, index, x, y, z);
        },
        get_old_position() {
            return get_vec3_array_gen(target.old_position, index);
        },
        set_old_position(x, y, z) {
            set_vec3_gen(target.old_position, index, x, y, z);
        },
        init_position(x_or_vec, y_or_v_idx, z) {
            init_position(target, index, x_or_vec, y_or_v_idx, z);
        }
    };
}
function init_position(particle_data, p_index, x_or_vec, y_or_v_idx, _z) {
    const pos = particle_data.position;
    const old_pos = particle_data.old_position;
    if (typeof x_or_vec === 'number') {
        const x = x_or_vec, y = y_or_v_idx, z = _z;
        pos[p_index * 3] = x;
        pos[p_index * 3 + 1] = y;
        pos[p_index * 3 + 2] = z;
        old_pos[p_index * 3] = x;
        old_pos[p_index * 3 + 1] = y;
        old_pos[p_index * 3 + 2] = z;
    }
    else {
        const vectors = x_or_vec;
        const v_index = y_or_v_idx;
        pos[p_index * 3] = vectors[v_index * 3];
        pos[p_index * 3 + 1] = vectors[v_index * 3 + 1];
        pos[p_index * 3 + 2] = vectors[v_index * 3 + 2];
        old_pos[p_index * 3] = vectors[v_index * 3];
        old_pos[p_index * 3 + 1] = vectors[v_index * 3 + 1];
        old_pos[p_index * 3 + 2] = vectors[v_index * 3 + 2];
    }
}
/**
 * set particle mass.
 * mass == 0.0 is static particle.
 * @param particle_data target particle data
 * @param index particle index
 * @param mass particle mass
 */
function set_mass(particle_data, index, mass) {
    particle_data.mass[index] = mass;
    particle_data.inv_mass[index] = mass !== 0.0 ? 1.0 / mass : 0.0;
}

const NullFaceID = 0xffffffff;
function build_particle_mesh(particles, indices) {
    const num_faces = indices.length / 3;
    const faces = [...Array(num_faces).keys()] // 0..(num of faces - 1)
        .map(index => ({
        vertices: [indices[index * 3], indices[index * 3 + 1], indices[index * 3 + 2]]
    }));
    const overlapped_edges = faces
        .map(face => [
        [face.vertices[0], face.vertices[1]],
        [face.vertices[1], face.vertices[2]],
        [face.vertices[2], face.vertices[0]] // edge 2-0
    ])
        .reduce((a, b) => a.concat(b)); // flatten
    const p_edges = new Map();
    const edges = [];
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
        }
        else {
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
function update_face_normals(positions, indices, face_normals) {
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
function update_vertex_normals(face_normals, indices, vertex_normals) {
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
function find_edge(vertex_edge_map, a, b) {
    if (vertex_edge_map.has(a)) {
        const edges = vertex_edge_map.get(a);
        for (let i = 0, len = edges.length; i < len; ++i) {
            if ((edges[i].vertex_pair[0] === a && edges[i].vertex_pair[1] === b) ||
                (edges[i].vertex_pair[0] === b && edges[i].vertex_pair[1] === a)) {
                return edges[i];
            }
        }
    }
    return undefined;
}
function add_edge_to_map(vertex_edge_map, key_a, key_b, edge) {
    const edge_list_a = vertex_edge_map.get(key_a) || [];
    edge_list_a.push(edge);
    vertex_edge_map.set(key_a, edge_list_a);
    const edge_list_b = vertex_edge_map.get(key_b) || [];
    edge_list_b.push(edge);
    vertex_edge_map.set(key_b, edge_list_b);
}

class SimpleAllocator {
    constructor(object_creator) {
        this.object_creator = object_creator;
        this._usage_count = 0;
        this._objects = [];
    }
    dispose() {
        this._usage_count = 0;
        this._objects.length = 0;
    }
    get() {
        if (this._usage_count < this._objects.length) {
            return this._objects[this._usage_count++];
        }
        else {
            const obj = this.object_creator();
            this._objects.push(obj);
            ++this._usage_count;
            return obj;
        }
    }
    release_all() {
        this._usage_count = 0;
    }
}

const sqrt = Math.sqrt;
function create_distance_constraint(particles, particle1, particle2) {
    const rest_len = distance_vectors_vec3(particles.position, particle1, particle2);
    return { rest_len, particle1, particle2, solver: solve_distance_constraint };
}
function solve_distance_constraint(positions, inv_mass, stiffnesses) {
    const constraint = this;
    const p1 = constraint.particle1;
    const p2 = constraint.particle2;
    const rest_len = constraint.rest_len;
    const imass1 = inv_mass[p1];
    const imass2 = inv_mass[p2];
    const sum_imass = imass1 + imass2;
    if (sum_imass == 0.0)
        return;
    const p1_x = positions[p1 * 3];
    const p1_y = positions[p1 * 3 + 1];
    const p1_z = positions[p1 * 3 + 2];
    const p2_x = positions[p2 * 3];
    const p2_y = positions[p2 * 3 + 1];
    const p2_z = positions[p2 * 3 + 2];
    const x = p2_x - p1_x;
    const y = p2_y - p1_y;
    const z = p2_z - p1_z;
    const d = sqrt(x * x + y * y + z * z);
    const nx = x / d;
    const ny = y / d;
    const nz = z / d;
    const stiff = d < rest_len ? stiffnesses.compression : stiffnesses.stretch;
    const a = stiff * (d - rest_len) / sum_imass;
    const cx = a * nx;
    const cy = a * ny;
    const cz = a * nz;
    if (imass1 !== 0.0) {
        positions[p1 * 3] = p1_x + imass1 * cx;
        positions[p1 * 3 + 1] = p1_y + imass1 * cy;
        positions[p1 * 3 + 2] = p1_z + imass1 * cz;
    }
    if (imass2 !== 0.0) {
        positions[p2 * 3] = p2_x + -imass2 * cx;
        positions[p2 * 3 + 1] = p2_y + -imass2 * cy;
        positions[p2 * 3 + 2] = p2_z + -imass2 * cz;
    }
}
const create_isometric_bending_constraint = (() => {
    const vec = new three.Vector3();
    return (particles, p1, p2, p3, p4) => {
        const x_s = [
            new three.Vector3().fromArray(particles[p1].get_position()),
            new three.Vector3().fromArray(particles[p2].get_position()),
            new three.Vector3().fromArray(particles[p3].get_position()),
            new three.Vector3().fromArray(particles[p4].get_position())
        ];
        const e_s = [
            new three.Vector3().subVectors(x_s[0], x_s[1]),
            new three.Vector3().subVectors(x_s[2], x_s[1]),
            new three.Vector3().subVectors(x_s[0], x_s[2]),
            new three.Vector3().subVectors(x_s[0], x_s[3]),
            new three.Vector3().subVectors(x_s[3], x_s[1])
        ];
        const c01 = cot_theta(e_s[0], e_s[1]);
        const c02 = cot_theta(e_s[0], e_s[2]);
        const c03 = cot_theta(e_s[0], e_s[3]);
        const c04 = cot_theta(e_s[0], e_s[4]);
        const K = [c01 + c04, c02 + c03, -c01 - c02, -c03 - c04];
        const Q_s = new Float32Array(16);
        for (let i = 0; i < 4; ++i) {
            for (let j = 0; j < 4; ++j) {
                Q_s[i * 4 + j] = K[i] * K[j];
            }
        }
        const A0 = vec.crossVectors(e_s[0], e_s[1]).length() * 0.5;
        const A1 = vec.crossVectors(e_s[0], e_s[4]).length() * 0.5;
        const coef = 3 / (A0 + A1);
        Q_s.forEach((_, i, Q_s) => Q_s[i] *= coef);
        return { p_s: [p1, p2, p3, p4], Q_s, solver: solve_isometric_bending_constraint };
    };
})();
const solve_isometric_bending_constraint = (() => {
    const grad_C = new Float32Array(12);
    return function (positions, inv_mass, stiffnesses) {
        const constraint = this;
        const p_s = constraint.p_s;
        const Q_s = constraint.Q_s;
        const x = positions;
        let energy = 0;
        for (let i = 0; i < 4; ++i) {
            const i_X = p_s[i] * 3, i_Y = i_X + 1, i_Z = i_X + 2;
            for (let j = 0; j < 4; ++j) {
                const j_X = p_s[j] * 3, j_Y = j_X + 1, j_Z = j_X + 2;
                energy += Q_s[i * 4 + j] * (x[i_X] * x[j_X] + x[i_Y] * x[j_Y] + x[i_Z] * x[j_Z]);
            }
        }
        energy *= 0.5;
        grad_C.fill(0);
        for (let i = 0; i < 4; ++i) {
            const i_X = i * 3, i_Y = i_X + 1, i_Z = i_X + 2;
            for (let j = 0; j < 4; ++j) {
                const j_X = p_s[j] * 3, j_Y = j_X + 1, j_Z = j_X + 2;
                grad_C[i_X] += Q_s[i * 4 + j] * x[j_X];
                grad_C[i_Y] += Q_s[i * 4 + j] * x[j_Y];
                grad_C[i_Z] += Q_s[i * 4 + j] * x[j_Z];
            }
        }
        let grad_C_sum = 0;
        for (let i = 0; i < 4; ++i) {
            const i_X = i * 3, i_Y = i_X + 1, i_Z = i_X + 2;
            const grad_C_x = grad_C[i_X], grad_C_y = grad_C[i_Y], grad_C_z = grad_C[i_Z];
            grad_C_sum += inv_mass[p_s[i]] * (grad_C_x * grad_C_x + grad_C_y * grad_C_y + grad_C_z * grad_C_z);
        }
        if (grad_C_sum < EPSILON)
            return;
        const s = energy / grad_C_sum;
        const stiff = stiffnesses.bending;
        for (let i = 0; i < 4; ++i) {
            const w_i = inv_mass[p_s[i]];
            if (w_i !== 0.0) {
                const p_i_X = p_s[i] * 3, p_i_Y = p_i_X + 1, p_i_Z = p_i_X + 2;
                const i_X = i * 3, i_Y = i_X + 1, i_Z = i_X + 2;
                positions[p_i_X] += -stiff * s * w_i * grad_C[i_X];
                positions[p_i_Y] += -stiff * s * w_i * grad_C[i_Y];
                positions[p_i_Z] += -stiff * s * w_i * grad_C[i_Z];
            }
        }
    };
})();
class ContactConstraintAllocator extends SimpleAllocator {
    constructor() {
        super(() => {
            const buffer = new ArrayBuffer(36); // 3 float arrays, each has 3 elements
            return {
                particle: 0,
                normal: new Float32Array(buffer, 0, 3),
                contact_point: new Float32Array(buffer, 12, 3),
                contact_normal: new Float32Array(buffer, 24, 3)
            };
        });
    }
}
function create_contact_constraint(particle, n_x, n_y, n_z, cp_x, cp_y, cp_z, cn_x, cn_y, cn_z, allocator) {
    const constraint = allocator.get();
    constraint.particle = particle;
    const normal = constraint.normal;
    normal[0] = n_x, normal[1] = n_y, normal[2] = n_z;
    const contact_point = constraint.contact_point;
    contact_point[0] = cp_x, contact_point[1] = cp_y, contact_point[2] = cp_z;
    const contact_normal = constraint.contact_normal;
    contact_normal[0] = cn_x, contact_normal[1] = cn_y, contact_normal[2] = cn_z;
    return constraint;
}
function solve_contact_constraint(constraint, positions, inv_mass, contact_normals, depths) {
    const p = constraint.particle;
    if (inv_mass[p] === 0.0)
        return;
    const C = contact_constraint_func(constraint, positions);
    if (C >= 0.0)
        return;
    const X = p * 3, Y = X + 1, Z = X + 2;
    const p_x = positions[X];
    const p_y = positions[Y];
    const p_z = positions[Z];
    const normals = constraint.normal;
    const nx = normals[0];
    const ny = normals[1];
    const nz = normals[2];
    positions[X] = p_x - C * nx;
    positions[Y] = p_y - C * ny;
    positions[Z] = p_z - C * nz;
    const cn = constraint.contact_normal;
    contact_normals[X] += cn[0];
    contact_normals[Y] += cn[1];
    contact_normals[Z] += cn[2];
    depths[p] -= C;
}
function prestabilize_contact_constraint(constraint, positions, delta_positions, n_collisions, inv_mass) {
    const p = constraint.particle;
    if (inv_mass[p] === 0.0)
        return;
    const C = contact_constraint_func(constraint, positions);
    if (C >= 0.0)
        return;
    const normals = constraint.normal;
    const nx = normals[0];
    const ny = normals[1];
    const nz = normals[2];
    delta_positions[p * 3] += -C * nx;
    delta_positions[p * 3 + 1] += -C * ny;
    delta_positions[p * 3 + 2] += -C * nz;
    ++n_collisions[p];
}
function contact_constraint_func(constraint, positions) {
    const p = constraint.particle;
    const p_x = positions[p * 3];
    const p_y = positions[p * 3 + 1];
    const p_z = positions[p * 3 + 2];
    const contact_point = constraint.contact_point;
    const cp_x = contact_point[0];
    const cp_y = contact_point[1];
    const cp_z = contact_point[2];
    const contact_normal = constraint.contact_normal;
    const cn_x = contact_normal[0];
    const cn_y = contact_normal[1];
    const cn_z = contact_normal[2];
    return (p_x - cp_x) * cn_x + (p_y - cp_y) * cn_y + (p_z - cp_z) * cn_z;
}

const min = Math.min;
const max = Math.max;
function create_sphere_rigid_bodies(n_bodies) {
    const radiuses = new Float32Array(n_bodies);
    const positions = new Float32Array(n_bodies * 3);
    const AABBs = new Float32Array(n_bodies * 3 * 2);
    const tolerances = new Float32Array(n_bodies);
    return { bodies: [], radiuses, positions, AABBs, tolerances, length: n_bodies };
}
function sphere_rigid_body_accessor(target, index) {
    return {
        get body() { return target.bodies[index]; },
        get radius() { return target.radiuses[index]; },
        set radius(rad) { target.radiuses[index] = rad; },
        get tolerance() { return target.tolerances[index]; },
        set tolerance(tol) { target.tolerances[index] = tol; },
        get_position() {
            return get_vec3_array_gen(target.positions, index);
        },
        set_position(x, y, z) {
            set_vec3_gen(target.positions, index, x, y, z);
        },
        get_AABB() {
            return get_AABB(target.AABBs, index);
        },
        calc_AABB() {
            const AABBs = target.AABBs;
            const positions = target.positions;
            const radius = target.radiuses[index] + target.tolerances[index];
            const min_index = index * 3 * 2;
            const max_index = index * 3 * 2 + 3;
            const X = index * 3, Y = X + 1, Z = X + 2;
            AABBs[min_index] = positions[X] - radius;
            AABBs[min_index + 1] = positions[Y] - radius;
            AABBs[min_index + 2] = positions[Z] - radius;
            AABBs[max_index] = positions[X] + radius;
            AABBs[max_index + 1] = positions[Y] + radius;
            AABBs[max_index + 2] = positions[Z] + radius;
        }
    };
}
function create_box_rigid_bodies(n_bodies) {
    const half_widths = new Float32Array(n_bodies);
    const half_heights = new Float32Array(n_bodies);
    const half_depths = new Float32Array(n_bodies);
    const positions = new Float32Array(n_bodies * 3);
    const basis_matrices = new Float32Array(n_bodies * 9);
    const inv_basis_mats = new Float32Array(n_bodies * 9);
    const AABBs = new Float32Array(n_bodies * 3 * 2);
    const tolerances = new Float32Array(n_bodies);
    return { bodies: [], half_widths, half_heights, half_depths, positions, basis_matrices, inv_basis_mats, AABBs, tolerances, length: n_bodies };
}
function box_rigid_body_accessor(target, index) {
    return {
        get body() { return target.bodies[index]; },
        get half_width() { return target.half_widths[index]; },
        set half_width(width) { target.half_widths[index] = width; },
        get half_height() { return target.half_heights[index]; },
        set half_height(height) { target.half_heights[index] = height; },
        get half_depth() { return target.half_depths[index]; },
        set half_depth(depth) { target.half_depths[index] = depth; },
        get tolerance() { return target.tolerances[index]; },
        set tolerance(tol) { target.tolerances[index] = tol; },
        get_position() {
            return get_vec3_array_gen(target.positions, index);
        },
        set_position(x, y, z) {
            set_vec3_gen(target.positions, index, x, y, z);
        },
        get_basis_vector(basis_index) {
            const basis_matrices = target.basis_matrices;
            const idx = index * 9;
            return [
                basis_matrices[idx + basis_index],
                basis_matrices[idx + basis_index + 3],
                basis_matrices[idx + basis_index + 6]
            ];
        },
        set_basis_vector(basis_index, x, y, z) {
            const basis_matrices = target.basis_matrices;
            const idx = index * 9;
            basis_matrices[idx + basis_index] = x;
            basis_matrices[idx + basis_index + 3] = y;
            basis_matrices[idx + basis_index + 6] = z;
        },
        get_inv_basis_vector(basis_index) {
            const inv_basis_matrices = target.inv_basis_mats;
            const idx = index * 9;
            return [
                inv_basis_matrices[idx + basis_index],
                inv_basis_matrices[idx + basis_index + 3],
                inv_basis_matrices[idx + basis_index + 6]
            ];
        },
        calc_inv_basis_matrix: (() => {
            const mat_a = new three.Matrix3();
            const mat_b = new three.Matrix3();
            const byte_offset = index * 9 * 4;
            mat_a.elements = new Float32Array(target.inv_basis_mats.buffer, byte_offset, 9);
            mat_b.elements = new Float32Array(target.basis_matrices.buffer, byte_offset, 9);
            return () => {
                mat_a.getInverse(mat_b);
            };
        })(),
        get_AABB() {
            return get_AABB(target.AABBs, index);
        },
        calc_AABB: (() => {
            const min_vec = new Float32Array(3);
            const max_vec = new Float32Array(3);
            const AABBs = target.AABBs;
            const positions = target.positions;
            const half_widths = target.half_widths;
            const half_heights = target.half_heights;
            const half_depths = target.half_depths;
            const tolerances = target.tolerances;
            const basis_matrices = target.basis_matrices;
            const min_index = index * 3 * 2;
            const max_index = index * 3 * 2 + 3;
            const idx = index * 9;
            return () => {
                const tolerance = tolerances[index];
                min_vec[0] = -half_widths[index] - tolerance;
                min_vec[1] = -half_heights[index] - tolerance;
                min_vec[2] = -half_depths[index] - tolerance;
                max_vec[0] = half_widths[index] + tolerance;
                max_vec[1] = half_heights[index] + tolerance;
                max_vec[2] = half_depths[index] + tolerance;
                // http://dev.theomader.com/transform-bounding-boxes/
                for (let j = 0; j < 3; ++j) {
                    AABBs[min_index + j] = positions[index * 3 + j];
                    AABBs[max_index + j] = positions[index * 3 + j];
                    for (let i = 0; i < 3; ++i) {
                        const a = basis_matrices[idx + i * 3 + j] * min_vec[i];
                        const b = basis_matrices[idx + i * 3 + j] * max_vec[i];
                        if (a < b) {
                            AABBs[min_index + j] += a;
                            AABBs[max_index + j] += b;
                        }
                        else {
                            AABBs[min_index + j] += b;
                            AABBs[max_index + j] += a;
                        }
                    }
                }
            };
        })()
    };
}
function create_capsule_rigid_bodies(n_bodies) {
    const radiuses = new Float32Array(n_bodies);
    const lengths = new Float32Array(n_bodies);
    const half_lengths = new Float32Array(n_bodies);
    const positions = new Float32Array(n_bodies * 3);
    const directions = new Float32Array(n_bodies * 3);
    const AABBs = new Float32Array(n_bodies * 3 * 2);
    const tolerances = new Float32Array(n_bodies);
    return { bodies: [], radiuses, lengths, half_lengths, positions, directions, AABBs, tolerances, length: n_bodies };
}
function capsule_rigid_body_accessor(target, index) {
    return {
        get body() { return target.bodies[index]; },
        get radius() { return target.radiuses[index]; },
        set radius(radius) { target.radiuses[index] = radius; },
        get length() { return target.lengths[index]; },
        set length(length) {
            target.lengths[index] = length;
            target.half_lengths[index] = length * 0.5;
        },
        get half_length() { return target.half_lengths[index]; },
        get tolerance() { return target.tolerances[index]; },
        set tolerance(tol) { target.tolerances[index] = tol; },
        get_position() {
            return get_vec3_array_gen(target.positions, index);
        },
        set_position(x, y, z) {
            set_vec3_gen(target.positions, index, x, y, z);
        },
        get_direction() {
            return get_vec3_array_gen(target.directions, index);
        },
        set_direction(x, y, z) {
            set_vec3_gen(target.directions, index, x, y, z);
        },
        get_AABB() {
            return get_AABB(target.AABBs, index);
        },
        calc_AABB() {
            const positions = target.positions;
            const directions = target.directions;
            const half_length = target.half_lengths[index];
            const radius = target.radiuses[index] + target.tolerances[index];
            const X = index * 3, Y = X + 1, Z = X + 2;
            const p0_x = positions[X] + directions[X] * half_length;
            const p0_y = positions[Y] + directions[Y] * half_length;
            const p0_z = positions[Z] + directions[Z] * half_length;
            const p1_x = positions[X] - directions[X] * half_length;
            const p1_y = positions[Y] - directions[Y] * half_length;
            const p1_z = positions[Z] - directions[Z] * half_length;
            const s0_min_x = p0_x - radius;
            const s0_min_y = p0_y - radius;
            const s0_min_z = p0_z - radius;
            const s0_max_x = p0_x + radius;
            const s0_max_y = p0_y + radius;
            const s0_max_z = p0_z + radius;
            const s1_min_x = p1_x - radius;
            const s1_min_y = p1_y - radius;
            const s1_min_z = p1_z - radius;
            const s1_max_x = p1_x + radius;
            const s1_max_y = p1_y + radius;
            const s1_max_z = p1_z + radius;
            const AABBs = target.AABBs;
            const min_index = index * 3 * 2;
            const max_index = index * 3 * 2 + 3;
            AABBs[min_index] = min(s0_min_x, s1_min_x);
            AABBs[min_index + 1] = min(s0_min_y, s1_min_y);
            AABBs[min_index + 2] = min(s0_min_z, s1_min_z);
            AABBs[max_index] = max(s0_max_x, s1_max_x);
            AABBs[max_index + 1] = max(s0_max_y, s1_max_y);
            AABBs[max_index + 2] = max(s0_max_z, s1_max_z);
        }
    };
}
function create_plane_rigid_bodies(n_bodies) {
    const constants = new Float32Array(n_bodies);
    const normals = new Float32Array(n_bodies * 3);
    const tolerances = new Float32Array(n_bodies);
    return { constants, normals, tolerances, length: n_bodies };
}
function plane_rigid_body_accessor(target, index) {
    return {
        get constant() { return target.constants[index]; },
        set constant(constant) { target.constants[index] = constant; },
        get tolerance() { return target.tolerances[index]; },
        set tolerance(tol) { target.tolerances[index] = tol; },
        get_normal() {
            return get_vec3_array_gen(target.normals, index);
        },
        set_normal(x, y, z) {
            set_vec3_gen(target.normals, index, x, y, z);
        }
    };
}
function get_AABB(AABBs, index) {
    return [
        get_vec3_array_gen(AABBs, index * 2),
        get_vec3_array_gen(AABBs, (index * 2) + 1)
    ];
}

const abs = Math.abs;
class SpatialHash {
    constructor(params) {
        this.table_size = params.table_size;
        this.grid_cell_size = params.grid_cell_size;
        this.position_offset = params.position_offset;
        this._table = new Array(this.table_size);
        for (let i = 0, len = this.table_size; i < len; ++i) {
            this._table[i] = { timestamp: 0, entries: [] };
        }
    }
    hash(x, y, z) {
        const offset = this.position_offset;
        return ((abs(x + offset) * 73856093) ^ (abs(y + offset) * 19349669) ^ (abs(z + offset) * 83492791)) % this.table_size;
    }
    set(x, y, z, timestamp, content) {
        const slot = this._table[this.hash(x, y, z)];
        if (slot.timestamp < timestamp) {
            slot.timestamp = timestamp;
            slot.entries.length = 0;
        }
        slot.entries.push(content);
    }
    get(x, y, z, timestamp) {
        const slot = this._table[this.hash(x, y, z)];
        if (slot.timestamp === timestamp) {
            return slot.entries;
        }
        else {
            return undefined;
        }
    }
}

const floor = Math.floor;
const sqrt$1 = Math.sqrt;
const min$1 = Math.min;
const max$1 = Math.max;
class CollisionDetector {
    constructor(broadphase_params) {
        this._timestamp = 0;
        this._allocator = new ContactConstraintAllocator();
        this._broadphase = new SpatialHash(broadphase_params);
    }
    release_collision_constraints() {
        this._allocator.release_all();
    }
    generate_collision_constraints(particles, objects) {
        const position = particles.position;
        const mass = particles.mass;
        const broadphase = this._broadphase;
        const inv_grid_cell_size = 1 / broadphase.grid_cell_size;
        const timestamp = this._timestamp;
        // store particles in the hash table.
        for (let i = 0, len = particles.length; i < len; ++i) {
            if (mass[i] === 0.0)
                continue;
            const x = floor(position[i * 3] * inv_grid_cell_size);
            const y = floor(position[i * 3 + 1] * inv_grid_cell_size);
            const z = floor(position[i * 3 + 2] * inv_grid_cell_size);
            broadphase.set(x, y, z, timestamp, i);
        }
        const constraints = [];
        const allocator = this._allocator;
        const sphere_bodies = objects.spheres;
        if (sphere_bodies) {
            dispatch_collision_handler(sphere_bodies, particles, broadphase, timestamp, (particle_id, object_id, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len) => {
                const c = create_sphere_contact_constraint(particle_id, object_id, sphere_bodies, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator);
                c && constraints.push(c);
            });
        }
        const box_bodies = objects.boxes;
        if (box_bodies) {
            dispatch_collision_handler(box_bodies, particles, broadphase, timestamp, (particle_id, object_id, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len) => {
                const c = create_box_contact_constraint(particle_id, object_id, box_bodies, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator);
                c && constraints.push(c);
            });
        }
        const capsule_bodies = objects.capsules;
        if (capsule_bodies) {
            dispatch_collision_handler(capsule_bodies, particles, broadphase, timestamp, (particle_id, object_id, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len) => {
                const c = create_capsule_contact_constraint(particle_id, object_id, capsule_bodies, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator);
                c && constraints.push(c);
            });
        }
        const plane_bodies = objects.planes;
        if (plane_bodies) {
            const old_pos = particles.old_position;
            for (let i = 0, len = particles.length; i < len; ++i) {
                if (mass[i] === 0.0)
                    continue;
                // FIXME: DRY
                const X = i * 3, Y = X + 1, Z = X + 2;
                const p_x = position[X];
                const p_y = position[Y];
                const p_z = position[Z];
                const op_x = old_pos[X];
                const op_y = old_pos[Y];
                const op_z = old_pos[Z];
                const d_x = p_x - op_x;
                const d_y = p_y - op_y;
                const d_z = p_z - op_z;
                const sq_len = (d_x * d_x) + (d_y * d_y) + (d_z * d_z);
                const delta_len = sqrt$1(sq_len);
                if (delta_len < EPSILON)
                    continue;
                const inv_len = 1 / delta_len;
                const n_x = d_x * inv_len;
                const n_y = d_y * inv_len;
                const n_z = d_z * inv_len;
                generate_plane_contact_constraints(constraints, i, plane_bodies, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator);
            }
        }
        this._timestamp = timestamp + 1;
        return constraints;
    }
}
function dispatch_collision_handler(objects, particles, broadphase, timestamp, handler) {
    const position = particles.position;
    const old_pos = particles.old_position;
    const AABBs = objects.AABBs;
    const inv_grid_cell_size = 1 / broadphase.grid_cell_size;
    for (let object_id = 0, len = objects.length; object_id < len; ++object_id) {
        const min_index = object_id * 3 * 2;
        const max_index = object_id * 3 * 2 + 3;
        const min_x = AABBs[min_index];
        const min_y = AABBs[min_index + 1];
        const min_z = AABBs[min_index + 2];
        const max_x = AABBs[max_index];
        const max_y = AABBs[max_index + 1];
        const max_z = AABBs[max_index + 2];
        const grid_min_x = floor(min_x * inv_grid_cell_size);
        const grid_min_y = floor(min_y * inv_grid_cell_size);
        const grid_min_z = floor(min_z * inv_grid_cell_size);
        const grid_max_x = floor(max_x * inv_grid_cell_size);
        const grid_max_y = floor(max_y * inv_grid_cell_size);
        const grid_max_z = floor(max_z * inv_grid_cell_size);
        for (let grid_x = grid_min_x; grid_x <= grid_max_x; ++grid_x) {
            for (let grid_y = grid_min_y; grid_y <= grid_max_y; ++grid_y) {
                for (let grid_z = grid_min_z; grid_z <= grid_max_z; ++grid_z) {
                    const p_list = broadphase.get(grid_x, grid_y, grid_z, timestamp);
                    if (!p_list)
                        continue;
                    for (let p = 0, len = p_list.length; p < len; ++p) {
                        const particle_id = p_list[p];
                        const X = particle_id * 3, Y = X + 1, Z = X + 2;
                        const p_x = position[X];
                        const p_y = position[Y];
                        const p_z = position[Z];
                        if (AABB_contain_point(min_x, min_y, min_z, max_x, max_y, max_z, p_x, p_y, p_z)) {
                            const op_x = old_pos[X];
                            const op_y = old_pos[Y];
                            const op_z = old_pos[Z];
                            const d_x = p_x - op_x;
                            const d_y = p_y - op_y;
                            const d_z = p_z - op_z;
                            const sq_len = (d_x * d_x) + (d_y * d_y) + (d_z * d_z);
                            const delta_len = sqrt$1(sq_len);
                            if (delta_len < EPSILON)
                                continue;
                            const inv_len = 1 / delta_len;
                            const n_x = d_x * inv_len;
                            const n_y = d_y * inv_len;
                            const n_z = d_z * inv_len;
                            handler(particle_id, object_id, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len);
                        }
                    }
                }
            }
        }
    }
}
function AABB_contain_point(min_x, min_y, min_z, max_x, max_y, max_z, p_x, p_y, p_z) {
    return (p_x >= min_x && max_x >= p_x
        && p_y >= min_y && max_y >= p_y
        && p_z >= min_z && max_z >= p_z);
}
const create_sphere_contact_constraint = (() => {
    const cp = [];
    const cn = [];
    return (particle_id, object_id, sphere_bodies, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator) => {
        const positions = sphere_bodies.positions;
        const sp_x = positions[object_id * 3];
        const sp_y = positions[object_id * 3 + 1];
        const sp_z = positions[object_id * 3 + 2];
        const tolerance = sphere_bodies.tolerances[object_id];
        const radius = sphere_bodies.radiuses[object_id] + tolerance;
        const sq_radius = radius * radius;
        const t_min = ray_test_sphere(op_x, op_y, op_z, n_x, n_y, n_z, delta_len, sp_x, sp_y, sp_z, sq_radius);
        if (t_min !== t_min)
            return undefined;
        // continuous collision
        if (t_min >= 0.0) {
            const cp_x = op_x + t_min * n_x;
            const cp_y = op_y + t_min * n_y;
            const cp_z = op_z + t_min * n_z;
            const spcp_x = cp_x - sp_x;
            const spcp_y = cp_y - sp_y;
            const spcp_z = cp_z - sp_z;
            const inv_spcp_len = 1 / radius;
            const cn_x = spcp_x * inv_spcp_len;
            const cn_y = spcp_y * inv_spcp_len;
            const cn_z = spcp_z * inv_spcp_len;
            // TODO: What is the correct gradient function of continuous collision?
            return create_contact_constraint(particle_id, cn_x, cn_y, cn_z, cp_x, cp_y, cp_z, cn_x, cn_y, cn_z, allocator);
        }
        else {
            static_collision_sphere_contact_info(p_x, p_y, p_z, sp_x, sp_y, sp_z, radius, cp, cn);
            return create_contact_constraint(particle_id, cn[0], cn[1], cn[2], cp[0], cp[1], cp[2], cn[0], cn[1], cn[2], allocator);
        }
    };
})();
function create_box_contact_constraint(particle_id, object_id, box_bodies, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator) {
    const positions = box_bodies.positions;
    const X = object_id * 3, Y = X + 1, Z = X + 2;
    const bp_x = positions[X];
    const bp_y = positions[Y];
    const bp_z = positions[Z];
    const tmp_x = op_x - bp_x;
    const tmp_y = op_y - bp_y;
    const tmp_z = op_z - bp_z;
    const inv_mats = box_bodies.inv_basis_mats;
    const idx = object_id * 9;
    const lop_x = inv_mats[idx + 0] * tmp_x + inv_mats[idx + 3] * tmp_y + inv_mats[idx + 6] * tmp_z;
    const lop_y = inv_mats[idx + 1] * tmp_x + inv_mats[idx + 4] * tmp_y + inv_mats[idx + 7] * tmp_z;
    const lop_z = inv_mats[idx + 2] * tmp_x + inv_mats[idx + 5] * tmp_y + inv_mats[idx + 8] * tmp_z;
    const ln_x = inv_mats[idx + 0] * n_x + inv_mats[idx + 3] * n_y + inv_mats[idx + 6] * n_z;
    const ln_y = inv_mats[idx + 1] * n_x + inv_mats[idx + 4] * n_y + inv_mats[idx + 7] * n_z;
    const ln_z = inv_mats[idx + 2] * n_x + inv_mats[idx + 5] * n_y + inv_mats[idx + 8] * n_z;
    const inv_ln_x = 1 / ln_x;
    const inv_ln_y = 1 / ln_y;
    const inv_ln_z = 1 / ln_z;
    const half_widths = box_bodies.half_widths;
    const half_heights = box_bodies.half_heights;
    const half_depths = box_bodies.half_depths;
    const tolerance = box_bodies.tolerances[object_id];
    const box_x_min = -half_widths[object_id] - tolerance;
    const box_x_max = half_widths[object_id] + tolerance;
    const box_y_min = -half_heights[object_id] - tolerance;
    const box_y_max = half_heights[object_id] + tolerance;
    const box_z_min = -half_depths[object_id] - tolerance;
    const box_z_max = half_depths[object_id] + tolerance;
    let c_plane = inv_ln_x >= 0.0 ? 0 : 3;
    let t_min = 0, t_max = 0;
    if (inv_ln_x >= 0.0) {
        t_min = (box_x_min - lop_x) * inv_ln_x;
        t_max = (box_x_max - lop_x) * inv_ln_x;
    }
    else {
        t_min = (box_x_max - lop_x) * inv_ln_x;
        t_max = (box_x_min - lop_x) * inv_ln_x;
    }
    let ty_min = 0, ty_max = 0;
    if (inv_ln_y >= 0.0) {
        ty_min = (box_y_min - lop_y) * inv_ln_y;
        ty_max = (box_y_max - lop_y) * inv_ln_y;
    }
    else {
        ty_min = (box_y_max - lop_y) * inv_ln_y;
        ty_max = (box_y_min - lop_y) * inv_ln_y;
    }
    if ((t_min > ty_max) || (ty_min > t_max))
        return undefined;
    if (ty_min > t_min || t_min !== t_min) {
        t_min = ty_min;
        c_plane = inv_ln_y >= 0.0 ? 1 : 4;
    }
    if (ty_max < t_max || t_max !== t_max)
        t_max = ty_max;
    let tz_min = 0, tz_max = 0;
    if (inv_ln_z >= 0.0) {
        tz_min = (box_z_min - lop_z) * inv_ln_z;
        tz_max = (box_z_max - lop_z) * inv_ln_z;
    }
    else {
        tz_min = (box_z_max - lop_z) * inv_ln_z;
        tz_max = (box_z_min - lop_z) * inv_ln_z;
    }
    if ((t_min > tz_max) || (tz_min > t_max))
        return undefined;
    if (tz_min > t_min || t_min !== t_min) {
        t_min = tz_min;
        c_plane = inv_ln_z >= 0.0 ? 2 : 5;
    }
    if (tz_max < t_max || t_max !== t_max)
        t_max = tz_max;
    if (t_max < 0.0 || t_min > delta_len)
        return undefined;
    if (t_min < 0.0 && t_max <= delta_len)
        return undefined;
    // continuous collision
    if (t_min >= 0.0) {
        const cp_x = op_x + t_min * n_x;
        const cp_y = op_y + t_min * n_y;
        const cp_z = op_z + t_min * n_z;
        const sign = c_plane < 3 ? 1 : -1;
        const axis = c_plane % 3;
        const cn_x = sign * inv_mats[idx + axis];
        const cn_y = sign * inv_mats[idx + axis + 3];
        const cn_z = sign * inv_mats[idx + axis + 6];
        // TODO: What is the correct gradient function of continuous collision?
        return create_contact_constraint(particle_id, cn_x, cn_y, cn_z, cp_x, cp_y, cp_z, cn_x, cn_y, cn_z, allocator);
    }
    else {
        const tmp_x = p_x - bp_x;
        const tmp_y = p_y - bp_y;
        const tmp_z = p_z - bp_z;
        const lp_x = inv_mats[idx + 0] * tmp_x + inv_mats[idx + 3] * tmp_y + inv_mats[idx + 6] * tmp_z;
        const lp_y = inv_mats[idx + 1] * tmp_x + inv_mats[idx + 4] * tmp_y + inv_mats[idx + 7] * tmp_z;
        const lp_z = inv_mats[idx + 2] * tmp_x + inv_mats[idx + 5] * tmp_y + inv_mats[idx + 8] * tmp_z;
        const min_x = lp_x >= 0.0 ? box_x_max - lp_x : lp_x - box_x_min;
        const min_y = lp_y >= 0.0 ? box_y_max - lp_y : lp_y - box_y_min;
        const min_z = lp_z >= 0.0 ? box_z_max - lp_z : lp_z - box_z_min;
        const t_min = min$1(min_x, min_y, min_z);
        if (t_min === min_x)
            c_plane = lp_x >= 0.0 ? 0 : 3;
        if (t_min === min_y)
            c_plane = lp_y >= 0.0 ? 1 : 4;
        if (t_min === min_z)
            c_plane = lp_z >= 0.0 ? 2 : 5;
        const sign = c_plane < 3 ? 1 : -1;
        const axis = c_plane % 3;
        const cn_x = sign * inv_mats[idx + axis];
        const cn_y = sign * inv_mats[idx + axis + 3];
        const cn_z = sign * inv_mats[idx + axis + 6];
        const cp_x = p_x + t_min * cn_x;
        const cp_y = p_y + t_min * cn_y;
        const cp_z = p_z + t_min * cn_z;
        return create_contact_constraint(particle_id, cn_x, cn_y, cn_z, cp_x, cp_y, cp_z, cn_x, cn_y, cn_z, allocator);
    }
}
const create_capsule_contact_constraint = (() => {
    const cp = [];
    const cn = [];
    return (particle_id, object_id, capsule_bodies, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator) => {
        const half_length = capsule_bodies.half_lengths[object_id];
        const X = object_id * 3, Y = X + 1, Z = X + 2;
        const positions = capsule_bodies.positions;
        const cap_x = positions[X];
        const cap_y = positions[Y];
        const cap_z = positions[Z];
        const directions = capsule_bodies.directions;
        const dir_x = directions[X];
        const dir_y = directions[Y];
        const dir_z = directions[Z];
        const sp1_x = cap_x + dir_x * half_length;
        const sp1_y = cap_y + dir_y * half_length;
        const sp1_z = cap_z + dir_z * half_length;
        const sp2_x = cap_x - dir_x * half_length;
        const sp2_y = cap_y - dir_y * half_length;
        const sp2_z = cap_z - dir_z * half_length;
        const tolerance = capsule_bodies.tolerances[object_id];
        const radius = capsule_bodies.radiuses[object_id] + tolerance;
        const sq_radius = radius * radius;
        // test sphere 1
        if (test_capsule_sphere(p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, sp1_x, sp1_y, sp1_z, sp2_x, sp2_y, sp2_z, radius, sq_radius, cp, cn)) {
            return create_contact_constraint(particle_id, cn[0], cn[1], cn[2], cp[0], cp[1], cp[2], cn[0], cn[1], cn[2], allocator);
        }
        // test sphere 2
        if (test_capsule_sphere(p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, sp2_x, sp2_y, sp2_z, sp1_x, sp1_y, sp1_z, radius, sq_radius, cp, cn)) {
            return create_contact_constraint(particle_id, cn[0], cn[1], cn[2], cp[0], cp[1], cp[2], cn[0], cn[1], cn[2], allocator);
        }
        // test cilinder
        const ct_min = ray_test_inf_cilinder(op_x, op_y, op_z, n_x, n_y, n_z, delta_len, cap_x, cap_y, cap_z, dir_x, dir_y, dir_z, sq_radius);
        if (ct_min !== ct_min)
            return undefined;
        // continuous collision
        if (ct_min >= 0.0) {
            const cp_x = op_x + ct_min * n_x;
            const cp_y = op_y + ct_min * n_y;
            const cp_z = op_z + ct_min * n_z;
            if (three_vectors_dot(sp2_x, sp2_y, sp2_z, sp1_x, sp1_y, sp1_z, cp_x, cp_y, cp_z) > 0.0
                && three_vectors_dot(sp1_x, sp1_y, sp1_z, sp2_x, sp2_y, sp2_z, cp_x, cp_y, cp_z) > 0.0) {
                const dot = (cp_x - sp2_x) * dir_x + (cp_y - sp2_y) * dir_y + (cp_z - sp2_z) * dir_z;
                const center_x = sp2_x + dot * dir_x;
                const center_y = sp2_y + dot * dir_y;
                const center_z = sp2_z + dot * dir_z;
                const v_x = cp_x - center_x;
                const v_y = cp_y - center_y;
                const v_z = cp_z - center_z;
                const inv_v_len = 1 / radius;
                const cn_x = v_x * inv_v_len;
                const cn_y = v_y * inv_v_len;
                const cn_z = v_z * inv_v_len;
                // TODO: What is the correct gradient function of continuous collision?
                return create_contact_constraint(particle_id, cn_x, cn_y, cn_z, cp_x, cp_y, cp_z, cn_x, cn_y, cn_z, allocator);
            }
            else {
                return undefined;
            }
        }
        else {
            if (three_vectors_dot(sp2_x, sp2_y, sp2_z, sp1_x, sp1_y, sp1_z, p_x, p_y, p_z) > 0.0
                && three_vectors_dot(sp1_x, sp1_y, sp1_z, sp2_x, sp2_y, sp2_z, p_x, p_y, p_z) > 0.0) {
                const dot = (p_x - sp2_x) * dir_x + (p_y - sp2_y) * dir_y + (p_z - sp2_z) * dir_z;
                const center_x = sp2_x + dot * dir_x;
                const center_y = sp2_y + dot * dir_y;
                const center_z = sp2_z + dot * dir_z;
                const v_x = p_x - center_x;
                const v_y = p_y - center_y;
                const v_z = p_z - center_z;
                const v_len = sqrt$1(v_x * v_x + v_y * v_y + v_z * v_z);
                const inv_v_len = 1 / v_len;
                const cn_x = v_x * inv_v_len;
                const cn_y = v_y * inv_v_len;
                const cn_z = v_z * inv_v_len;
                const cp_x = radius * cn_x + center_x;
                const cp_y = radius * cn_y + center_y;
                const cp_z = radius * cn_z + center_z;
                return create_contact_constraint(particle_id, cn_x, cn_y, cn_z, cp_x, cp_y, cp_z, cn_x, cn_y, cn_z, allocator);
            }
            else {
                return undefined;
            }
        }
    };
})();
function generate_plane_contact_constraints(constraints, particle_id, plane_bodies, p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator) {
    const constants = plane_bodies.constants;
    const normals = plane_bodies.normals;
    for (let i = 0, len = plane_bodies.length; i < len; ++i) {
        const X = i * 3, Y = X + 1, Z = X + 2;
        const tolerance = plane_bodies.tolerances[i];
        const constant = constants[i] - tolerance;
        const pn_x = normals[X];
        const pn_y = normals[Y];
        const pn_z = normals[Z];
        const denom = pn_x * n_x + pn_y * n_y + pn_z * n_z;
        const dist = pn_x * op_x + pn_y * op_y + pn_z * op_z + constant;
        let t_min = 0;
        if (denom === 0) {
            if (dist === 0)
                t_min = 0;
            else
                continue;
        }
        else {
            t_min = -dist / denom;
        }
        if (t_min > delta_len)
            continue;
        // continuous collision
        if (t_min >= 0.0) {
            const cp_x = op_x + t_min * n_x;
            const cp_y = op_y + t_min * n_y;
            const cp_z = op_z + t_min * n_z;
            // TODO: What is the correct gradient function of continuous collision?
            constraints.push(create_contact_constraint(particle_id, pn_x, pn_y, pn_z, cp_x, cp_y, cp_z, pn_x, pn_y, pn_z, allocator));
        }
        else {
            const cp_x = pn_x * -dist + p_x;
            const cp_y = pn_y * -dist + p_y;
            const cp_z = pn_z * -dist + p_z;
            constraints.push(create_contact_constraint(particle_id, pn_x, pn_y, pn_z, cp_x, cp_y, cp_z, pn_x, pn_y, pn_z, allocator));
        }
    }
}
function static_collision_sphere_contact_info(p_x, p_y, p_z, sp_x, sp_y, sp_z, radius, out_cp, out_cn) {
    const spp_x = p_x - sp_x;
    const spp_y = p_y - sp_y;
    const spp_z = p_z - sp_z;
    const spp_len = sqrt$1((spp_x * spp_x) + (spp_y * spp_y) + (spp_z * spp_z));
    const inv_spp_len = 1 / spp_len;
    const cn_x = spp_x * inv_spp_len;
    const cn_y = spp_y * inv_spp_len;
    const cn_z = spp_z * inv_spp_len;
    const cp_x = sp_x + cn_x * radius;
    const cp_y = sp_y + cn_y * radius;
    const cp_z = sp_z + cn_z * radius;
    out_cp[0] = cp_x, out_cp[1] = cp_y, out_cp[2] = cp_z;
    out_cn[0] = cn_x, out_cn[1] = cn_y, out_cn[2] = cn_z;
}
function ray_test_sphere(orig_x, orig_y, orig_z, dir_x, dir_y, dir_z, ray_length, sphere_x, sphere_y, sphere_z, sq_radius) {
    const lo_s_x = sphere_x - orig_x;
    const lo_s_y = sphere_y - orig_y;
    const lo_s_z = sphere_z - orig_z;
    const B = (dir_x * lo_s_x) + (dir_y * lo_s_y) + (dir_z * lo_s_z);
    const C = (lo_s_x * lo_s_x) + (lo_s_y * lo_s_y) + (lo_s_z * lo_s_z) - sq_radius;
    const sq = B * B - C;
    if (sq < 0.0)
        return NaN;
    const s = sqrt$1(sq);
    const a1 = B - s, a2 = B + s;
    const t_min = min$1(a1, a2);
    const t_max = max$1(a1, a2);
    if (t_max < 0.0 || t_min > ray_length)
        return NaN;
    if (t_min < 0.0 && t_max <= ray_length)
        return NaN;
    return t_min;
}
function ray_test_inf_cilinder(orig_x, orig_y, orig_z, dir_x, dir_y, dir_z, ray_length, cp_x, cp_y, cp_z, cdir_x, cdir_y, cdir_z, sq_radius) {
    const p_x = cp_x - orig_x;
    const p_y = cp_y - orig_y;
    const p_z = cp_z - orig_z;
    const dot_sd = cdir_x * dir_x + cdir_y * dir_y + cdir_z * dir_z;
    const dot_pd = p_x * dir_x + p_y * dir_y + p_z * dir_z;
    const dot_ps = p_x * cdir_x + p_y * cdir_y + p_z * cdir_z;
    const dot_pp = p_x * p_x + p_y * p_y + p_z * p_z;
    const A = 1 - dot_sd * dot_sd;
    const B = dot_pd - dot_ps * dot_sd;
    const C = dot_pp - dot_ps * dot_ps - sq_radius;
    if (A === 0.0)
        return NaN;
    const sq = B * B - A * C;
    if (sq < 0.0)
        return NaN;
    const s = sqrt$1(sq);
    const inv_A = 1 / A;
    const a1 = (B - s) * inv_A;
    const a2 = (B + s) * inv_A;
    const t_min = min$1(a1, a2);
    const t_max = max$1(a1, a2);
    if (t_max < 0.0 || t_min > ray_length)
        return NaN;
    if (t_min < 0.0 && t_max <= ray_length)
        return NaN;
    return t_min;
}
function test_capsule_sphere(p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_length, sp_x, sp_y, sp_z, sp2_x, sp2_y, sp2_z, radius, sq_radius, out_cp, out_cn) {
    const t_min = ray_test_sphere(op_x, op_y, op_z, n_x, n_y, n_z, delta_length, sp_x, sp_y, sp_z, sq_radius);
    if (t_min === t_min) {
        // continuous collision
        if (t_min >= 0.0) {
            const cp_x = op_x + t_min * n_x;
            const cp_y = op_y + t_min * n_y;
            const cp_z = op_z + t_min * n_z;
            if (three_vectors_dot(sp2_x, sp2_y, sp2_z, sp_x, sp_y, sp_z, cp_x, cp_y, cp_z) <= 0.0) {
                const spcp_x = cp_x - sp_x;
                const spcp_y = cp_y - sp_y;
                const spcp_z = cp_z - sp_z;
                const inv_spcp_len = 1 / radius;
                const cn_x = spcp_x * inv_spcp_len;
                const cn_y = spcp_y * inv_spcp_len;
                const cn_z = spcp_z * inv_spcp_len;
                out_cp[0] = cp_x, out_cp[1] = cp_y, out_cp[2] = cp_z;
                out_cn[0] = cn_x, out_cn[1] = cn_y, out_cn[2] = cn_z;
                return true;
            }
        }
        else {
            if (three_vectors_dot(sp2_x, sp2_y, sp2_z, sp_x, sp_y, sp_z, p_x, p_y, p_z) <= 0.0) {
                static_collision_sphere_contact_info(p_x, p_y, p_z, sp_x, sp_y, sp_z, radius, out_cp, out_cn);
                return true;
            }
        }
    }
    return false;
}
function three_vectors_dot(x1, y1, z1, x2, y2, z2, x3, y3, z3) {
    return (x1 - x2) * (x3 - x2) + (y1 - y2) * (y3 - y2) + (z1 - z2) * (z3 - z2);
}

const sqrt$2 = Math.sqrt;
const min$2 = Math.min;
const step = (() => {
    let prev_coll_consts = [];
    return (model, objects, detector, config) => {
        const step_iter = config.step_iter;
        for (let i = 0; i < step_iter; ++i) {
            prev_coll_consts = internal_step(model, objects, detector, prev_coll_consts, config);
        }
    };
})();
function internal_step(model, objects, detector, prev_collision_constraints, config) {
    const particles = model.particles;
    const n_particle = particles.length;
    const position = particles.position;
    const old_position = particles.old_position;
    const velocity = particles.velocity;
    const mass = particles.mass;
    const inv_mass = particles.inv_mass;
    const time_step = config.time_step / config.step_iter;
    // integrate velocity
    const gravity = config.gravity;
    integrate_velocity(velocity, mass, n_particle, gravity, time_step);
    // velocity damping
    const r_vec = particles.extra_vec;
    const velocity_damp_factor = config.velocity_damp_factor;
    velocity_damping(velocity, position, mass, r_vec, n_particle, velocity_damp_factor);
    // position predictions
    position_predictions(position, old_position, mass, velocity, n_particle, time_step);
    // pre-stabilization
    if (prev_collision_constraints.length) {
        const delta_positions = particles.extra_vec;
        const n_collisions = particles.extra_uint8;
        for (let i = 0, len = config.pre_stabilize_iter; i < len; ++i) {
            pre_stabilization(position, old_position, prev_collision_constraints, inv_mass, n_particle, delta_positions, n_collisions);
        }
    }
    // generate collision constraints
    detector.release_collision_constraints();
    const collision_constraints = detector.generate_collision_constraints(particles, objects);
    // project constraints
    const contact_normals = particles.extra_vec;
    const depths = particles.extra_float;
    const static_friction_coeff = config.static_friction_coeff;
    const kinetic_friction_coeff = config.kinetic_friction_coeff;
    for (let i = 0, len = config.max_proj_iter; i < len; ++i) {
        project_constraint(model.constraints, collision_constraints, config.stiffnesses, position, inv_mass, contact_normals, depths);
        apply_friction(position, old_position, n_particle, contact_normals, depths, static_friction_coeff, kinetic_friction_coeff);
    }
    // update velocity
    update_velocity(velocity, position, old_position, mass, n_particle, time_step);
    return collision_constraints;
}
function integrate_velocity(velocity, mass, n_particle, gravity, time_step) {
    const gravity_speed = gravity * time_step;
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            const Y = i * 3 + 1;
            velocity[Y] += gravity_speed;
        }
    }
}
const velocity_damping = (() => {
    const l_vec = new three.Vector3();
    const r_mat = new three.Matrix3();
    const r_mat2 = new three.Matrix3();
    const i_mat = new three.Matrix3();
    const inv_mat = new three.Matrix3();
    return (velocities, positions, masses, r_vec, n_particle, k_damping) => {
        let p_cm_x = 0;
        let p_cm_y = 0;
        let p_cm_z = 0;
        let v_cm_x = 0;
        let v_cm_y = 0;
        let v_cm_z = 0;
        let sum_mass = 0;
        for (let i = 0; i < n_particle; ++i) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            const mass = masses[i];
            p_cm_x += positions[X] * mass;
            p_cm_y += positions[Y] * mass;
            p_cm_z += positions[Z] * mass;
            v_cm_x += velocities[X] * mass;
            v_cm_y += velocities[Y] * mass;
            v_cm_z += velocities[Z] * mass;
            sum_mass += mass;
        }
        p_cm_x /= sum_mass;
        p_cm_y /= sum_mass;
        p_cm_z /= sum_mass;
        v_cm_x /= sum_mass;
        v_cm_y /= sum_mass;
        v_cm_z /= sum_mass;
        l_vec.set(0, 0, 0);
        i_mat.identity();
        const r_elm = r_mat.elements;
        const i_elm = i_mat.elements;
        for (let i = 0; i < n_particle; ++i) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            const r_x = r_vec[X] = positions[X] - p_cm_x;
            const r_y = r_vec[Y] = positions[Y] - p_cm_y;
            const r_z = r_vec[Z] = positions[Z] - p_cm_z;
            const mass = masses[i];
            const mv_x = velocities[X] * mass;
            const mv_y = velocities[Y] * mass;
            const mv_z = velocities[Z] * mass;
            l_vec.x += r_y * mv_z - r_z * mv_y;
            l_vec.y += r_z * mv_x - r_x * mv_z;
            l_vec.z += r_x * mv_y - r_y * mv_x;
            r_mat.set(0, -r_z, r_y, r_z, 0, -r_x, -r_y, r_x, 0);
            r_mat2.copy(r_mat).transpose();
            r_mat.multiply(r_mat2).multiplyScalar(mass);
            for (let k = 0; k < 9; ++k)
                i_elm[k] += r_elm[k];
        }
        inv_mat.getInverse(i_mat);
        l_vec.applyMatrix3(inv_mat);
        for (let i = 0; i < n_particle; ++i) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            const r_x = r_vec[X];
            const r_y = r_vec[Y];
            const r_z = r_vec[Z];
            const dv_x = v_cm_x + (l_vec.y * r_z + l_vec.z * r_y) - velocities[X];
            const dv_y = v_cm_y + (l_vec.z * r_x + l_vec.x * r_z) - velocities[Y];
            const dv_z = v_cm_z + (l_vec.x * r_y + l_vec.y * r_x) - velocities[Z];
            velocities[X] += k_damping * dv_x;
            velocities[Y] += k_damping * dv_y;
            velocities[Z] += k_damping * dv_z;
        }
    };
})();
function position_predictions(position, old_position, mass, velocity, n_particle, time_step) {
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            old_position[X] = position[X];
            old_position[Y] = position[Y];
            old_position[Z] = position[Z];
            position[X] += velocity[X] * time_step;
            position[Y] += velocity[Y] * time_step;
            position[Z] += velocity[Z] * time_step;
        }
    }
}
function pre_stabilization(position, old_position, prev_collision_constraints, inv_mass, n_particle, delta_positions, n_collisions) {
    delta_positions.fill(0);
    n_collisions.fill(0);
    for (let j = 0, len = prev_collision_constraints.length; j < len; ++j) {
        prestabilize_contact_constraint(prev_collision_constraints[j], old_position, delta_positions, n_collisions, inv_mass);
    }
    for (let j = 0; j < n_particle; ++j) {
        const X = j * 3, Y = X + 1, Z = X + 2;
        const n = n_collisions[j];
        if (n === 0)
            continue;
        const inv_n = 1 / n;
        old_position[X] += delta_positions[X] * inv_n;
        old_position[Y] += delta_positions[Y] * inv_n;
        old_position[Z] += delta_positions[Z] * inv_n;
        position[X] += delta_positions[X] * inv_n;
        position[Y] += delta_positions[Y] * inv_n;
        position[Z] += delta_positions[Z] * inv_n;
    }
}
function project_constraint(constraints, collision_constraints, stiffnesses, position, inv_mass, contact_normals, depths) {
    contact_normals.fill(0);
    depths.fill(0);
    for (let j = 0, len = constraints.length; j < len; ++j) {
        constraints[j].solver(position, inv_mass, stiffnesses);
    }
    for (let j = 0, len = collision_constraints.length; j < len; ++j) {
        solve_contact_constraint(collision_constraints[j], position, inv_mass, contact_normals, depths);
    }
}
function apply_friction(position, old_position, n_particle, contact_normals, depths, static_friction_coeff, kinetic_friction_coeff) {
    for (let j = 0; j < n_particle; ++j) {
        if (depths[j] > 0) {
            normalize_vec3_gen(contact_normals, j);
            const X = j * 3, Y = X + 1, Z = X + 2;
            const cn_x = contact_normals[X];
            const cn_y = contact_normals[Y];
            const cn_z = contact_normals[Z];
            const d_x = position[X] - old_position[X];
            const d_y = position[Y] - old_position[Y];
            const d_z = position[Z] - old_position[Z];
            const dot = d_x * cn_x + d_y * cn_y + d_z * cn_z;
            const dis_x = -d_x + (cn_x * dot);
            const dis_y = -d_y + (cn_y * dot);
            const dis_z = -d_z + (cn_z * dot);
            const dis_len = sqrt$2((dis_x * dis_x) + (dis_y * dis_y) + (dis_z * dis_z));
            const depth = depths[j];
            if (static_friction_coeff * depth > dis_len) {
                position[X] += dis_x;
                position[Y] += dis_y;
                position[Z] += dis_z;
            }
            else {
                const coeff = min$2((kinetic_friction_coeff * depth) / dis_len, 1.0);
                position[X] += dis_x * coeff;
                position[Y] += dis_y * coeff;
                position[Z] += dis_z * coeff;
            }
        }
    }
}
function update_velocity(velocity, position, old_position, mass, n_particle, time_step) {
    const inv_time_step = 1.0 / time_step;
    for (let i = 0; i < n_particle; ++i) {
        if (mass[i] !== 0.0) {
            const X = i * 3, Y = X + 1, Z = X + 2;
            let v_x = position[X] - old_position[X];
            let v_y = position[Y] - old_position[Y];
            let v_z = position[Z] - old_position[Z];
            v_x *= inv_time_step;
            v_y *= inv_time_step;
            v_z *= inv_time_step;
            velocity[X] = v_x;
            velocity[Y] = v_y;
            velocity[Z] = v_z;
        }
    }
}

function to_enumerable(target, accessor) {
    return new Proxy({}, {
        get(obj, key, receiver) {
            if (typeof key === 'string') {
                const index = parseInt(key, 10);
                if (Number.isInteger(index)) {
                    return accessor(target, index);
                }
                if (key === 'length') {
                    return target.length;
                }
            }
            if (key === Symbol.iterator) {
                return function* () {
                    for (let i = 0, len = target.length; i < len; ++i) {
                        yield accessor(target, i);
                    }
                };
            }
            return Reflect.get(obj, key, receiver);
        }
    });
}

// depends on typings/TypedArray.d.ts
function sub_typed_array(array, start, count) {
    return array.subarray(start, start + count);
}

function get_constructor(array) {
    return array.__proto__.constructor;
}

const MimeTypes = ['video/webm;codecs=vp9', 'video/mp4;codecs=avc1', 'video/webm;codecs=vp8', 'video/webm'];
const UrlParamName = 'capture';
class CanvasRecorder {
    constructor(canvas, record_time) {
        if (window.MediaRecorder !== undefined) {
            const support_type = MimeTypes.find(type => MediaRecorder.isTypeSupported(type));
            if (support_type) {
                this.mime_type = support_type;
                this.recorder = new MediaRecorder(canvas.captureStream(60), { mimeType: support_type });
                this.record_time = record_time;
                const ext = support_type.match(/video\/(\w+);?/i)[1];
                this.recorder.ondataavailable = e => download(e.data, `capture.${ext}`);
            }
        }
    }
    static createFromUrlParams(canvas, url) {
        const params = new URL(url).searchParams;
        const record_time = parseInt(params.get(UrlParamName) || '', 10);
        if (!isNaN(record_time)) {
            return new CanvasRecorder(canvas, record_time * 1000);
        }
        else {
            return undefined;
        }
    }
    start() {
        if (this.recorder) {
            setTimeout(() => {
                this.recorder.stop();
            }, this.record_time);
            this.recorder.start();
        }
    }
}
function download(blob, file_name) {
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.style.display = 'none';
    a.href = url;
    a.download = file_name;
    a.click();
    URL.revokeObjectURL(url);
}

const RigidBodyColor = 0x151cef;
const AABBLineColor = 0xff00ff;
function particle_helper(particles, dynamic_particle_color = [0, 1, 0], static_particle_color = [0, 0, 1]) {
    const geometry = new three.BufferGeometry();
    const pos_attr = new three.BufferAttribute(particles.position, 3);
    pos_attr.setDynamic(true);
    geometry.addAttribute('position', pos_attr);
    const colors = particles.mass
        .reduce((acc, m) => acc.concat(m !== 0.0 ? dynamic_particle_color : static_particle_color), []);
    geometry.addAttribute('color', new three.Float32BufferAttribute(colors, 3));
    const material = new three.PointsMaterial({ vertexColors: three.VertexColors, size: 0.3 });
    return new three.Points(geometry, material);
}
function rigid_body_helper(rigid_bodies, wireframe = true) {
    const render_objects = [];
    if (rigid_bodies.spheres) {
        for (const sphere of to_enumerable(rigid_bodies.spheres, sphere_rigid_body_accessor)) {
            render_objects.push(create_sphere_mesh(sphere, wireframe));
            wireframe && render_objects.push(create_AABB_line_segments(sphere));
        }
    }
    if (rigid_bodies.boxes) {
        for (const box of to_enumerable(rigid_bodies.boxes, box_rigid_body_accessor)) {
            render_objects.push(create_box_mesh(box, wireframe));
            wireframe && render_objects.push(create_AABB_line_segments(box));
        }
    }
    if (rigid_bodies.capsules) {
        for (const capsule of to_enumerable(rigid_bodies.capsules, capsule_rigid_body_accessor)) {
            render_objects.push(create_capsule_mesh(capsule, wireframe));
            wireframe && render_objects.push(create_AABB_line_segments(capsule));
        }
    }
    return render_objects;
}
function create_sphere_mesh(sphere, wireframe) {
    const sphere_geo = new three.SphereBufferGeometry(sphere.radius, 20, 20);
    const sphere_mesh = new three.Mesh(sphere_geo, new three.MeshStandardMaterial({
        color: RigidBodyColor, emissive: new three.Color(0x0f087c),
        roughness: 0.88, metalness: 0.9, wireframe
    }));
    sphere_mesh.userData.update = ((s, m) => () => {
        m.position.fromArray(s.get_position());
    })(sphere, sphere_mesh);
    sphere_mesh.userData.update();
    return sphere_mesh;
}
function create_box_mesh(box, wireframe) {
    const box_geo = new three.BoxBufferGeometry(box.half_width * 2, box.half_height * 2, box.half_depth * 2);
    const box_mesh = new three.Mesh(box_geo, new three.MeshStandardMaterial({
        color: RigidBodyColor, emissive: new three.Color(0x0f087c),
        roughness: 0.88, metalness: 0.9, wireframe
    }));
    box_mesh.userData.update = ((b, m) => () => {
        const x_axis = b.get_inv_basis_vector(0);
        const y_axis = b.get_inv_basis_vector(1);
        const z_axis = b.get_inv_basis_vector(2);
        const pos = b.get_position();
        m.matrix.set(x_axis[0], y_axis[0], z_axis[0], pos[0], x_axis[1], y_axis[1], z_axis[1], pos[1], x_axis[2], y_axis[2], z_axis[2], pos[2], 0, 0, 0, 1);
        m.matrix.decompose(m.position, m.quaternion, m.scale);
    })(box, box_mesh);
    box_mesh.userData.update();
    return box_mesh;
}
function create_capsule_mesh(capsule, wireframe) {
    const capsule_geo = create_capsule_geometry(capsule.radius, capsule.length, 16, 8);
    const capsule_mesh = new three.Mesh(capsule_geo, new three.MeshStandardMaterial({
        color: RigidBodyColor, emissive: new three.Color(0x0f087c),
        roughness: 0.88, metalness: 0.9, wireframe
    }));
    capsule_mesh.userData.update = ((c, m) => {
        const v_from = new three.Vector3();
        const v_to = new three.Vector3();
        const quat = new three.Quaternion();
        const v_pos = new three.Vector3();
        return () => {
            const pos = c.get_position();
            const dir = c.get_direction();
            v_from.set(0, 1, 0);
            v_to.fromArray(dir);
            quat.setFromUnitVectors(v_from, v_to);
            m.matrix.makeRotationFromQuaternion(quat);
            m.matrix.setPosition(v_pos.fromArray(pos));
            m.matrix.decompose(m.position, m.quaternion, m.scale);
        };
    })(capsule, capsule_mesh);
    capsule_mesh.userData.update();
    return capsule_mesh;
}
function create_AABB_line_segments(object) {
    const indices = new Uint16Array([0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7]);
    const positions = new Float32Array(8 * 3);
    const geometry = new three.BufferGeometry();
    geometry.setIndex(new three.BufferAttribute(indices, 1));
    geometry.addAttribute('position', new three.BufferAttribute(positions, 3));
    const line_segments = new three.LineSegments(geometry, new three.LineBasicMaterial({ color: AABBLineColor }));
    line_segments.matrixAutoUpdate = false;
    line_segments.userData.update = ((o, pos) => {
        const min = new three.Vector3();
        const max = new three.Vector3();
        return () => {
            const AABB = o.get_AABB();
            min.fromArray(AABB[0]);
            max.fromArray(AABB[1]);
            const array = pos.array;
            array[0] = max.x;
            array[1] = max.y;
            array[2] = max.z;
            array[3] = min.x;
            array[4] = max.y;
            array[5] = max.z;
            array[6] = min.x;
            array[7] = min.y;
            array[8] = max.z;
            array[9] = max.x;
            array[10] = min.y;
            array[11] = max.z;
            array[12] = max.x;
            array[13] = max.y;
            array[14] = min.z;
            array[15] = min.x;
            array[16] = max.y;
            array[17] = min.z;
            array[18] = min.x;
            array[19] = min.y;
            array[20] = min.z;
            array[21] = max.x;
            array[22] = min.y;
            array[23] = min.z;
            pos.needsUpdate = true;
        };
    })(object, geometry.getAttribute('position'));
    line_segments.userData.update();
    return line_segments;
}
function create_capsule_geometry(radius, cylinderHeight, segmentsRadius, segmentsHeight) {
    const geometry = new three.CylinderGeometry(radius, radius, cylinderHeight, segmentsRadius, segmentsHeight, true);
    const upperSphere = new three.Mesh(new three.SphereGeometry(radius, segmentsRadius, segmentsHeight, 0, Math.PI * 2, 0, Math.PI / 2));
    const lowerSphere = new three.Mesh(new three.SphereGeometry(radius, segmentsRadius, segmentsHeight, 0, Math.PI * 2, Math.PI / 2, Math.PI / 2));
    upperSphere.position.set(0, cylinderHeight / 2, 0);
    lowerSphere.position.set(0, -cylinderHeight / 2, 0);
    upperSphere.updateMatrix();
    lowerSphere.updateMatrix();
    geometry.mergeMesh(upperSphere);
    geometry.mergeMesh(lowerSphere);
    geometry.mergeVertices();
    return new three.BufferGeometry().fromGeometry(geometry);
}

const TimeStep = 1 / 60;
const StepIteration = 4;
const MaxProjectionIteration = 5;
const PreStabilizationIteration = 1;
const PhysicsStabilizationTime = 2;
const Gravity = -9.81 * 14;
const VelocityDampingFactor = 0.00125;
const ClothCompressionStiffness = 1;
const ClothStretchStiffness = 1;
const EnableBendingStiffness = false;
const ClothBendingStiffness = 0.05;
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
document.addEventListener('DOMContentLoaded', async () => {
    const width = window.innerWidth;
    const height = window.innerHeight;
    const stats = new Stats();
    document.body.appendChild(stats.dom);
    const renderer = new three.WebGLRenderer();
    renderer.setClearColor(new three.Color(0xfcfcfc));
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
            stretch: ClothStretchStiffness,
            bending: ClothBendingStiffness
        }
    };
    stabilize_physics(mmd_helper, mmd_mesh, model, rigid_bodies, detector, simulation_config, PhysicsStabilizationTime);
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
function handle_resize(camera, renderer) {
    const width = window.innerWidth;
    const height = window.innerHeight;
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
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
function load_mmd_mesh(model_path, motion_path, helper) {
    return new Promise((resolve, reject) => {
        const loader = new three.MMDLoader();
        loader.load(model_path, [motion_path], mesh => {
            const geometry = mesh.geometry;
            geometry.getAttribute('position').setDynamic(true);
            geometry.getAttribute('normal').setDynamic(true);
            // Since the result of the simulation is written directly to the vertex buffer,
            // it conflicts with morphing animation.
            // TODO: Find a solution other than deleting morphing animation.
            geometry.animations = geometry.animations.filter(c => !is_morph_animation_clip(c));
            helper.add(mesh);
            helper.setAnimation(mesh);
            helper.setPhysics(mesh, { world: build_physics_world() });
            helper.unifyAnimationDuration({ afterglow: 0.0 });
            resolve(mesh);
        }, undefined, reject);
    });
}
function build_model(mmd_mesh) {
    const geometry = mmd_mesh.geometry;
    const vertices = geometry.getAttribute('position').array;
    const skin_indices = geometry.getAttribute('skinIndex').array;
    const skin_weights = geometry.getAttribute('skinWeight').array;
    const material_index = get_material_index_by_name(mmd_mesh.material, TargetMaterialName);
    const geo_group = get_buffer_geometry_group_by_material_id(geometry, material_index);
    const geo_indices = sub_typed_array(geometry.index.array, geo_group.start, geo_group.count);
    const [mesh_indices, vertex_particle_map] = create_particle_mesh_indices(geo_indices);
    const n_particle = vertex_particle_map.size;
    const indices_ctor = get_constructor(geo_indices);
    const vertex_idx_list = new indices_ctor(vertex_particle_map.keys());
    const particle_idx_list = new indices_ctor(vertex_particle_map.values());
    const pairs = { vertex_idx_list, particle_idx_list, length: n_particle };
    const particle_data = create_particle_data(n_particle);
    const particles = to_enumerable(particle_data, particle_accessor);
    copy_particle_positions_from_vertices(particles, vertices, pairs, mmd_mesh.matrixWorld);
    const attachments = find_attached_particles(vertices, skin_indices, skin_weights, pairs);
    for (let i = 0; i < n_particle; ++i) {
        particles[i].mass = ClothMass;
    }
    for (let i = 0, len = attachments.length; i < len; ++i) {
        particles[attachments.particle_indices[i]].mass = 0.0;
    }
    const particle_mesh = build_particle_mesh(particle_data, mesh_indices);
    const constraints = new Array();
    // distance constraint
    const distance_constraints = particle_mesh.edges
        .map(e => create_distance_constraint(particle_data, e.vertex_pair[0], e.vertex_pair[1]));
    Array.prototype.push.apply(constraints, distance_constraints);
    // isometric bending constraint
    if (EnableBendingStiffness) {
        const isometric_constraints = particle_mesh.edges
            .filter(e => !e.face_pair.includes(NullFaceID))
            .map(e => {
            const face0 = particle_mesh.faces[e.face_pair[0]];
            const face1 = particle_mesh.faces[e.face_pair[1]];
            const p1 = e.vertex_pair[0];
            const p2 = e.vertex_pair[1];
            const p3 = face0.vertices.find(v => !e.vertex_pair.includes(v));
            const p4 = face1.vertices.find(v => !e.vertex_pair.includes(v));
            return create_isometric_bending_constraint(particles, p1, p2, p3, p4);
        });
        Array.prototype.push.apply(constraints, isometric_constraints);
    }
    return { particles: particle_data, mesh: particle_mesh, constraints, attachments, pairs };
}
function setup_rigid_bodies(physics) {
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
function stabilize_physics(mmd_helper, mmd_mesh, model, rigid_bodies, detector, simulation_config, stabilize_time) {
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
function update_rigid_body_transform(rigid_bodies) {
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
        const t12 = m32 * m13 - m33 * m12;
        const t13 = m23 * m12 - m22 * m13;
        const inv_det = 1 / (m11 * t11 + m21 * t12 + m31 * t13);
        const dir_x = (m23 * m31 - m21 * m33) * inv_det;
        const dir_y = (m11 * m33 - m13 * m31 * inv_det);
        const dir_z = (m13 * m21 - m11 * m23) * inv_det;
        capsule.set_direction(dir_x, dir_y, dir_z);
        capsule.calc_AABB();
    }
}
function create_loop_runner(environment) {
    const stats = environment.stats;
    const renderer = environment.renderer;
    const scene = environment.scene;
    const camera = environment.camera;
    const mmd_helper = environment.mmd_helper;
    const mmd_mesh = environment.mmd_mesh;
    const geometry = mmd_mesh.geometry;
    const pos_attr = geometry.getAttribute('position');
    const pos_buffer = pos_attr.array;
    const normal_attr = geometry.getAttribute('normal');
    const normal_buffer = normal_attr.array;
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
    const debug_points_geo = environment.debug_points.geometry;
    const debug_points_attr = debug_points_geo.getAttribute('position');
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
        update_vertex_attributes(pos_buffer, positions, normal_buffer, vertex_normals, masses, vertex_particle_pairs, inv_transform.getInverse(mmd_mesh.matrixWorld));
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
function is_morph_animation_clip(clip) {
    return (clip.tracks.length > 0 && clip.tracks[0].name.indexOf('.morphTargetInfluences') === 0);
}
function build_physics_world() {
    const config = new ammo.btDefaultCollisionConfiguration();
    const dispatcher = new ammo.btCollisionDispatcher(config);
    const cache = new ammo.btDbvtBroadphase();
    const solver = new ammo.btSequentialImpulseConstraintSolver();
    const world = new ammo.btDiscreteDynamicsWorld(dispatcher, cache, solver, config);
    world.setGravity(new ammo.btVector3(0, -9.8 * 10, 0));
    const form = new ammo.btTransform();
    form.setIdentity();
    form.setOrigin(new ammo.btVector3(0, -1, 0));
    const ground = new ammo.btRigidBody(new ammo.btRigidBodyConstructionInfo(0, new ammo.btDefaultMotionState(form), new ammo.btBoxShape(new ammo.btVector3(10, 1, 10)), new ammo.btVector3(0, 0, 0)));
    world.addRigidBody(ground);
    return world;
}
function get_material_index_by_name(materials, name) {
    for (let i = 0, len = materials.length; i < len; ++i) {
        if (materials[i].name === name)
            return i;
    }
    return -1;
}
function get_buffer_geometry_group_by_material_id(geometry, material_index) {
    const groups = geometry.groups;
    for (const group of groups) {
        if (group.materialIndex === material_index)
            return group;
    }
    return undefined;
}
function create_particle_mesh_indices(geo_indices) {
    const indices_ctor = get_constructor(geo_indices);
    const vertex_particle_map = new Map();
    const mesh_indices = [];
    let mesh_index = 0;
    for (const geo_index of geo_indices) {
        if (vertex_particle_map.has(geo_index)) {
            mesh_indices.push(vertex_particle_map.get(geo_index));
        }
        else {
            mesh_indices.push(mesh_index);
            vertex_particle_map.set(geo_index, mesh_index++);
        }
    }
    return [new indices_ctor(mesh_indices), vertex_particle_map];
}
const copy_particle_positions_from_vertices = (() => {
    const vert = new three.Vector3();
    return (particles, vertices, pairs, transform) => {
        const vert_ids = pairs.vertex_idx_list;
        const particle_ids = pairs.particle_idx_list;
        for (let i = 0, len = particles.length; i < len; ++i) {
            const X = vert_ids[i] * 3, Y = X + 1, Z = X + 2;
            vert.set(vertices[X], vertices[Y], vertices[Z]).applyMatrix4(transform);
            particles[particle_ids[i]].init_position(vert.x, vert.y, vert.z);
        }
    };
})();
function find_attached_particles(vertices, skin_indices, skin_weights, pairs) {
    const attachments = [];
    const vert_ids = pairs.vertex_idx_list;
    const particle_ids = pairs.particle_idx_list;
    for (let i = 0, len = pairs.length; i < len; ++i) {
        const vert_offset = vert_ids[i] * 3;
        const skin_offset = vert_ids[i] * 4;
        // if the vertex has been bind to bone, it is regarded as an attached particle.
        if (!(skin_indices[skin_offset] === 0.0 && skin_weights[skin_offset] === 1.0)) {
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
    return (particles, attachments, skeleton) => {
        if (attachments.length === 0)
            return;
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
                if (weight === 0.0)
                    continue;
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
    return (pos_buff, positions, normal_buff, normals, masses, pairs, inv_transform) => {
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

}(THREE,Stats,Ammo));
//# sourceMappingURL=bundle-mmd.js.map
