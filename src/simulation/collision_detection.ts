import { ParticleData } from './ParticleData';
import { SphereRigidBodies, BoxRigidBodies, CapsuleRigidBodies, PlaneRigidBodies } from './rigid_bodies';
import { ContactConstraint, create_contact_constraint, ContactConstraintAllocator } from './constraints';
import { SpatialHash, SpatialHashParams } from './SpatialHash';
import { EPSILON } from './math';

export interface CollisionObjects {
    spheres?: Readonly<SphereRigidBodies>;
    boxes?: Readonly<BoxRigidBodies>;
    capsules?: Readonly<CapsuleRigidBodies>;
    planes?: Readonly<PlaneRigidBodies>;
}

export interface CollisionTolerances {
    sphere: number;
    box: number;
    capsule: number;
    plane: number;
}

const floor = Math.floor;
const sqrt = Math.sqrt;
const min = Math.min;
const max = Math.max;

export class CollisionDetector {
    private _allocator: ContactConstraintAllocator;
    private _broadphase: SpatialHash<number>;
    private _timestamp = 0;

    constructor(broadphase_params: SpatialHashParams) {
        this._allocator = new ContactConstraintAllocator();
        this._broadphase = new SpatialHash(broadphase_params);
    }

    release_collision_constraints() {
        this._allocator.release_all();
    }

    generate_collision_constraints(particles: ParticleData, objects: Readonly<CollisionObjects>) {
        const position = particles.position;
        const mass = particles.mass;
        const broadphase = this._broadphase;
        const inv_grid_cell_size = 1 / broadphase.grid_cell_size;
        const timestamp = this._timestamp;

        // store particles in the hash table.
        for (let i = 0, len = particles.length; i < len; ++i) {
            if (mass[i] === 0.0) continue;
            const x = floor(position[i * 3] * inv_grid_cell_size);
            const y = floor(position[i * 3 + 1] * inv_grid_cell_size);
            const z = floor(position[i * 3 + 2] * inv_grid_cell_size);
            broadphase.set(x, y, z, timestamp, i);
        }

        const constraints = [] as ContactConstraint[];
        const allocator = this._allocator;

        const sphere_bodies = objects.spheres;
        if (sphere_bodies) {
            dispatch_collision_handler(sphere_bodies, particles, broadphase, timestamp,
                (particle_id: number, object_id: number,
                    p_x: number, p_y: number, p_z: number,
                    op_x: number, op_y: number, op_z: number,
                    n_x: number, n_y: number, n_z: number,
                    delta_len: number) => {

                    const c = create_sphere_contact_constraint(particle_id, object_id, sphere_bodies,
                        p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator);

                    c && constraints.push(c);
                });
        }

        const box_bodies = objects.boxes;
        if (box_bodies) {
            dispatch_collision_handler(box_bodies, particles, broadphase, timestamp,
                (particle_id: number, object_id: number,
                    p_x: number, p_y: number, p_z: number,
                    op_x: number, op_y: number, op_z: number,
                    n_x: number, n_y: number, n_z: number,
                    delta_len: number) => {
    
                    const c = create_box_contact_constraint(particle_id, object_id, box_bodies,
                        p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator);
    
                    c && constraints.push(c);
                });
        }

        const capsule_bodies = objects.capsules;
        if (capsule_bodies) {
            dispatch_collision_handler(capsule_bodies, particles, broadphase, timestamp,
                (particle_id: number, object_id: number,
                    p_x: number, p_y: number, p_z: number,
                    op_x: number, op_y: number, op_z: number,
                    n_x: number, n_y: number, n_z: number,
                    delta_len: number) => {
    
                    const c = create_capsule_contact_constraint(particle_id, object_id, capsule_bodies,
                        p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator);
    
                    c && constraints.push(c);
                });
        }

        const plane_bodies = objects.planes;
        if (plane_bodies) {
            const old_pos = particles.old_position;
            for (let i = 0, len = particles.length; i < len; ++i) {
                if (mass[i] === 0.0) continue;

                // FIXME: DRY
                const X = i * 3, Y = X + 1, Z = X + 2;
                const p_x = position[X]
                const p_y = position[Y];
                const p_z = position[Z];
                const op_x = old_pos[X];
                const op_y = old_pos[Y];
                const op_z = old_pos[Z];
                const d_x = p_x - op_x;
                const d_y = p_y - op_y;
                const d_z = p_z - op_z;
                const sq_len = (d_x * d_x) + (d_y * d_y) + (d_z * d_z);
                const delta_len = sqrt(sq_len);
                if (delta_len < EPSILON) continue;
                const inv_len = 1 / delta_len;
                const n_x = d_x * inv_len;
                const n_y = d_y * inv_len;
                const n_z = d_z * inv_len;

                generate_plane_contact_constraints(constraints, i, plane_bodies,
                    p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len, allocator);
            }
        }

        this._timestamp = timestamp + 1;

        return constraints;
    }
}

type CollisionObjectArray = SphereRigidBodies | BoxRigidBodies | CapsuleRigidBodies;
type CollisionHandler = (
    particle_id: number, object_id: number,
    p_x: number, p_y: number, p_z: number,
    op_x: number, op_y: number, op_z: number,
    n_x: number, n_y: number, n_z: number,
    delta_len: number ) => void;

function dispatch_collision_handler(objects: Readonly<CollisionObjectArray>, particles: ParticleData,
    broadphase: SpatialHash<number>, timestamp: number, handler: CollisionHandler) {

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
                    if (!p_list) continue;
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
                            const delta_len = sqrt(sq_len);
                            if (delta_len < EPSILON) continue;
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

function AABB_contain_point(min_x: number, min_y: number, min_z: number,
    max_x: number, max_y: number, max_z: number, p_x: number, p_y: number, p_z: number) {

    return (p_x >= min_x && max_x >= p_x
        && p_y >= min_y && max_y >= p_y
        && p_z >= min_z && max_z >= p_z);
}

const create_sphere_contact_constraint = (() => {
    const cp = [] as number[];
    const cn = [] as number[];
    return (particle_id: number, object_id: number,
        sphere_bodies: Readonly<SphereRigidBodies>,
        p_x: number, p_y: number, p_z: number,
        op_x: number, op_y: number, op_z: number,
        n_x: number, n_y: number, n_z: number,
        delta_len: number, allocator: ContactConstraintAllocator) => {

        const positions = sphere_bodies.positions;
        const sp_x = positions[object_id * 3];
        const sp_y = positions[object_id * 3 + 1];
        const sp_z = positions[object_id * 3 + 2];
        const tolerance = sphere_bodies.tolerances[object_id];
        const radius = sphere_bodies.radiuses[object_id] + tolerance;
        const sq_radius = radius * radius;

        const t_min = ray_test_sphere(op_x, op_y, op_z, n_x, n_y, n_z, delta_len, sp_x, sp_y,sp_z, sq_radius);
        if (t_min !== t_min) return undefined;

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
        // static collision
        else {
            static_collision_sphere_contact_info(p_x, p_y, p_z, sp_x, sp_y, sp_z, radius, cp, cn);
            return create_contact_constraint(particle_id, cn[0], cn[1], cn[2], cp[0], cp[1], cp[2], cn[0], cn[1], cn[2], allocator);
        }
    };
})();

function create_box_contact_constraint(
    particle_id: number, object_id: number,
    box_bodies: Readonly<BoxRigidBodies>,
    p_x: number, p_y: number, p_z: number,
    op_x: number, op_y: number, op_z: number,
    n_x: number, n_y: number, n_z: number,
    delta_len: number, allocator: ContactConstraintAllocator) {

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
    } else {
        t_min = (box_x_max - lop_x) * inv_ln_x;
        t_max = (box_x_min - lop_x) * inv_ln_x;
    }

    let ty_min = 0, ty_max = 0;
    if (inv_ln_y >= 0.0) {
        ty_min = (box_y_min - lop_y) * inv_ln_y;
        ty_max = (box_y_max - lop_y) * inv_ln_y;
    } else {
        ty_min = (box_y_max - lop_y) * inv_ln_y;
        ty_max = (box_y_min - lop_y) * inv_ln_y;
    }

    if ((t_min > ty_max) || (ty_min > t_max)) return undefined;
    if (ty_min > t_min || t_min !== t_min) { t_min = ty_min; c_plane = inv_ln_y >= 0.0 ? 1 : 4; }
    if (ty_max < t_max || t_max !== t_max) t_max = ty_max;

    let tz_min = 0, tz_max = 0;
    if (inv_ln_z >= 0.0) {
        tz_min = (box_z_min - lop_z) * inv_ln_z;
        tz_max = (box_z_max - lop_z) * inv_ln_z;
    } else {
        tz_min = (box_z_max - lop_z) * inv_ln_z;
        tz_max = (box_z_min - lop_z) * inv_ln_z;
    }

    if ((t_min > tz_max) || (tz_min > t_max)) return undefined;
    if (tz_min > t_min || t_min !== t_min) { t_min = tz_min; c_plane = inv_ln_z >= 0.0 ? 2 : 5; }
    if (tz_max < t_max || t_max !== t_max) t_max = tz_max;

    if (t_max < 0.0 || t_min > delta_len) return undefined;
    if (t_min < 0.0 && t_max <= delta_len) return undefined;

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
    // static collision
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
        const t_min = min(min_x, min_y, min_z);
        if (t_min === min_x) c_plane = lp_x >= 0.0 ? 0 : 3;
        if (t_min === min_y) c_plane = lp_y >= 0.0 ? 1 : 4;
        if (t_min === min_z) c_plane = lp_z >= 0.0 ? 2 : 5;
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
    const cp = [] as number[];
    const cn = [] as number[];
    return (particle_id: number, object_id: number,
        capsule_bodies: Readonly<CapsuleRigidBodies>,
        p_x: number, p_y: number, p_z: number,
        op_x: number, op_y: number, op_z: number,
        n_x: number, n_y: number, n_z: number,
        delta_len: number, allocator: ContactConstraintAllocator) => {

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
        if ( test_capsule_sphere(p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len,
            sp1_x, sp1_y, sp1_z, sp2_x, sp2_y, sp2_z, radius, sq_radius, cp, cn) ) {

            return create_contact_constraint(particle_id, cn[0], cn[1], cn[2], cp[0], cp[1], cp[2], cn[0], cn[1], cn[2], allocator);
        }

        // test sphere 2
        if ( test_capsule_sphere(p_x, p_y, p_z, op_x, op_y, op_z, n_x, n_y, n_z, delta_len,
            sp2_x, sp2_y, sp2_z, sp1_x, sp1_y, sp1_z, radius, sq_radius, cp, cn) ) {

            return create_contact_constraint(particle_id, cn[0], cn[1], cn[2], cp[0], cp[1], cp[2], cn[0], cn[1], cn[2], allocator);
        }

        // test cilinder
        const ct_min = ray_test_inf_cilinder(op_x, op_y, op_z, n_x, n_y, n_z, delta_len,
            cap_x, cap_y, cap_z, dir_x, dir_y, dir_z, sq_radius);

        if (ct_min !== ct_min) return undefined;

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
            } else {
                return undefined;
            }
        }
        // static collision
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
                const v_len = sqrt(v_x * v_x + v_y * v_y + v_z * v_z);
                const inv_v_len = 1 / v_len;
                const cn_x = v_x * inv_v_len;
                const cn_y = v_y * inv_v_len;
                const cn_z = v_z * inv_v_len;
                const cp_x = radius * cn_x + center_x;
                const cp_y = radius * cn_y + center_y;
                const cp_z = radius * cn_z + center_z;
                return create_contact_constraint(particle_id, cn_x, cn_y, cn_z, cp_x, cp_y, cp_z, cn_x, cn_y, cn_z, allocator);
            } else {
                return undefined;
            }
        }
    };
})();

function generate_plane_contact_constraints(constraints: ContactConstraint[],
    particle_id: number,
    plane_bodies: Readonly<PlaneRigidBodies>,
    p_x: number, p_y: number, p_z: number,
    op_x: number, op_y: number, op_z: number,
    n_x: number, n_y: number, n_z: number,
    delta_len: number, allocator: ContactConstraintAllocator) {

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
            if (dist === 0) t_min = 0;
            else continue;
        } else {
            t_min = -dist / denom;
        }

        if (t_min > delta_len) continue;

        // continuous collision
        if (t_min >= 0.0) {
            const cp_x = op_x + t_min * n_x;
            const cp_y = op_y + t_min * n_y;
            const cp_z = op_z + t_min * n_z;

            // TODO: What is the correct gradient function of continuous collision?
            constraints.push(create_contact_constraint(particle_id, pn_x, pn_y, pn_z, cp_x, cp_y, cp_z, pn_x, pn_y, pn_z, allocator));
        }
        // static collision
        else {
            const cp_x = pn_x * -dist + p_x;
            const cp_y = pn_y * -dist + p_y;
            const cp_z = pn_z * -dist + p_z;

            constraints.push(create_contact_constraint(particle_id, pn_x, pn_y, pn_z, cp_x, cp_y, cp_z, pn_x, pn_y, pn_z, allocator));
        }
    }
}

function static_collision_sphere_contact_info(p_x: number, p_y: number, p_z: number,
    sp_x: number, sp_y: number, sp_z: number, radius: number, out_cp: number[], out_cn: number[]) {

    const spp_x = p_x - sp_x;
    const spp_y = p_y - sp_y;
    const spp_z = p_z - sp_z;
    const spp_len = sqrt((spp_x * spp_x) + (spp_y * spp_y) + (spp_z * spp_z));
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

function ray_test_sphere(orig_x: number, orig_y: number, orig_z: number,
    dir_x: number, dir_y: number, dir_z: number, ray_length: number,
    sphere_x: number, sphere_y: number, sphere_z: number, sq_radius: number) {

    const lo_s_x = sphere_x - orig_x;
    const lo_s_y = sphere_y - orig_y;
    const lo_s_z = sphere_z - orig_z;
    const B = (dir_x * lo_s_x) + (dir_y * lo_s_y) + (dir_z * lo_s_z);
    const C = (lo_s_x * lo_s_x) + (lo_s_y * lo_s_y) + (lo_s_z * lo_s_z) - sq_radius;
    const sq = B * B - C;

    if (sq < 0.0) return NaN;

    const s = sqrt(sq);
    const a1 = B - s, a2 = B + s;
    const t_min = min(a1, a2);
    const t_max = max(a1, a2);

    if (t_max < 0.0 || t_min > ray_length) return NaN;
    if (t_min < 0.0 && t_max <= ray_length) return NaN;

    return t_min;
}

function ray_test_inf_cilinder(orig_x: number, orig_y: number, orig_z: number,
    dir_x: number, dir_y: number, dir_z: number, ray_length: number,
    cp_x: number, cp_y: number, cp_z: number, cdir_x: number, cdir_y: number, cdir_z: number, sq_radius: number) {

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

    if (A === 0.0) return NaN;

    const sq = B * B - A * C;

    if (sq < 0.0) return NaN;

    const s = sqrt(sq);
    const inv_A = 1 / A;
    const a1 = (B - s) * inv_A;
    const a2 = (B + s) * inv_A;
    const t_min = min(a1, a2);
    const t_max = max(a1, a2);

    if (t_max < 0.0 || t_min > ray_length) return NaN;
    if (t_min < 0.0 && t_max <= ray_length) return NaN;

    return t_min;
}

function test_capsule_sphere(
    p_x: number, p_y: number, p_z: number,
    op_x: number, op_y: number, op_z: number,
    n_x: number, n_y: number, n_z: number, delta_length: number,
    sp_x: number, sp_y: number, sp_z: number,
    sp2_x: number, sp2_y: number, sp2_z: number, radius: number, sq_radius: number,
    out_cp: number[], out_cn: number[]) {

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
        // static collision
        else {
            if (three_vectors_dot(sp2_x, sp2_y, sp2_z, sp_x, sp_y, sp_z, p_x, p_y, p_z) <= 0.0) {
                static_collision_sphere_contact_info(p_x, p_y, p_z, sp_x, sp_y, sp_z, radius, out_cp, out_cn);
                return true;
            }
        }
    }
    return false;
}

function three_vectors_dot(
    x1: number, y1: number, z1: number,
    x2: number, y2: number, z2: number,
    x3: number, y3: number, z3: number) {

    return (x1 - x2) * (x3 - x2) + (y1 - y2) * (y3 - y2) + (z1 - z2) * (z3 - z2);
}
