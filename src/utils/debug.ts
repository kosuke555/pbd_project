import * as three from 'three';
import { ParticleData } from '../simulation/ParticleData';
import { sphere_rigid_body_accessor, box_rigid_body_accessor, capsule_rigid_body_accessor,
    SphereRigidBodyAccessor, BoxRigidBodyAccessor, CapsuleRigidBodyAccessor } from '../simulation/rigid_bodies';
import { CollisionObjects } from '../simulation/collision_detection';
import { to_enumerable } from './transformers';

const RigidBodyColor = 0x151cef;
const AABBLineColor = 0xff00ff;

export function particle_helper(particles: ParticleData,
    dynamic_particle_color = [0, 1, 0], static_particle_color = [0, 0, 1]): three.Points {

    const geometry = new three.BufferGeometry();
    const pos_attr = new three.BufferAttribute(particles.position, 3);
    pos_attr.setDynamic(true);
    geometry.addAttribute('position', pos_attr);
    const colors = particles.mass
        .reduce((acc, m) => acc.concat(m !== 0.0 ? dynamic_particle_color : static_particle_color), [] as number[]);
    geometry.addAttribute('color', new three.Float32BufferAttribute(colors, 3));
    const material = new three.PointsMaterial({ vertexColors: three.VertexColors, size: 0.3 });
    return new three.Points(geometry, material);
}

export function rigid_body_helper(rigid_bodies: CollisionObjects, wireframe = true) {
    const render_objects = [] as (three.Mesh | three.LineSegments)[];

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

function create_sphere_mesh(sphere: SphereRigidBodyAccessor, wireframe: boolean) {
    const sphere_geo = new three.SphereBufferGeometry(sphere.radius, 20, 20);
    const sphere_mesh = new three.Mesh(sphere_geo, new three.MeshStandardMaterial({
        color: RigidBodyColor, emissive: new three.Color(0x0f087c),
        roughness: 0.88, metalness: 0.9, wireframe
    }));
    sphere_mesh.userData.update = ((s: typeof sphere, m: three.Mesh) => () => {
        m.position.fromArray(s.get_position());
    })(sphere, sphere_mesh);
    sphere_mesh.userData.update();

    return sphere_mesh;
}

function create_box_mesh(box: BoxRigidBodyAccessor, wireframe: boolean) {
    const box_geo = new three.BoxBufferGeometry(box.half_width * 2, box.half_height * 2, box.half_depth * 2);
    const box_mesh = new three.Mesh(box_geo, new three.MeshStandardMaterial({
        color: RigidBodyColor, emissive: new three.Color(0x0f087c),
        roughness: 0.88, metalness: 0.9, wireframe
    }));
    box_mesh.userData.update = ((b: typeof box, m: three.Mesh) => () => {
        const x_axis = b.get_inv_basis_vector(0);
        const y_axis = b.get_inv_basis_vector(1);
        const z_axis = b.get_inv_basis_vector(2);
        const pos = b.get_position();
        m.matrix.set(
            x_axis[0], y_axis[0], z_axis[0], pos[0],
            x_axis[1], y_axis[1], z_axis[1], pos[1],
            x_axis[2], y_axis[2], z_axis[2], pos[2],
            0, 0, 0, 1
        );
        m.matrix.decompose(m.position, m.quaternion, m.scale);
    })(box, box_mesh);
    box_mesh.userData.update();

    return box_mesh;
}

function create_capsule_mesh(capsule: CapsuleRigidBodyAccessor, wireframe: boolean) {
    const capsule_geo = create_capsule_geometry(capsule.radius, capsule.length, 16, 8);
    const capsule_mesh = new three.Mesh(capsule_geo, new three.MeshStandardMaterial({
        color: RigidBodyColor, emissive: new three.Color(0x0f087c),
        roughness: 0.88, metalness: 0.9, wireframe
    }));
    capsule_mesh.userData.update = ((c: typeof capsule, m: three.Mesh) => {
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

function create_AABB_line_segments(object: SphereRigidBodyAccessor | BoxRigidBodyAccessor | CapsuleRigidBodyAccessor) {
    const indices = new Uint16Array([0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7]);
    const positions = new Float32Array(8 * 3);

    const geometry = new three.BufferGeometry();
	geometry.setIndex(new three.BufferAttribute(indices, 1));
    geometry.addAttribute('position', new three.BufferAttribute(positions, 3));

    const line_segments = new three.LineSegments(geometry,
        new three.LineBasicMaterial({ color: AABBLineColor }));
    line_segments.matrixAutoUpdate = false;
    line_segments.userData.update = ((o: typeof object, pos: three.BufferAttribute) => {
        const min = new three.Vector3();
        const max = new three.Vector3();
        return () => {
            const AABB = o.get_AABB();
            min.fromArray(AABB[0]);
            max.fromArray(AABB[1]);
            const array = pos.array as Float32Array;
            array[ 0 ] = max.x; array[ 1 ] = max.y; array[ 2 ] = max.z;
            array[ 3 ] = min.x; array[ 4 ] = max.y; array[ 5 ] = max.z;
            array[ 6 ] = min.x; array[ 7 ] = min.y; array[ 8 ] = max.z;
            array[ 9 ] = max.x; array[ 10 ] = min.y; array[ 11 ] = max.z;
            array[ 12 ] = max.x; array[ 13 ] = max.y; array[ 14 ] = min.z;
            array[ 15 ] = min.x; array[ 16 ] = max.y; array[ 17 ] = min.z;
            array[ 18 ] = min.x; array[ 19 ] = min.y; array[ 20 ] = min.z;
            array[ 21 ] = max.x; array[ 22 ] = min.y; array[ 23 ] = min.z;
            pos.needsUpdate = true;
        };
    })(object, geometry.getAttribute('position') as three.BufferAttribute);
    line_segments.userData.update();

    return line_segments;
}

function create_capsule_geometry(radius: number, cylinderHeight: number, segmentsRadius: number, segmentsHeight: number) {
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
