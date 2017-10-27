declare module 'ammo.js' {

    export function castObject<T>(object: any, clazz: { new (...args: any[]): T }): T;

    export class btDefaultCollisionConstructionInfo {
        constructor();
    }

    export class btDefaultCollisionConfiguration {
        constructor(info?: btDefaultCollisionConstructionInfo);
    }

    export class btPersistentManifold {
        constructor();

        getBody0(): btCollisionObject;
        getBody1(): btCollisionObject;
    }

    export interface btDispatcher {
        getNumManifolds(): number;
        getManifoldByIndexInternal(index: number): btPersistentManifold;
    }

    export class btCollisionDispatcher implements btDispatcher {
        constructor(conf: btDefaultCollisionConfiguration);

        getNumManifolds(): number;
        getManifoldByIndexInternal(index: number): btPersistentManifold;
    }

    export class btDbvtBroadphase {
        constructor();
    }

    export class btSequentialImpulseConstraintSolver {
        constructor();
    }

    export class btDiscreteDynamicsWorld {
        constructor(dispatcher: btDispatcher, pairCache: btDbvtBroadphase, constraintSolver: btSequentialImpulseConstraintSolver, collisionConfiguration: btDefaultCollisionConfiguration);

        setGravity(gravity: btVector3): void;
        addRigidBody(body: btRigidBody): void;
        addRigidBody(body: btRigidBody, group: number, mask: number): void;
    }

    export interface btCollisionObject {
        getCollisionShape(): btCollisionShape;
    }

    export class btRigidBody implements btCollisionObject {
        constructor(constructionInfo: any);

        getCollisionShape(): btCollisionShape;
        getCenterOfMassTransform(): btTransform;
    }

    export interface btMotionState {
        getWorldTransform(worldTrans: btTransform): void;
        setWorldTransform(worldTrans: btTransform): void;
    }

    export class btDefaultMotionState implements btMotionState {
        constructor(startTrans?: btTransform, centerOfMassOffset?: btTransform);

        m_graphicsWorldTrans: btTransform;
        getWorldTransform(worldTrans: btTransform): void;
        setWorldTransform(worldTrans: btTransform): void;
    }

    export class btRigidBodyConstructionInfo {
        constructor(mass: number, motionState: btMotionState, collisionShape: btCollisionShape, localInertia?: btVector3);
    }

    export interface btCollisionShape { }

    export interface btConvexShape extends btCollisionShape { }

    export class btSphereShape implements btCollisionShape {
        constructor(radius: number);
    }

    export class btBoxShape implements btCollisionShape {
        constructor(boxHalfExtents: btVector3);
    }

    export class btCapsuleShape implements btCollisionShape {
        constructor(radius: number, height: number);
    }

    export class btTransform {
        constructor();
        constructor(q: btQuaternion, v: btVector3);

        setIdentity(): void;
        setOrigin(origin: btVector3): void;
        getBasis(): btMatrix3x3;
        getOrigin(): btVector3;
        getRotation(): btQuaternion;
    }

    export interface btMatrix3x3 {
        setEulerZYX(ex: number, ey: number, ez: number): void;
        getRotation(q: btQuaternion): void;
        getRow(y: number): btVector3;
    }

    export class btVector3 {
        constructor();
        constructor(x: number, y: number, z: number);

        x(): number;
        y(): number;
        z(): number;
    }

    export class btQuaternion {
        constructor(x: number, y: number, z: number, w: number);

        x(): number;
        y(): number;
        z(): number;
        w(): number;
    }

}
