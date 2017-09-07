declare module 'ammo.js';

declare module 'ammo.js' {

    export class btDiscreteDynamicsWorld {
        constructor(dispatcher: any, pairCache: any, constraintSolver: any, collisionConfiguration: any);
    }

    export class btRigidBody {
        constructor(constructionInfo: any);

        getCollisionShape(): btCollisionShape;
        getCenterOfMassTransform(): btTransform;
    }

    export class btCollisionShape { }

    export class btConvexShape extends btCollisionShape { }

    export class btConvexInternalShape extends btConvexShape { }

    export class btPolyhedralConvexShape extends btConvexInternalShape { }

    export class btSphereShape extends btConvexInternalShape {
        getRadius(): number;
    }

    export class btBoxShape extends btPolyhedralConvexShape {
        getHalfExtentsWithMargin(): btVector3;
    }

    export class btCapsuleShape extends btConvexInternalShape {
        getRadius(): number;
        getHalfHeight(): number;
    }

    export class btTransform {
        constructor();

        getBasis(): btMatrix3x3;
        getOrigin(): btVector3;
        getRotation(): btQuaternion;
    }

    export class btMatrix3x3 {
        op_get(index: number): btVector3;
    }

    export class btVector3 {
        x(): number;
        y(): number;
        z(): number;
    }

    export class btQuaternion {
        x(): number;
        y(): number;
        z(): number;
        w(): number;
    }

}
