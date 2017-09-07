import * as three from 'three';
import * as ammo from 'ammo.js';

declare module 'three' {

    export interface MMDPhysicsParams {
        unitStep?: number;
        maxStepNum?: number;
        world?: ammo.btDiscreteDynamicsWorld;
    }

    export class MMDPhysics {
        constructor(mesh: three.SkinnedMesh, params: MMDPhysicsParams);

        mesh: three.SkinnedMesh;
        helper: MMDPhysics.ResourceHelper;
        unitStep: number;
        maxStepNum: number;
        world: ammo.btDiscreteDynamicsWorld;
        bodies: MMDPhysics.RigidBody[];
        constraints: MMDPhysics.Constraint[];
    }

    export namespace MMDPhysics {

        export class ResourceHelper {
            constructor();
        }

        export class RigidBody {
            constructor(mesh: three.SkinnedMesh, world: ammo.btDiscreteDynamicsWorld, params: MMDRigidBody, helper: ResourceHelper);

            mesh: three.SkinnedMesh;
            world: ammo.btDiscreteDynamicsWorld;
            params: MMDRigidBody;
            helper: ResourceHelper;
            body: ammo.btRigidBody;
            bone: three.Bone;
            boneOffsetForm: ammo.btTransform;
            boneOffsetFormInverse: ammo.btTransform;
        }

        export class Constraint {
            constructor(mesh: three.SkinnedMesh, world: ammo.btDiscreteDynamicsWorld, bodyA: RigidBody, bodyB: RigidBody, params: MMDConstraint, helper: ResourceHelper);
        }

    }

}
