import * as three from 'three';

declare module 'three' {

    export interface MMDBone { }

    export interface MMDIk { }

    export interface MMDGrant { }

    // collision shape type
    // 0: sphere, 1: box, 2: capsule
    export type MMDRigidBodyShapeType = 0 | 1 | 2;

    export interface MMDRigidBody {
        shapeType: MMDRigidBodyShapeType;
        width: number;
        height: number;
        depth: number;
        type: number;
        boneIndex: number;
        position: [number, number, number];
    }

    export interface MMDConstraint { }

    export interface MMDData {
        mmdFormat: 'pmd'|'pmx';
        bones: MMDBone[];
        iks: MMDIk[];
        grants: MMDGrant[];
        rigidBodies: MMDRigidBody[];
        constraints: MMDConstraint[];
        animations: three.AnimationClip[];
    }

    export interface MMDMesh extends three.SkinnedMesh {
        geometry: three.BufferGeometry & MMDData;
    }

    export class MMDLoader extends three.Loader {
        constructor(manager?: three.LoadingManager);

        load(modelUrl: string, vmdUrls: string[], callback: (mesh: MMDMesh) => void,
            onProgress?: (event: ProgressEvent) => void, onError?: (event: ErrorEvent) => void): void;
    }

    export interface AdditionalMMDAnimationData {
        mixer: three.AnimationMixer;
    }

    export interface AdditionalMMDPhysicsData {
        physics: MMDPhysics;
    }

    export class MMDHelper {
        constructor();

        doAnimation: boolean;
        doIk: boolean;
        doGrant: boolean;
        doCameraAnimation: boolean;

        add(mesh: MMDMesh): void;
        setAnimation(mesh: MMDMesh): void;
        setPhysics(mesh: MMDMesh,
            params?: { warmup?: number, preventAnimationWarmup?: boolean } & MMDPhysicsParams): void;
        enablePhysics(enabled: boolean): void;
        unifyAnimationDuration(params?: { afterglow?: number }): void;
        animate(delta: number): void;
    }

}
