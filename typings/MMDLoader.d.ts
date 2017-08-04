import * as three from 'three';

declare module 'three' {

    export class MMDLoader extends three.Loader {
        constructor(manager?: three.LoadingManager);

        load(modelUrl: string, vmdUrls: string[], callback: (mesh: three.Mesh) => void,
            onProgress?: (event: ProgressEvent) => void, onError?: (event: ErrorEvent) => void): void;
    }

    export interface MMDPhysicsParams {
        unitStep?: number;
        maxStepNum?: number;
        world?: any;
    }

    export class MMDHelper {
        constructor();

        doAnimation: boolean;
        doIk: boolean;
        doGrant: boolean;
        doCameraAnimation: boolean;

        add(mesh: three.Mesh): void;
        setAnimation(mesh: three.Mesh): void;
        setPhysics(mesh: three.Mesh,
            params?: { warmup?: number, preventAnimationWarmup?: boolean } & MMDPhysicsParams): void;
        enablePhysics(enabled: boolean): void;
        unifyAnimationDuration(params?: { afterglow?: number }): void;
        animate(delta: number): void;
    }

}
