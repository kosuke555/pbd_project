import * as three from 'three';
import Stats from 'stats.js';
import Ammo from 'ammo.js';
import CanvasRecorder from './utils/CanvasRecorder';

const TimeStep = 1/60;

const DefaultMMDModel = 'assets/sylvie/sylvie2.pmx';
const DefaultMMDMotion = 'assets/vmd/agura.vmd';

document.addEventListener('DOMContentLoaded', () => {
    const width = window.innerWidth;
    const height = window.innerHeight;

    const stats = new Stats();
    document.body.appendChild(stats.dom);

    const renderer = new three.WebGLRenderer();
    renderer.setClearColor(new three.Color(0xfcfcfc))
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(width, height);
    document.body.appendChild(renderer.domElement);

    const recorder = CanvasRecorder.createFromUrlParams(renderer.domElement, document.location.href);
    
    const camera = new three.PerspectiveCamera(45, width / height, 1, 1000);
    camera.position.set(1, 1, 1).setLength(30);
    camera.lookAt(new three.Vector3(0, 10, 0));

    const mmd_helper = new three.MMDHelper();
    const scene = build_scene(mmd_helper);

    const render = () => {
        stats.begin();

        mmd_helper.animate(TimeStep);
        renderer.render(scene, camera);

        stats.end();

        requestAnimationFrame(render);
    };

    window.addEventListener('resize', () => {
        handle_resize(camera, renderer);
    });

    recorder && recorder.start();
    render();
});

function handle_resize(camera: three.PerspectiveCamera, renderer: three.WebGLRenderer) {
    const width = window.innerWidth;
    const height = window.innerHeight;
    renderer.setSize(width, height);
    camera.aspect = width / height;
}

function build_scene(helper: three.MMDHelper): three.Scene {
    const scene = new three.Scene();

    const loader = new three.MMDLoader();
    loader.load(DefaultMMDModel, [DefaultMMDMotion], mesh => {
        scene.add(mesh);
        helper.add(mesh);
        helper.setAnimation(mesh);
        helper.setPhysics(mesh, { world: build_physics_world() });
        helper.unifyAnimationDuration({ afterglow: 2.0 });
    });

    const light = new three.DirectionalLight(0x887766);
    light.position.set(-1, 1, 1).normalize();
    const ambient = new three.AmbientLight(0x666666);
    scene.add(light);
    scene.add(ambient);

    return scene;
}

function build_physics_world() {
    const config = new Ammo.btDefaultCollisionConfiguration();
    const dispatcher = new Ammo.btCollisionDispatcher( config );
    const cache = new Ammo.btDbvtBroadphase();
    const solver = new Ammo.btSequentialImpulseConstraintSolver();
    const world = new Ammo.btDiscreteDynamicsWorld( dispatcher, cache, solver, config );
    world.setGravity( new Ammo.btVector3( 0, -9.8 * 10, 0 ) );

    const form = new Ammo.btTransform();
    form.setIdentity();
    form.setOrigin(new Ammo.btVector3(0, -1, 0));
    const ground = new Ammo.btRigidBody(
        new Ammo.btRigidBodyConstructionInfo(
            0,
            new Ammo.btDefaultMotionState(form),
            new Ammo.btBoxShape(new Ammo.btVector3(10, 1, 10)),
            new Ammo.btVector3(0, 0, 0)
        )
    );
    world.addRigidBody(ground);
    
    return world;
}
