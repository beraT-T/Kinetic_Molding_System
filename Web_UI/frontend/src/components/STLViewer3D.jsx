import React, { useEffect, useRef } from 'react';
import { Activity } from 'lucide-react';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

export default function STLViewer3D({ file }) {
    const mountRef = useRef(null);
    const sceneRef = useRef(null);
    const rendererRef = useRef(null);
    const controlsRef = useRef(null);
    const animationIdRef = useRef(null);
    const cameraRef = useRef(null);
    const cameraHomeRef = useRef(null);

    useEffect(() => {
        if (!mountRef.current) return;

        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x111827);
        sceneRef.current = scene;

        const camera = new THREE.PerspectiveCamera(
            50,
            mountRef.current.clientWidth / mountRef.current.clientHeight,
            0.1,
            10000
        );
        camera.position.set(150, 150, 150);
        cameraRef.current = camera;

        const renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: true
        });
        renderer.setSize(mountRef.current.clientWidth, mountRef.current.clientHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        mountRef.current.appendChild(renderer.domElement);
        rendererRef.current = renderer;

        const controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.08;
        controls.minDistance = 30;
        controls.maxDistance = 800;
        controls.autoRotate = false;
        controls.autoRotateSpeed = 0.5;
        controlsRef.current = controls;

        // Lighting
        scene.add(new THREE.AmbientLight(0xffffff, 0.5));
        scene.add(new THREE.HemisphereLight(0x8899cc, 0x223344, 0.6));

        const mainLight = new THREE.DirectionalLight(0xffffff, 1.4);
        mainLight.position.set(100, 180, 100);
        mainLight.castShadow = true;
        mainLight.shadow.mapSize.width = 2048;
        mainLight.shadow.mapSize.height = 2048;
        scene.add(mainLight);

        const fillLight = new THREE.DirectionalLight(0xaabbcc, 0.4);
        fillLight.position.set(-100, 60, -80);
        scene.add(fillLight);

        // Grid ve Floor — model yüklendikten sonra ölçeklenir, başlangıç değerleri makul
        const gridHelper = new THREE.GridHelper(400, 40, 0x334466, 0x1e2a3a);
        gridHelper.material.opacity = 0.4;
        gridHelper.material.transparent = true;
        gridHelper.name = 'grid';
        scene.add(gridHelper);

        const floorGeometry = new THREE.PlaneGeometry(500, 500);
        const floorMaterial = new THREE.MeshStandardMaterial({
            color: 0x1a2535,
            metalness: 0.2,
            roughness: 0.9,
            opacity: 0.7,
            transparent: true
        });
        const floor = new THREE.Mesh(floorGeometry, floorMaterial);
        floor.rotation.x = -Math.PI / 2;
        floor.position.y = 0;
        floor.receiveShadow = true;
        floor.name = 'floor';
        scene.add(floor);

        const animate = () => {
            animationIdRef.current = requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        };
        animate();

        const handleResize = () => {
            if (!mountRef.current) return;
            const width = mountRef.current.clientWidth;
            const height = mountRef.current.clientHeight;
            camera.aspect = width / height;
            camera.updateProjectionMatrix();
            renderer.setSize(width, height);
        };
        window.addEventListener('resize', handleResize);

        if (file) {
            const loader = new STLLoader();
            const reader = new FileReader();

            reader.onload = (e) => {
                try {
                    const geometry = loader.parse(e.target.result);
                    geometry.computeVertexNormals();

                    // Bounding box hesapla, modeli XZ'de ortala, alt yüzey y=0'a otursun
                    geometry.computeBoundingBox();
                    const geoBox = geometry.boundingBox;
                    const geoSize = new THREE.Vector3();
                    geoBox.getSize(geoSize);
                    const geoCenter = new THREE.Vector3();
                    geoBox.getCenter(geoCenter);
                    geometry.translate(-geoCenter.x, -geoBox.min.y, -geoCenter.z);

                    const material = new THREE.MeshStandardMaterial({
                        color: 0xd8e4f0,
                        metalness: 0.05,
                        roughness: 0.35,
                        flatShading: false,
                    });

                    const mesh = new THREE.Mesh(geometry, material);
                    mesh.castShadow = true;
                    mesh.receiveShadow = true;
                    mesh.name = 'stl-model';

                    const oldMeshes = scene.children.filter(c => c.name === 'stl-model');
                    oldMeshes.forEach(m => {
                        scene.remove(m);
                        m.geometry.dispose();
                        m.material.dispose();
                    });

                    scene.add(mesh);

                    const maxDim = Math.max(geoSize.x, geoSize.y, geoSize.z);

                    // Grid ve floor'u modelin boyutuna göre ölçekle
                    const gridSize = maxDim * 4;
                    const existingGrid = scene.getObjectByName('grid');
                    const existingFloor = scene.getObjectByName('floor');
                    if (existingGrid) {
                        existingGrid.scale.setScalar(gridSize / 400);
                    }
                    if (existingFloor) {
                        existingFloor.scale.setScalar(gridSize / 500);
                    }
                    const target = new THREE.Vector3(0, geoSize.y / 2, 0);
                    const distance = maxDim * 2.5;

                    camera.position.set(distance * 0.8, distance * 0.6, distance * 0.8);
                    camera.near = maxDim * 0.001;
                    camera.far = maxDim * 100;
                    camera.updateProjectionMatrix();
                    camera.lookAt(target);

                    controls.target.copy(target);
                    controls.minDistance = maxDim * 0.05;
                    controls.maxDistance = maxDim * 20;
                    controls.update();

                    cameraHomeRef.current = {
                        position: camera.position.clone(),
                        target: target.clone(),
                    };
                } catch (error) {
                    console.error('STL Error:', error);
                }
            };

            reader.readAsArrayBuffer(file);
        }

        return () => {
            window.removeEventListener('resize', handleResize);
            if (animationIdRef.current) cancelAnimationFrame(animationIdRef.current);
            if (mountRef.current && renderer.domElement && mountRef.current.contains(renderer.domElement)) {
                mountRef.current.removeChild(renderer.domElement);
            }
            scene.traverse((object) => {
                if (object.geometry) object.geometry.dispose();
                if (object.material) {
                    if (Array.isArray(object.material)) {
                        object.material.forEach(m => m.dispose());
                    } else {
                        object.material.dispose();
                    }
                }
            });
            renderer.dispose();
            controls.dispose();
        };
    }, [file]);

    const handleResetCamera = () => {
        const camera = cameraRef.current;
        const controls = controlsRef.current;
        const home = cameraHomeRef.current;
        if (!camera || !controls || !home) return;
        camera.position.copy(home.position);
        controls.target.copy(home.target);
        controls.update();
    };

    if (!file) {
        return <div className="w-full h-full" />;
    }

    return (
        <div className="w-full h-full relative">
            <div ref={mountRef} className="w-full h-full rounded-2xl overflow-hidden" />
            <button
                onClick={handleResetCamera}
                title="Kamerayı sıfırla"
                className="absolute bottom-3 right-3 flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-slate-800/80 hover:bg-slate-700/90 text-slate-300 hover:text-white border border-slate-600/50 text-[10px] font-bold uppercase tracking-wider transition-all backdrop-blur-sm shadow-lg"
            >
                <Activity size={11} />
                Reset Camera
            </button>
        </div>
    );
}