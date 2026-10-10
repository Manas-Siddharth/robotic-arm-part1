/**
 * High-Fidelity 3D Visualization of 5-DOF Dynamixel AX-12A Robotic Arm.
 *
 * Models physical hardware characteristics:
 * - Silver rectangular base enclosure and circular rotating turntable
 * - Dual vertical aluminum upright brackets with weight-reduction cutouts
 * - Detailed black Dynamixel AX-12A modular servo housings (ribs, drive horns, idler bearings)
 * - Silver stamped/machined aluminum U-brackets, side plates, cross-spacers, and hardware screws
 * - White 3-pin flexible wiring routed along links with flexible pivot loops
 * - Black two-finger parallel gripper with mounting bracket and visible actuator servo
 *
 * Kinematic Hierarchy (matching backend/kinematics.py):
 * L1 Base Height: 145mm  (Base turntable to Shoulder pitch axis)
 * L2 Upper Arm:   105mm  (Shoulder pitch axis to Elbow pitch axis)
 * L3 Forearm:     105mm  (Elbow pitch axis to Wrist pitch axis)
 * L4 Wrist/Eff:    90mm  (Wrist pitch axis to Gripper fingertip)
 *
 * Convention:
 * J1 (Base Yaw):       150° = 0 rad (along +X axis, rotation about World Y)
 * J2 (Shoulder Pitch): 150° = 0 rad (horizontal along +X), 240° = +90° (vertical up +Y)
 * J3 (Elbow Pitch):    150° = 0 rad (inline with Upper Arm)
 * J4 (Wrist Pitch):    150° = 0 rad (inline with Forearm)
 * J5 (Wrist Roll):     150° = 0 rad (gripper centered, rotation about local X)
 */

class Arm3DView {
  constructor(containerId) {
    this.container = document.getElementById(containerId);
    if (!this.container) return;

    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;

    // Kinematic Joint Groups
    this.baseGroup = null; // Stationary base enclosure & turntable ring
    this.j1Group = null;   // Base Yaw (rotates about Y)
    this.j2Group = null;   // Shoulder Pitch (rotates about Z)
    this.j3Group = null;   // Elbow Pitch (rotates about Z)
    this.j4Group = null;   // Wrist Pitch (rotates about Z)
    this.j5Group = null;   // Wrist Roll (rotates about local X)
    this.endEffectorMesh = null;

    // Target Ghost Hierarchy
    this.targetGroup = null;
    this.targetJ1 = null;
    this.targetJ2 = null;
    this.targetJ3 = null;
    this.targetJ4 = null;
    this.targetJ5 = null;
    this.showTarget = false; // Off by default so physical arm is unobstructed

    // Configurable Link Dimensions (mm)
    this.L1 = 145.0; // Base to Shoulder pitch axis
    this.L2 = 105.0; // Shoulder to Elbow pitch axis
    this.L3 = 105.0; // Elbow to Wrist pitch axis
    this.L4 = 90.0;  // Wrist pitch axis to Gripper tip

    // Base enclosure dimensions
    this.baseBoxHeight = 32.0;

    this.currentAngles = { 1: 150, 2: 150, 3: 150, 4: 150, 5: 150 };
    this.targetAngles = { 1: 150, 2: 150, 3: 150, 4: 150, 5: 150 };

    this.initMaterials();
    this.init();
  }

  initMaterials() {
    // Brushed silver aluminum (stamped/milled brackets, links, turntable, stanchions)
    this.matSilver = new THREE.MeshStandardMaterial({
      color: 0xecf0f5,
      metalness: 0.38,
      roughness: 0.32,
    });

    // Darker silver / edge chamfers / bearing races
    this.matSilverDark = new THREE.MeshStandardMaterial({
      color: 0xc8d1dc,
      metalness: 0.35,
      roughness: 0.38,
    });

    // Rectangular base enclosure silver sheet aluminum
    this.matBaseEnclosure = new THREE.MeshStandardMaterial({
      color: 0xe2e8f0,
      metalness: 0.30,
      roughness: 0.36,
    });

    // Dynamixel AX-12A black engineering plastic casing (contrasting matte black)
    this.matBlackServo = new THREE.MeshStandardMaterial({
      color: 0x18191c,
      metalness: 0.12,
      roughness: 0.55,
    });

    // Servo drive horn & idler hub
    this.matServoHorn = new THREE.MeshStandardMaterial({
      color: 0x2d3139,
      metalness: 0.40,
      roughness: 0.35,
    });

    // Chrome / steel hardware screws, bolts, and standoff spacers
    this.matHardware = new THREE.MeshStandardMaterial({
      color: 0xf8fafc,
      metalness: 0.85,
      roughness: 0.15,
    });

    // White 3-pin flexible wiring cables
    this.matCable = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      metalness: 0.02,
      roughness: 0.45,
    });

    // Black nylon cable clips & zip ties
    this.matCableClip = new THREE.MeshStandardMaterial({
      color: 0x1e2025,
      metalness: 0.10,
      roughness: 0.70,
    });

    // Black anodized two-finger gripper plates
    this.matGripper = new THREE.MeshStandardMaterial({
      color: 0x181a1f,
      metalness: 0.25,
      roughness: 0.50,
    });

    // Inner gripper finger friction pads (amber industrial silicone)
    this.matGripPad = new THREE.MeshStandardMaterial({
      color: 0xf59e0b,
      metalness: 0.05,
      roughness: 0.85,
    });

    // Base rubber anti-vibration feet
    this.matRubber = new THREE.MeshStandardMaterial({
      color: 0x0f1114,
      metalness: 0.05,
      roughness: 0.90,
    });
  }

  init() {
    if (typeof THREE === 'undefined') {
      console.warn('Three.js is not loaded. Skipping 3D visualization.');
      this.container.innerHTML = '<div class="webgl-error"><i class="fa-solid fa-triangle-exclamation"></i> Three.js library not available.</div>';
      return;
    }

    const width = this.container.clientWidth || 600;
    const height = this.container.clientHeight || 450;

    // Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a0e17);

    // Camera positioned to view full base enclosure, uprights, links, and gripper
    this.camera = new THREE.PerspectiveCamera(38, width / height, 1, 2500);
    this.camera.position.set(340, 270, 390);

    // Renderer
    try {
      this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
      this.renderer.setSize(width, height);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
      this.container.innerHTML = '';
      this.container.appendChild(this.renderer.domElement);
    } catch (e) {
      console.error('WebGL initialization error:', e);
      this.container.innerHTML = '<div class="webgl-error"><i class="fa-solid fa-triangle-exclamation"></i> WebGL not supported or hardware accelerated canvas unavailable.</div>';
      return;
    }

    // Orbit Controls
    if (typeof THREE.OrbitControls !== 'undefined') {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.05;
      this.controls.target.set(95, 80, 0);
      this.controls.maxPolarAngle = Math.PI / 2 + 0.02; // Floor limit
      this.controls.minDistance = 80;
      this.controls.maxDistance = 1200;
      this.controls.update();
    }

    // Studio Lighting (Clean, neutral white with hemisphere ambient radiance)
    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x334155, 0.85);
    this.scene.add(hemiLight);

    // Main Key directional light with soft shadows (front-right)
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.25);
    keyLight.position.set(280, 380, 320);
    keyLight.castShadow = true;
    keyLight.shadow.mapSize.width = 1024;
    keyLight.shadow.mapSize.height = 1024;
    keyLight.shadow.camera.near = 50;
    keyLight.shadow.camera.far = 1000;
    keyLight.shadow.camera.left = -260;
    keyLight.shadow.camera.right = 260;
    keyLight.shadow.camera.top = 260;
    keyLight.shadow.camera.bottom = -260;
    keyLight.shadow.bias = -0.0005;
    this.scene.add(keyLight);

    // Fill light (rear-left, cool tone)
    const fillLight = new THREE.DirectionalLight(0xdbeafe, 0.65);
    fillLight.position.set(-200, 260, -200);
    this.scene.add(fillLight);

    // Overhead light (top-down specular pop)
    const topLight = new THREE.DirectionalLight(0xffffff, 0.50);
    topLight.position.set(0, 450, 0);
    this.scene.add(topLight);

    // Workstation floor grid (neutral industrial slate)
    const gridHelper = new THREE.GridHelper(700, 35, 0x475569, 0x1e293b);
    gridHelper.position.y = 0;
    this.scene.add(gridHelper);

    // Coordinate orientation axes at corner
    const axesHelper = new THREE.AxesHelper(60);
    axesHelper.position.set(-200, 1, -200);
    this.scene.add(axesHelper);

    // Build Realistic Physical Model & Ghost
    this.buildArmModel();
    this.buildTargetGhostModel();

    // Event listeners
    window.addEventListener('resize', () => this.onWindowResize());

    // Initial pose update
    this.updateAngles(this.currentAngles, this.targetAngles);

    // Render loop
    this.animate();
  }

  // ==========================================
  // GEOMETRIC HELPER BUILDERS
  // ==========================================

  /** Create a small metallic screw head */
  createScrewHead(radius = 1.6, height = 1.0) {
    const geom = new THREE.CylinderGeometry(radius, radius, height, 8);
    const mesh = new THREE.Mesh(geom, this.matHardware);
    mesh.castShadow = true;
    return mesh;
  }

  /**
   * Create an accurate Dynamixel AX-12A servo model.
   * Dimensions: ~32mm (W) x 50mm (H) x 40mm (D)
   * With side cooling ribs, drive horn hub, 4 screw holes, and rear idler hub.
   */
  createAx12Servo() {
    const servoGroup = new THREE.Group();

    // Main rectangular body
    const bodyGeom = new THREE.BoxGeometry(32, 50, 40);
    const bodyMesh = new THREE.Mesh(bodyGeom, this.matBlackServo);
    bodyMesh.castShadow = true;
    bodyMesh.receiveShadow = true;
    servoGroup.add(bodyMesh);

    // Side structural ribs (3 ribs on each side face)
    for (const side of [-16.2, 16.2]) {
      for (let r = -12; r <= 12; r += 12) {
        const ribGeom = new THREE.BoxGeometry(0.8, 4, 34);
        const ribMesh = new THREE.Mesh(ribGeom, this.matBlackServo);
        ribMesh.position.set(side, r, 0);
        servoGroup.add(ribMesh);
      }
    }

    // Drive horn assembly on the front (+Z face)
    const hornBaseGeom = new THREE.CylinderGeometry(11, 11, 2.5, 24);
    hornBaseGeom.rotateX(Math.PI / 2);
    const hornBase = new THREE.Mesh(hornBaseGeom, this.matServoHorn);
    hornBase.position.set(0, 12, 20 + 1.25);
    hornBase.castShadow = true;
    servoGroup.add(hornBase);

    // Center pivot axle bolt
    const centerBolt = this.createScrewHead(2.2, 1.5);
    centerBolt.rotation.x = Math.PI / 2;
    centerBolt.position.set(0, 12, 22.8);
    servoGroup.add(centerBolt);

    // 4 horn perimeter bolt heads
    const hornRadius = 7.5;
    for (let a = 0; a < 4; a++) {
      const angle = (a * Math.PI) / 2 + Math.PI / 4;
      const bolt = this.createScrewHead(1.1, 0.8);
      bolt.rotation.x = Math.PI / 2;
      bolt.position.set(Math.cos(angle) * hornRadius, 12 + Math.sin(angle) * hornRadius, 22.4);
      servoGroup.add(bolt);
    }

    // Rear passive idler bearing disc on back (-Z face)
    const idlerGeom = new THREE.CylinderGeometry(10, 10, 2.0, 24);
    idlerGeom.rotateX(Math.PI / 2);
    const idler = new THREE.Mesh(idlerGeom, this.matServoHorn);
    idler.position.set(0, 12, -20 - 1.0);
    idler.castShadow = true;
    servoGroup.add(idler);

    const idlerCenterBolt = this.createScrewHead(2.0, 1.2);
    idlerCenterBolt.rotation.x = Math.PI / 2;
    idlerCenterBolt.position.set(0, 12, -22.2);
    servoGroup.add(idlerCenterBolt);

    // 3-pin connector slot indication on rear top
    const portGeom = new THREE.BoxGeometry(10, 5, 2);
    const portMat = new THREE.MeshBasicMaterial({ color: 0x050505 });
    const portMesh = new THREE.Mesh(portGeom, portMat);
    portMesh.position.set(0, -18, -20);
    servoGroup.add(portMesh);

    return servoGroup;
  }

  /**
   * Create an aluminum bracket plate with weight-reduction cutouts.
   */
  createBracketPlate(length, height, thickness = 2.5, numCutouts = 2) {
    const group = new THREE.Group();

    // Solid plate with cutouts simulated by multi-segment plates
    const cutoutW = (length * 0.55) / numCutouts;
    const cutoutH = height * 0.50;
    const segmentW = (length - numCutouts * cutoutW) / (numCutouts + 1);

    // Top and bottom rail bars
    const railH = (height - cutoutH) / 2;
    const topRailGeom = new THREE.BoxGeometry(length, railH, thickness);
    const topRail = new THREE.Mesh(topRailGeom, this.matSilver);
    topRail.position.set(length / 2, height / 2 - railH / 2, 0);
    topRail.castShadow = true;
    group.add(topRail);

    const btmRailGeom = new THREE.BoxGeometry(length, railH, thickness);
    const btmRail = new THREE.Mesh(btmRailGeom, this.matSilver);
    btmRail.position.set(length / 2, -height / 2 + railH / 2, 0);
    btmRail.castShadow = true;
    group.add(btmRail);

    // Vertical bridge webs between cutouts
    for (let i = 0; i <= numCutouts; i++) {
      const bridgeGeom = new THREE.BoxGeometry(segmentW, cutoutH, thickness);
      const bridge = new THREE.Mesh(bridgeGeom, this.matSilver);
      const posX = i * (cutoutW + segmentW) + segmentW / 2;
      bridge.position.set(posX, 0, 0);
      bridge.castShadow = true;
      group.add(bridge);
    }

    return group;
  }

  /**
   * Create a realistic dual-plate channel link arm spanning `length` along local +X.
   */
  createTrussLink(length, height = 30, width = 36, cutouts = 2) {
    const linkGroup = new THREE.Group();

    // Front plate (near Z)
    const plateFront = this.createBracketPlate(length, height, 2.2, cutouts);
    plateFront.position.set(0, 0, width / 2 - 1.1);
    linkGroup.add(plateFront);

    // Rear plate (far Z)
    const plateRear = this.createBracketPlate(length, height, 2.2, cutouts);
    plateRear.position.set(0, 0, -width / 2 + 1.1);
    linkGroup.add(plateRear);

    // Circular pivot boss discs at proximal end (X=0)
    for (const zPos of [width / 2, -width / 2]) {
      const bossGeom = new THREE.CylinderGeometry(height / 2, height / 2, 2.2, 20);
      bossGeom.rotateX(Math.PI / 2);
      const boss = new THREE.Mesh(bossGeom, this.matSilver);
      boss.position.set(0, 0, zPos);
      boss.castShadow = true;
      linkGroup.add(boss);
    }

    // Circular pivot boss discs at distal end (X=length)
    for (const zPos of [width / 2, -width / 2]) {
      const bossGeom = new THREE.CylinderGeometry(height / 2 - 2, height / 2 - 2, 2.2, 20);
      bossGeom.rotateX(Math.PI / 2);
      const boss = new THREE.Mesh(bossGeom, this.matSilver);
      boss.position.set(length, 0, zPos);
      boss.castShadow = true;
      linkGroup.add(boss);
    }

    // Hex spacer standoffs connecting the two side plates
    const numStandoffs = Math.max(2, cutouts + 1);
    for (let s = 1; s <= numStandoffs; s++) {
      const standoffX = (length * s) / (numStandoffs + 1);
      const standoffGeom = new THREE.CylinderGeometry(3.0, 3.0, width - 4, 8);
      standoffGeom.rotateX(Math.PI / 2);
      const standoff = new THREE.Mesh(standoffGeom, this.matHardware);
      standoff.position.set(standoffX, 0, 0);
      standoff.castShadow = true;
      linkGroup.add(standoff);

      // Screw heads on exterior faces
      for (const sign of [-1, 1]) {
        const screw = this.createScrewHead(2.2, 1.2);
        screw.rotation.x = (sign * Math.PI) / 2;
        screw.position.set(standoffX, 0, sign * (width / 2 + 0.6));
        linkGroup.add(screw);
      }
    }

    return linkGroup;
  }

  /**
   * Create a smooth 3D curved cable tube using CatmullRom spline.
   */
  createCableTube(pointsArray, radius = 2.0) {
    const vectors = pointsArray.map(p => new THREE.Vector3(p[0], p[1], p[2]));
    const curve = new THREE.CatmullRomCurve3(vectors);
    const tubeGeom = new THREE.TubeGeometry(curve, 32, radius, 8, false);
    const cableMesh = new THREE.Mesh(tubeGeom, this.matCable);
    cableMesh.castShadow = true;
    return cableMesh;
  }

  /**
   * Create a small black bracket cable tie/clip.
   */
  createCableClipMesh(width = 6, height = 5, depth = 6) {
    const clipGeom = new THREE.BoxGeometry(width, height, depth);
    const clipMesh = new THREE.Mesh(clipGeom, this.matCableClip);
    clipMesh.castShadow = true;
    return clipMesh;
  }

  // ==========================================
  // PHYSICAL ROBOTIC ARM HIERARCHICAL BUILDER
  // ==========================================

  buildArmModel() {
    // ----------------------------------------------------
    // 1. BASE ENCLOSURE & TURNTABLE (Ground, Y=0 to Y=34mm)
    // ----------------------------------------------------
    this.baseGroup = new THREE.Group();
    this.scene.add(this.baseGroup);

    // Silver rectangular base enclosure box (180mm x 30mm x 160mm)
    const baseBoxGeom = new THREE.BoxGeometry(180, 26, 160);
    const baseBox = new THREE.Mesh(baseBoxGeom, this.matBaseEnclosure);
    baseBox.position.set(0, 13, 0);
    baseBox.castShadow = true;
    baseBox.receiveShadow = true;
    this.baseGroup.add(baseBox);

    // Top beveled lid edge plate
    const lidPlateGeom = new THREE.BoxGeometry(184, 3, 164);
    const lidPlate = new THREE.Mesh(lidPlateGeom, this.matSilver);
    lidPlate.position.set(0, 27.5, 0);
    lidPlate.castShadow = true;
    this.baseGroup.add(lidPlate);

    // 4 Corner hex mounting screws
    const cornerOffsets = [
      [-82, -72], [82, -72], [-82, 72], [82, 72]
    ];
    cornerOffsets.forEach(([cx, cz]) => {
      const screw = this.createScrewHead(2.8, 1.5);
      screw.position.set(cx, 29.5, cz);
      this.baseGroup.add(screw);

      // Rubber feet underneath
      const footGeom = new THREE.CylinderGeometry(8, 9, 4, 16);
      const foot = new THREE.Mesh(footGeom, this.matRubber);
      foot.position.set(cx, 2, cz);
      this.baseGroup.add(foot);
    });

    // Rear interface panel plate (power switch & DC connector socket)
    const rearPortGeom = new THREE.BoxGeometry(4, 12, 34);
    const rearPort = new THREE.Mesh(rearPortGeom, this.matBlackServo);
    rearPort.position.set(-90, 13, 0);
    this.baseGroup.add(rearPort);

    const dcJackGeom = new THREE.CylinderGeometry(3.5, 3.5, 3, 16);
    dcJackGeom.rotateZ(Math.PI / 2);
    const dcJack = new THREE.Mesh(dcJackGeom, this.matHardware);
    dcJack.position.set(-91, 13, -8);
    this.baseGroup.add(dcJack);

    // Circular fixed bearing outer race (stationary)
    const bearingOuterGeom = new THREE.CylinderGeometry(52, 54, 5, 36);
    const bearingOuter = new THREE.Mesh(bearingOuterGeom, this.matSilverDark);
    bearingOuter.position.set(0, 31.5, 0);
    bearingOuter.castShadow = true;
    this.baseGroup.add(bearingOuter);

    // ----------------------------------------------------
    // 2. JOINT 1: ROTATING TURNTABLE & VERTICAL STANCHIONS (Base Yaw)
    // ----------------------------------------------------
    this.j1Group = new THREE.Group();
    this.j1Group.position.set(0, 34, 0); // Rotation axis origin
    this.scene.add(this.j1Group);

    // Circular rotating turntable disc (radius 48mm, thickness 6mm)
    const turnPlateGeom = new THREE.CylinderGeometry(48, 48, 6, 36);
    const turnPlate = new THREE.Mesh(turnPlateGeom, this.matSilver);
    turnPlate.position.set(0, 3, 0);
    turnPlate.castShadow = true;
    this.j1Group.add(turnPlate);

    // Ring of 8 counter-sunk mounting screws around the turntable perimeter
    for (let i = 0; i < 8; i++) {
      const a = (i * Math.PI) / 4;
      const bolt = this.createScrewHead(1.8, 1.0);
      bolt.position.set(Math.cos(a) * 38, 6.5, Math.sin(a) * 38);
      this.j1Group.add(bolt);
    }

    // Servo 1 (Base Yaw): seated in the center of the turntable
    const servo1 = this.createAx12Servo();
    servo1.position.set(0, 25, 0);
    servo1.rotation.y = Math.PI; // Output horn drives the column
    this.j1Group.add(servo1);

    // Dual vertical upright aluminum side stanchions (rising to J2 shoulder axis at L1 = 145mm)
    // Distance from J1 origin (Y=34) to Shoulder (Y=145) is 111mm
    const stanchionHeight = this.L1 - 34; // 111mm
    const stanchionWidth = 46;
    const stanchionSpacing = 42;

    for (const sign of [-1, 1]) {
      const stanchionGroup = new THREE.Group();
      stanchionGroup.position.set(0, 6, sign * (stanchionSpacing / 2));

      // Main vertical bracket plate with cutouts
      const plate = this.createBracketPlate(stanchionWidth, stanchionHeight - 10, 2.8, 2);
      plate.rotation.z = Math.PI / 2;
      plate.position.set(stanchionWidth / 2, (stanchionHeight - 10) / 2, 0);
      stanchionGroup.add(plate);

      // Base mounting foot flange
      const footFlangeGeom = new THREE.BoxGeometry(stanchionWidth, 4, 8);
      const footFlange = new THREE.Mesh(footFlangeGeom, this.matSilver);
      footFlange.position.set(0, 2, sign * 2);
      footFlange.castShadow = true;
      stanchionGroup.add(footFlange);

      // Upper rounded bearing ear around Shoulder J2 axis
      const earGeom = new THREE.CylinderGeometry(20, 20, 2.8, 24);
      earGeom.rotateX(Math.PI / 2);
      const ear = new THREE.Mesh(earGeom, this.matSilver);
      ear.position.set(0, stanchionHeight - 6, 0);
      ear.castShadow = true;
      stanchionGroup.add(ear);

      this.j1Group.add(stanchionGroup);
    }

    // Cross-brace spacer standoff tubes connecting the two stanchions
    for (const h of [30, 65, 88]) {
      const standoffGeom = new THREE.CylinderGeometry(3.0, 3.0, stanchionSpacing - 2, 8);
      standoffGeom.rotateX(Math.PI / 2);
      const standoff = new THREE.Mesh(standoffGeom, this.matHardware);
      standoff.position.set(0, h, 0);
      standoff.castShadow = true;
      this.j1Group.add(standoff);
    }

    // White cable originating from base rising along left vertical stanchion
    const baseCable = this.createCableTube([
      [-10, 8, -stanchionSpacing / 2 - 4],
      [-5, 45, -stanchionSpacing / 2 - 4],
      [0, 85, -stanchionSpacing / 2 - 5],
      [10, stanchionHeight - 8, -stanchionSpacing / 2 - 8],
    ], 2.2);
    this.j1Group.add(baseCable);

    const clip1 = this.createCableClipMesh(5, 5, 5);
    clip1.position.set(-5, 45, -stanchionSpacing / 2 - 2);
    this.j1Group.add(clip1);

    // ----------------------------------------------------
    // 3. JOINT 2: SHOULDER PITCH ASSEMBLY (Axis at Y=145mm)
    // ----------------------------------------------------
    this.j2Group = new THREE.Group();
    this.j2Group.position.set(0, stanchionHeight, 0); // Exactly at Y=145mm from base
    this.j1Group.add(this.j2Group);

    // Servo 2 (Shoulder Pitch): mounted along Z axis inside the uprights
    const servo2 = this.createAx12Servo();
    servo2.rotation.y = Math.PI / 2; // Output horn on +X, drive along Z
    servo2.position.set(0, 0, 0);
    this.j2Group.add(servo2);

    // Silver U-bracket encasing the shoulder servo
    const uBracketGeom = new THREE.CylinderGeometry(21, 21, stanchionSpacing + 3, 24);
    uBracketGeom.rotateX(Math.PI / 2);
    const uBracket = new THREE.Mesh(uBracketGeom, this.matSilverDark);
    this.j2Group.add(uBracket);

    // Upper Arm Link (L2 = 105mm, extends along local +X)
    const upperArm = this.createTrussLink(this.L2, 30, 38, 2);
    this.j2Group.add(upperArm);

    // White cable along upper arm with flexible loop around shoulder pivot
    const shoulderCable = this.createCableTube([
      [-8, -12, -stanchionSpacing / 2 - 6],
      [-4, 8, -stanchionSpacing / 2 - 4],
      [20, 16, -18],
      [55, 16, -18],
      [90, 14, -18],
    ], 2.0);
    this.j2Group.add(shoulderCable);

    const clip2 = this.createCableClipMesh(5, 5, 5);
    clip2.position.set(55, 16, -16);
    this.j2Group.add(clip2);

    // ----------------------------------------------------
    // 4. JOINT 3: ELBOW PITCH ASSEMBLY (Axis at X=105mm of L2)
    // ----------------------------------------------------
    this.j3Group = new THREE.Group();
    this.j3Group.position.set(this.L2, 0, 0);
    this.j2Group.add(this.j3Group);

    // Servo 3 (Elbow Pitch): mounted along Z axis
    const servo3 = this.createAx12Servo();
    servo3.rotation.y = Math.PI / 2;
    servo3.position.set(0, 0, 0);
    this.j3Group.add(servo3);

    // Elbow pivot hubs
    for (const sign of [-1, 1]) {
      const hubGeom = new THREE.CylinderGeometry(15, 15, 4, 20);
      hubGeom.rotateX(Math.PI / 2);
      const hub = new THREE.Mesh(hubGeom, this.matServoHorn);
      hub.position.set(0, 0, sign * 20);
      hub.castShadow = true;
      this.j3Group.add(hub);
    }

    // Forearm Link (L3 = 105mm, extends along local +X)
    const forearm = this.createTrussLink(this.L3, 26, 34, 2);
    this.j3Group.add(forearm);

    // Flexible C-shaped cable loop under/around elbow pivot
    const elbowCable = this.createCableTube([
      [-15, 12, -18],
      [-12, -14, -20],
      [10, -16, -18],
      [35, 13, -16],
      [75, 13, -16],
      [95, 11, -16],
    ], 1.9);
    this.j3Group.add(elbowCable);

    const clip3 = this.createCableClipMesh(5, 5, 5);
    clip3.position.set(65, 13, -14);
    this.j3Group.add(clip3);

    // ----------------------------------------------------
    // 5. JOINT 4: WRIST PITCH ASSEMBLY (Axis at X=105mm of L3)
    // ----------------------------------------------------
    this.j4Group = new THREE.Group();
    this.j4Group.position.set(this.L3, 0, 0);
    this.j3Group.add(this.j4Group);

    // Servo 4 (Wrist Pitch)
    const servo4 = this.createAx12Servo();
    servo4.scale.set(0.9, 0.9, 0.9);
    servo4.rotation.y = Math.PI / 2;
    servo4.position.set(0, 0, 0);
    this.j4Group.add(servo4);

    // Wrist pitch bracket extending toward J5 roll axis (42mm)
    const wristBracketLen = 42.0;
    const wristBracket = this.createTrussLink(wristBracketLen, 22, 28, 1);
    this.j4Group.add(wristBracket);

    // Wrist pitch flexible cable loop
    const wristCable = this.createCableTube([
      [-12, 10, -16],
      [-8, -12, -16],
      [10, -12, -14],
      [28, 6, -12],
      [38, 4, -8],
    ], 1.7);
    this.j4Group.add(wristCable);

    // ----------------------------------------------------
    // 6. JOINT 5: WRIST ROLL & TWO-FINGER GRIPPER (Axis along local X)
    // ----------------------------------------------------
    this.j5Group = new THREE.Group();
    this.j5Group.position.set(wristBracketLen, 0, 0); // X = 42mm from J4
    this.j4Group.add(this.j5Group);

    // Servo 5 (Wrist Roll): oriented inline along local X axis
    const servo5 = this.createAx12Servo();
    servo5.scale.set(0.85, 0.85, 0.85);
    servo5.rotation.z = Math.PI / 2; // Output shaft points along +X
    servo5.position.set(12, 0, 0);
    this.j5Group.add(servo5);

    // Gripper mounting backplate
    const gripBackplateGeom = new THREE.BoxGeometry(5, 42, 24);
    const gripBackplate = new THREE.Mesh(gripBackplateGeom, this.matSilver);
    gripBackplate.position.set(24, 0, 0);
    gripBackplate.castShadow = true;
    this.j5Group.add(gripBackplate);

    // Visible micro-servo driving the gripper mechanism
    const gripServoGeom = new THREE.BoxGeometry(18, 22, 12);
    const gripServo = new THREE.Mesh(gripServoGeom, this.matBlackServo);
    gripServo.position.set(28, 12, 0);
    gripServo.castShadow = true;
    this.j5Group.add(gripServo);

    // Scissor linkage drive bars
    const linkGeom = new THREE.BoxGeometry(14, 2.5, 2.0);
    for (const sign of [-1, 1]) {
      const linkBar = new THREE.Mesh(linkGeom, this.matHardware);
      linkBar.position.set(32, sign * 6, 0);
      linkBar.rotation.z = sign * 0.35;
      linkBar.castShadow = true;
      this.j5Group.add(linkBar);
    }

    // Two-finger black gripper (scissoring/parallel plates)
    // Reach from J4 pivot to fingertip is exactly L4 = 90mm (24 + 24 = 48mm in J5)
    const fingerLength = this.L4 - wristBracketLen; // 90 - 42 = 48mm
    for (const sign of [-1, 1]) {
      const fingerGroup = new THREE.Group();
      fingerGroup.position.set(24, sign * 14, 0);

      // Contoured black finger plate
      const fingerGeom = new THREE.BoxGeometry(fingerLength - 4, 5, 12);
      fingerGeom.translate((fingerLength - 4) / 2, 0, 0);
      const fingerMesh = new THREE.Mesh(fingerGeom, this.matGripper);
      fingerMesh.castShadow = true;
      fingerGroup.add(fingerMesh);

      // Tapered fingertip claw angle
      const tipClawGeom = new THREE.BoxGeometry(10, 4, 10);
      tipClawGeom.translate(5, -sign * 2, 0);
      const tipClaw = new THREE.Mesh(tipClawGeom, this.matGripper);
      tipClaw.position.set(fingerLength - 14, 0, 0);
      tipClaw.rotation.z = -sign * 0.25;
      tipClaw.castShadow = true;
      fingerGroup.add(tipClaw);

      // High-visibility silicone/rubber grip friction pad
      const padGeom = new THREE.BoxGeometry(22, 2.0, 10);
      padGeom.translate(11, 0, 0);
      const pad = new THREE.Mesh(padGeom, this.matGripPad);
      pad.position.set(16, -sign * 2.8, 0);
      fingerGroup.add(pad);

      this.j5Group.add(fingerGroup);
    }

    // End-Effector Tip indicator (at x = L4 = 90mm from J4 pivot)
    const tipSphereGeom = new THREE.SphereGeometry(3.5, 16, 16);
    const tipSphereMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    this.endEffectorMesh = new THREE.Mesh(tipSphereGeom, tipSphereMat);
    this.endEffectorMesh.position.set(this.L4 - wristBracketLen, 0, 0); // 48mm in j5Group = 90mm from J4
    this.j5Group.add(this.endEffectorMesh);
  }

  // ==========================================
  // TARGET GHOST MODEL (Matching Physical Silhouette)
  // ==========================================

  buildTargetGhostModel() {
    const ghostMat = new THREE.MeshBasicMaterial({
      color: 0x06b6d4, // Cyan target ghost
      transparent: true,
      opacity: 0.35,
      wireframe: true,
    });

    this.targetGroup = new THREE.Group();
    this.targetGroup.visible = false; // Hidden by default, toggled via UI button
    this.scene.add(this.targetGroup);

    // Target J1 Group (Turntable & Uprights)
    this.targetJ1 = new THREE.Group();
    this.targetJ1.position.set(0, 34, 0);
    this.targetGroup.add(this.targetJ1);

    const ghostTurn = new THREE.Mesh(new THREE.CylinderGeometry(48, 48, 6, 24), ghostMat);
    ghostTurn.position.set(0, 3, 0);
    this.targetJ1.add(ghostTurn);

    const stanchionHeight = this.L1 - 34;
    for (const sign of [-1, 1]) {
      const ghostStanchion = new THREE.Mesh(new THREE.BoxGeometry(46, stanchionHeight - 10, 3), ghostMat);
      ghostStanchion.position.set(0, (stanchionHeight - 10) / 2 + 6, sign * 21);
      this.targetJ1.add(ghostStanchion);
    }

    // Target J2 Group (Shoulder & Upper Arm Truss Link)
    this.targetJ2 = new THREE.Group();
    this.targetJ2.position.set(0, stanchionHeight, 0);
    this.targetJ1.add(this.targetJ2);

    for (const sign of [-1, 1]) {
      const ghostL2Plate = new THREE.Mesh(new THREE.BoxGeometry(this.L2, 30, 2.5), ghostMat);
      ghostL2Plate.position.set(this.L2 / 2, 0, sign * 18);
      this.targetJ2.add(ghostL2Plate);
    }

    // Target J3 Group (Elbow & Forearm Truss Link)
    this.targetJ3 = new THREE.Group();
    this.targetJ3.position.set(this.L2, 0, 0);
    this.targetJ2.add(this.targetJ3);

    for (const sign of [-1, 1]) {
      const ghostL3Plate = new THREE.Mesh(new THREE.BoxGeometry(this.L3, 26, 2.5), ghostMat);
      ghostL3Plate.position.set(this.L3 / 2, 0, sign * 16);
      this.targetJ3.add(ghostL3Plate);
    }

    // Target J4 Group (Wrist Pitch Bracket)
    this.targetJ4 = new THREE.Group();
    this.targetJ4.position.set(this.L3, 0, 0);
    this.targetJ3.add(this.targetJ4);

    const ghostWrist = new THREE.Mesh(new THREE.BoxGeometry(42, 22, 28), ghostMat);
    ghostWrist.position.set(21, 0, 0);
    this.targetJ4.add(ghostWrist);

    // Target J5 Group (Wrist Roll & Gripper Claws)
    this.targetJ5 = new THREE.Group();
    this.targetJ5.position.set(42, 0, 0);
    this.targetJ4.add(this.targetJ5);

    const ghostBackplate = new THREE.Mesh(new THREE.BoxGeometry(5, 42, 24), ghostMat);
    ghostBackplate.position.set(24, 0, 0);
    this.targetJ5.add(ghostBackplate);

    for (const sign of [-1, 1]) {
      const ghostFinger = new THREE.Mesh(new THREE.BoxGeometry(this.L4 - 42, 5, 12), ghostMat);
      ghostFinger.position.set(24 + (this.L4 - 42) / 2, sign * 14, 0);
      this.targetJ5.add(ghostFinger);
    }
  }

  // ==========================================
  // JOINT MOTION & TELEMETRY UPDATES
  // ==========================================

  updateAngles(presentAngles, targetAngles = null) {
    if (!this.j1Group) return;

    if (presentAngles) {
      this.currentAngles = { ...presentAngles };
      // Radians mapping exactly matching kinematics.py
      const j1Rad = THREE.MathUtils.degToRad(presentAngles[1] - 150.0);
      const j2Rad = THREE.MathUtils.degToRad(presentAngles[2] - 150.0);
      const j3Rad = THREE.MathUtils.degToRad(presentAngles[3] - 150.0);
      const j4Rad = THREE.MathUtils.degToRad(presentAngles[4] - 150.0);
      const j5Rad = THREE.MathUtils.degToRad(presentAngles[5] - 150.0);

      this.j1Group.rotation.y = -j1Rad; // Yaw around Y axis
      this.j2Group.rotation.z = j2Rad;   // Shoulder Pitch around Z axis
      this.j3Group.rotation.z = j3Rad;   // Elbow Pitch around Z axis
      this.j4Group.rotation.z = j4Rad;   // Wrist Pitch around Z axis
      this.j5Group.rotation.x = j5Rad;   // Wrist Roll around local X axis
    }

    if (targetAngles && this.showTarget) {
      this.targetAngles = { ...targetAngles };
      const tj1Rad = THREE.MathUtils.degToRad(targetAngles[1] - 150.0);
      const tj2Rad = THREE.MathUtils.degToRad(targetAngles[2] - 150.0);
      const tj3Rad = THREE.MathUtils.degToRad(targetAngles[3] - 150.0);
      const tj4Rad = THREE.MathUtils.degToRad(targetAngles[4] - 150.0);
      const tj5Rad = THREE.MathUtils.degToRad(targetAngles[5] - 150.0);

      this.targetJ1.rotation.y = -tj1Rad;
      this.targetJ2.rotation.z = tj2Rad;
      this.targetJ3.rotation.z = tj3Rad;
      this.targetJ4.rotation.z = tj4Rad;
      this.targetJ5.rotation.x = tj5Rad;
      this.targetGroup.visible = true;
    } else if (!this.showTarget && this.targetGroup) {
      this.targetGroup.visible = false;
    }
  }

  resetCamera() {
    if (this.camera && this.controls) {
      this.camera.position.set(340, 270, 390);
      this.controls.target.set(95, 80, 0);
      this.controls.update();
    }
  }

  setCameraView(view) {
    if (!this.camera || !this.controls) return;
    if (view === 'top') {
      this.camera.position.set(95, 750, 1);
    } else if (view === 'front') {
      this.camera.position.set(95, 110, 650);
    } else if (view === 'side') {
      this.camera.position.set(680, 110, 0);
    }
    this.controls.target.set(95, 80, 0);
    this.controls.update();
  }

  onWindowResize() {
    if (!this.container || !this.renderer || !this.camera) return;
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    if (width === 0 || height === 0) return;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    if (this.controls) this.controls.update();
    if (this.renderer && this.scene && this.camera) {
      this.renderer.render(this.scene, this.camera);
    }
  }
}
