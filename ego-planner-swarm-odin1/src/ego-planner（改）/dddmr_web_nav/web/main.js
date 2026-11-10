import * as THREE from "./vendor/three.module.js";
import { OrbitControls } from "./vendor/jsm/controls/OrbitControls.js";

const ROSLIB = window.ROSLIB;
if (!ROSLIB) {
  throw new Error("ROSLIB is required.");
}

const elements = {
  badge: document.getElementById("connection-badge"),
  connectBtn: document.getElementById("connect-btn"),
  targetModeBtn: document.getElementById("target-mode-btn"),
  clearPathBtn: document.getElementById("clear-path-btn"),
  estopBtn: document.getElementById("estop-btn"),
  mapStatus: document.getElementById("map-status"),
  obstacleStatus: document.getElementById("obstacle-status"),
  odomStatus: document.getElementById("odom-status"),
  pathStatus: document.getElementById("path-status"),
  pathSize: document.getElementById("path-size"),
  goalStatus: document.getElementById("goal-status"),
  toast: document.getElementById("toast"),
  canvas: document.getElementById("scene"),
  rosbridgeUrl: document.getElementById("rosbridge-url"),
  mapTopic: document.getElementById("map-topic"),
  obstacleTopic: document.getElementById("obstacle-topic"),
  odomTopic: document.getElementById("odom-topic"),
  pathTopic: document.getElementById("path-topic"),
  clickedTopic: document.getElementById("clicked-topic"),
  targetFrame: document.getElementById("target-frame"),
  estopTopic: document.getElementById("estop-topic"),
};

function defaultRosbridgeUrl() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  return `${protocol}://${window.location.hostname}:9090`;
}

elements.rosbridgeUrl.value = defaultRosbridgeUrl();

const renderer = new THREE.WebGLRenderer({
  canvas: elements.canvas,
  antialias: true,
  powerPreference: "high-performance",
});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f11);

const camera = new THREE.PerspectiveCamera(58, 1, 0.05, 800);
camera.up.set(0, 0, 1);
camera.position.set(8, -10, 7);

const controls = new OrbitControls(camera, elements.canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0, 0.3);

scene.add(new THREE.HemisphereLight(0xddeeff, 0x182024, 0.9));
const keyLight = new THREE.DirectionalLight(0xffffff, 0.85);
keyLight.position.set(8, -6, 10);
scene.add(keyLight);

const grid = new THREE.GridHelper(40, 40, 0x344149, 0x20292e);
grid.rotation.x = Math.PI * 0.5;
scene.add(grid);

const axes = new THREE.AxesHelper(1.6);
scene.add(axes);

const raycaster = new THREE.Raycaster();
raycaster.params.Points.threshold = 0.28;
const pointer = new THREE.Vector2();

let ros = null;
let mapSubscriber = null;
let obstacleSubscriber = null;
let odomSubscriber = null;
let pathSubscriber = null;
let statusSubscriber = null;
let clickedPublisher = null;
let estopPublisher = null;
let mapPointsObject = null;
let obstaclePointsObject = null;
let pathObject = null;
let targetObject = null;
let targetMode = false;
let estopActive = false;
let pointerStart = null;

const robotObject = buildRobotObject();
scene.add(robotObject);

function buildRobotObject() {
  const group = new THREE.Group();
  group.visible = false;

  const white = new THREE.MeshStandardMaterial({ color: 0xf2f5f5, roughness: 0.42 });
  const dark = new THREE.MeshStandardMaterial({ color: 0x111417, roughness: 0.5 });
  const accent = new THREE.MeshStandardMaterial({ color: 0x4cc9f0, roughness: 0.35 });

  const body = new THREE.Mesh(new THREE.BoxGeometry(0.56, 0.22, 0.14), white);
  body.position.set(0, 0, 0.34);
  group.add(body);

  const front = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.16, 0.1), dark);
  front.position.set(0.34, 0, 0.35);
  group.add(front);

  const heading = new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.18, 24), accent);
  heading.rotation.z = -Math.PI * 0.5;
  heading.position.set(0.48, 0, 0.35);
  group.add(heading);

  const legGeometry = new THREE.BoxGeometry(0.045, 0.045, 0.25);
  [
    [0.18, 0.13],
    [0.18, -0.13],
    [-0.18, 0.13],
    [-0.18, -0.13],
  ].forEach(([x, y]) => {
    const leg = new THREE.Mesh(legGeometry, white);
    leg.position.set(x, y, 0.17);
    group.add(leg);
  });

  return group;
}

function setBadge(online) {
  elements.badge.textContent = online ? "在线" : "离线";
  elements.badge.classList.toggle("online", online);
  elements.badge.classList.toggle("offline", !online);
}

function showToast(text) {
  elements.toast.textContent = text;
  elements.toast.hidden = false;
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => {
    elements.toast.hidden = true;
  }, 2400);
}

function resizeRenderer() {
  const width = elements.canvas.clientWidth;
  const height = elements.canvas.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}

function disposeObject(object) {
  if (!object) {
    return;
  }
  object.traverse((child) => {
    if (child.geometry) {
      child.geometry.dispose();
    }
    if (Array.isArray(child.material)) {
      child.material.forEach((material) => material.dispose());
    } else if (child.material) {
      child.material.dispose();
    }
  });
  if (object.parent) {
    object.parent.remove(object);
  }
}

function buildPointCloudObject(marker, name) {
  const positions = new Float32Array(marker.points.length * 3);
  marker.points.forEach((point, index) => {
    const offset = index * 3;
    positions[offset] = point.x;
    positions[offset + 1] = point.y;
    positions[offset + 2] = point.z;
  });

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.computeBoundingSphere();

  const color = new THREE.Color();
  color.setRGB(marker.color.r, marker.color.g, marker.color.b);
  const material = new THREE.PointsMaterial({
    color,
    opacity: marker.color.a,
    transparent: marker.color.a < 1.0,
    size: marker.scale.x,
    sizeAttenuation: true,
  });

  const object = new THREE.Points(geometry, material);
  object.name = name;
  return object;
}

function updateMap(marker) {
  disposeObject(mapPointsObject);
  mapPointsObject = buildPointCloudObject(marker, "mapground");
  scene.add(mapPointsObject);
  elements.mapStatus.textContent = `${marker.points.length} 点`;
}

function updateObstacles(marker) {
  disposeObject(obstaclePointsObject);
  obstaclePointsObject = buildPointCloudObject(marker, "ego_obstacles");
  scene.add(obstaclePointsObject);
  elements.obstacleStatus.textContent = `${marker.points.length} 点`;
}

function updateRobot(odom) {
  const position = odom.pose.pose.position;
  const orientation = odom.pose.pose.orientation;
  robotObject.position.set(position.x, position.y, position.z);
  robotObject.rotation.set(0, 0, yawFromQuaternion(orientation));
  robotObject.visible = true;
  elements.odomStatus.textContent = `${position.x.toFixed(2)}, ${position.y.toFixed(2)}`;
}

function updatePath(path) {
  disposeObject(pathObject);
  pathObject = null;
  elements.pathSize.textContent = String(path.poses.length);

  if (path.poses.length < 2) {
    elements.pathStatus.textContent = "少于 2 点";
    return;
  }

  const positions = new Float32Array(path.poses.length * 3);
  path.poses.forEach((poseStamped, index) => {
    const point = poseStamped.pose.position;
    const offset = index * 3;
    positions[offset] = point.x;
    positions[offset + 1] = point.y;
    positions[offset + 2] = point.z + 0.06;
  });

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  const material = new THREE.LineBasicMaterial({ color: 0xffd166, linewidth: 3 });
  pathObject = new THREE.Line(geometry, material);
  scene.add(pathObject);
  elements.pathStatus.textContent = "已接收";
}

function yawFromQuaternion(q) {
  const sinyCosp = 2.0 * (q.w * q.z + q.x * q.y);
  const cosyCosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return Math.atan2(sinyCosp, cosyCosp);
}

function makeTargetMarker(point) {
  disposeObject(targetObject);

  const group = new THREE.Group();
  const material = new THREE.MeshBasicMaterial({ color: 0xd64242 });
  const sphere = new THREE.Mesh(new THREE.SphereGeometry(0.14, 24, 16), material);
  sphere.position.set(point.x, point.y, point.z + 0.18);
  group.add(sphere);

  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(0.28, 0.012, 12, 36),
    new THREE.MeshBasicMaterial({ color: 0xd64242 }),
  );
  ring.position.set(point.x, point.y, point.z + 0.03);
  group.add(ring);

  targetObject = group;
  scene.add(targetObject);
}

function rosNow() {
  const millis = Date.now();
  return {
    sec: Math.floor(millis / 1000),
    nanosec: (millis % 1000) * 1000000,
  };
}

function publishClickedPoint(point) {
  if (!clickedPublisher) {
    showToast("ROSBridge 未连接");
    return;
  }

  clickedPublisher.publish(
    new ROSLIB.Message({
      header: {
        stamp: rosNow(),
        frame_id: elements.targetFrame.value.trim(),
      },
      point: {
        x: point.x,
        y: point.y,
        z: point.z,
      },
    }),
  );

  makeTargetMarker(point);
  elements.goalStatus.textContent = `${point.x.toFixed(2)}, ${point.y.toFixed(2)}, ${point.z.toFixed(2)}`;
  showToast(`目标点已发布到 ${elements.clickedTopic.value.trim()}`);
}

function publishEstop(active) {
  if (!estopPublisher) {
    showToast("ROSBridge 未连接");
    return;
  }

  estopPublisher.publish(new ROSLIB.Message({ data: active }));
}

function updateEstopButton() {
  elements.estopBtn.classList.toggle("active", estopActive);
  elements.estopBtn.textContent = estopActive ? "解除急停" : "急停";
}

function updateTargetModeButton() {
  elements.targetModeBtn.classList.toggle("active", targetMode);
  elements.targetModeBtn.textContent = targetMode ? "选点中" : "目标点";
}

function unsubscribeAll() {
  [mapSubscriber, obstacleSubscriber, odomSubscriber, pathSubscriber, statusSubscriber].forEach(
    (subscriber) => {
      if (subscriber) {
        subscriber.unsubscribe();
      }
    },
  );
  mapSubscriber = null;
  obstacleSubscriber = null;
  odomSubscriber = null;
  pathSubscriber = null;
  statusSubscriber = null;
  clickedPublisher = null;
  estopPublisher = null;
}

function setupRosTopics() {
  unsubscribeAll();

  mapSubscriber = new ROSLIB.Topic({
    ros,
    name: elements.mapTopic.value.trim(),
    messageType: "visualization_msgs/Marker",
    throttle_rate: 500,
  });
  mapSubscriber.subscribe(updateMap);

  obstacleSubscriber = new ROSLIB.Topic({
    ros,
    name: elements.obstacleTopic.value.trim(),
    messageType: "visualization_msgs/Marker",
    throttle_rate: 300,
  });
  obstacleSubscriber.subscribe(updateObstacles);

  odomSubscriber = new ROSLIB.Topic({
    ros,
    name: elements.odomTopic.value.trim(),
    messageType: "nav_msgs/Odometry",
    throttle_rate: 100,
  });
  odomSubscriber.subscribe(updateRobot);

  pathSubscriber = new ROSLIB.Topic({
    ros,
    name: elements.pathTopic.value.trim(),
    messageType: "nav_msgs/Path",
    throttle_rate: 300,
  });
  pathSubscriber.subscribe(updatePath);

  statusSubscriber = new ROSLIB.Topic({
    ros,
    name: "/web_nav/status",
    messageType: "std_msgs/String",
    throttle_rate: 500,
  });
  statusSubscriber.subscribe(updateStatus);

  clickedPublisher = new ROSLIB.Topic({
    ros,
    name: elements.clickedTopic.value.trim(),
    messageType: "geometry_msgs/PointStamped",
  });

  estopPublisher = new ROSLIB.Topic({
    ros,
    name: elements.estopTopic.value.trim(),
    messageType: "std_msgs/Bool",
  });
}

function updateStatus(message) {
  const status = JSON.parse(message.data);
  if (!status.map_alive && !mapPointsObject) {
    elements.mapStatus.textContent = "等待";
  }
  if (!status.ego_obstacle_alive && !obstaclePointsObject) {
    elements.obstacleStatus.textContent = "等待";
  }
  if (!status.odom_alive && !robotObject.visible) {
    elements.odomStatus.textContent = "等待";
  }
  if (!status.path_alive && !pathObject) {
    elements.pathStatus.textContent = "等待";
  }
  elements.pathSize.textContent = String(status.path_size || 0);

  if (status.last_goal) {
    const goal = status.last_goal;
    elements.goalStatus.textContent = `${Number(goal.x).toFixed(2)}, ${Number(goal.y).toFixed(2)}, ${Number(goal.z).toFixed(2)}`;
  }
}

function connectRos() {
  if (ros) {
    unsubscribeAll();
    ros.close();
  }

  setBadge(false);
  elements.connectBtn.textContent = "连接中";
  const activeRos = new ROSLIB.Ros({ url: elements.rosbridgeUrl.value.trim() });
  ros = activeRos;

  activeRos.on("connection", () => {
    if (ros !== activeRos) {
      return;
    }
    setBadge(true);
    elements.connectBtn.textContent = "重连";
    setupRosTopics();
    showToast("ROSBridge 已连接");
  });

  activeRos.on("close", () => {
    if (ros !== activeRos) {
      return;
    }
    setBadge(false);
    elements.connectBtn.textContent = "连接";
    unsubscribeAll();
  });

  activeRos.on("error", () => {
    if (ros !== activeRos) {
      return;
    }
    setBadge(false);
    elements.connectBtn.textContent = "连接";
    showToast("ROSBridge 连接失败");
  });
}

function pointerToNdc(event) {
  const rect = elements.canvas.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
}

function pickMapPoint(event) {
  if (!mapPointsObject) {
    showToast("还没有可选地面点");
    return;
  }

  pointerToNdc(event);
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObject(mapPointsObject);
  if (hits.length === 0) {
    showToast("请点在绿色可通行地面上");
    return;
  }

  publishClickedPoint(hits[0].point);
}

elements.canvas.addEventListener("pointerdown", (event) => {
  pointerStart = { x: event.clientX, y: event.clientY };
});

elements.canvas.addEventListener("pointerup", (event) => {
  if (!targetMode || !pointerStart) {
    return;
  }

  const dx = event.clientX - pointerStart.x;
  const dy = event.clientY - pointerStart.y;
  pointerStart = null;
  if (Math.hypot(dx, dy) > 6) {
    return;
  }

  pickMapPoint(event);
});

elements.connectBtn.addEventListener("click", connectRos);

elements.targetModeBtn.addEventListener("click", () => {
  targetMode = !targetMode;
  updateTargetModeButton();
});

elements.clearPathBtn.addEventListener("click", () => {
  disposeObject(pathObject);
  pathObject = null;
  elements.pathStatus.textContent = "已清除";
  elements.pathSize.textContent = "0";
});

elements.estopBtn.addEventListener("click", () => {
  estopActive = !estopActive;
  publishEstop(estopActive);
  updateEstopButton();
});

window.addEventListener("resize", resizeRenderer);
resizeRenderer();
updateEstopButton();
updateTargetModeButton();
setBadge(false);
connectRos();

function animate() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

animate();
