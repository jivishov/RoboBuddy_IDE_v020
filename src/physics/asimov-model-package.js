import { registerModelPackage } from './model-registry.js';
import { ASIMOV_SOURCE } from './asimov-generated.js';
import { ASIMOV_CONTROLLERS, ASIMOV_FOOT_CONTACT_GEOMS } from './asimov-controller.js';
export const ASIMOV_ROBOT_ID='asimov_1_23dof_physical';
export const ASIMOV_LIMITATIONS=Object.freeze([
  'Pinned Menlo MJCF has 23 hinges and two fixed neck bodies, not the advertised 25 actuated hardware joints.',
  'Source link inertias, 32.224913 kg model mass, joint axes, reference offsets, collision primitives, foot contacts, exclusions and solver settings are retained.',
  'Ideal torque motors use URDF effort limits. Joint-space PD gains are repository estimates, not a Menlo trained policy or measured actuator dynamics.',
  'Ankles are source ideal pitch/roll joints, not a simulation of the hardware parallel transmission. No finger actuators, grasping, walking, balance recovery or hardware fidelity is claimed.',
  'Free-base posture hold may fall. Mounted tests explicitly fix the pelvis and cannot establish standing or walking capability.',
]);
const pkg = (variant,rootMode,initialCommand) => registerModelPackage({
  id:`asimov-1-${variant}-732cc60-v1`,modelId:`robobuddy-asimov-${variant}-v1`,robotId:ASIMOV_ROBOT_ID,
  asset:`models/asimov/${variant}.xml`,sha256:ASIMOV_SOURCE.modelHashes[variant],
  source:{url:`https://github.com/menloresearch/asimov-1/blob/${ASIMOV_SOURCE.revision}/sim-model/xmls/asimov_1.xml`,revision:ASIMOV_SOURCE.revision},
  license:'Menlo source: CERN-OHL-S-2.0 hardware and GPL-2.0 software; see models/asimov/source licenses and THIRD_PARTY_NOTICES.md',
  physics:{timestepSeconds:.005,integrator:'implicitfast',iterations:10,lsIterations:20},
  controllers:[ASIMOV_CONTROLLERS.LOWLEVEL,ASIMOV_CONTROLLERS.JOINT_HOLD],
  joints:ASIMOV_SOURCE.joints.map(({referenceRad,armature,...j})=>({...j,evidence:'source-derived'})),
  actuators:ASIMOV_SOURCE.joints.map(j=>({id:j.id,jointId:j.id,controllerId:ASIMOV_CONTROLLERS.LOWLEVEL,command:'torque-nm',
    controlRangeNm:[-j.effortLimitNm,j.effortLimitNm],forceRangeNm:[-j.effortLimitNm,j.effortLimitNm],evidence:'estimated'})),
  bodies:ASIMOV_SOURCE.bodies.map(id=>id==='pelvis_link' && rootMode==='free-base'?{id,freeJointId:'floating_base'}:{id}),
  rootMode,initialCommand,footContactGeoms:ASIMOV_FOOT_CONTACT_GEOMS,
  sceneConstraints:{fixtures:variant==='mounted'?['floor','asimov_declared_mount']:['floor'],objects:[]},
  evidence:{geometryAndInertia:'source-derived',effortAndVelocityLimits:'source-derived',idealActuation:'estimated',pdGains:'estimated',hardwareAlignment:'calibration-required'},
  limitations:ASIMOV_LIMITATIONS,
});
export const ASIMOV_FREEBASE_PACKAGE=pkg('freebase','free-base','joint-hold');
export const ASIMOV_MOUNTED_PACKAGE=pkg('mounted','fixed-mounted','joint-hold');
export const ASIMOV_DROP_PACKAGE=pkg('drop','free-base','passive');
export const ASIMOV_PACKAGES=Object.freeze({freebase:ASIMOV_FREEBASE_PACKAGE,mounted:ASIMOV_MOUNTED_PACKAGE,drop:ASIMOV_DROP_PACKAGE});
