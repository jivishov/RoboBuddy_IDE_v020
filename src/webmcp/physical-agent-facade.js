const OPENARM_LAB_BUILDER_TASK_ID = 'openarm-image-assisted-lab-builder';

export function installPhysicalAgentFacade(facade) {
  if (!facade || facade.__physicalFacadeInstalled) return facade;
  const describeTask = facade.describeTask.bind(facade);
  const inspectSimulation = facade.inspectSimulation.bind(facade);

  facade.describeTask = (snapshot, input = {}) => {
    const result = describeTask(snapshot, input);
    if (snapshot?.simulationMode !== 'physical_mujoco') return result;
    const openarm = snapshot?.profileId === 'openarm';
    const labBuilder = openarm && snapshot?.taskId === OPENARM_LAB_BUILDER_TASK_ID;
    const fidelityBoundaries = labBuilder ? [
      'Browser MuJoCo through one PhysicsSession is the OpenArm Lab Builder physical authority. The source-derived OpenArm V2 robot and its declared world-fixed mount remain protected; laboratory assets come only from the active authored SceneSpec.',
      'The builder starts without the reference-task bench, flask, beaker, hotplate, ring stand, tray, rack, hidden target or hidden evaluator arrangement. Authored support relations are metadata only; ordinary equipment is free-standing unless dynamic:false was explicitly authored as a fixed idealization or installed fixture.',
      'Image interpretation occurs in the external multimodal agent. Scene dimensions, masses, friction/contact parameters and inferred support relations remain source-labelled estimates or assumptions until independently measured. The supported autonomous task family is bounded dry vial/block transfer; no camera execution, general image-to-3D reconstruction, articulated-instrument physics, process physics or hardware fidelity is implied.',
      'Application-owned success is read from MuJoCo bodies, velocities and named contacts in ordered phases. User code and WebMCP programs cannot set evaluator state, attach task objects, teleport them, or convert command acceptance into task success.',
    ] : openarm ? [
      'Browser MuJoCo through one PhysicsSession is the OpenArm V2 physical authority; both arms, renderer, live Python, WebMCP and task evaluation consume that same state.',
      'The flask and beaker are true free bodies. The only equality constraints are source-derived mechanical finger couplings; no task object is welded, parented, snapped, teleported or advanced by presentation code.',
      'V2 kinematics, mirrored joint frames, inertials and simulator actuator settings are source-derived from pinned enactic/openarm_mujoco. Robot rendering and contacts share source-matched convex components. The servo operating profile and dry workcell remain simulator estimates, not hardware calibration.',
    ] : [
      'Browser MuJoCo through one PhysicsSession is the SO-101 physical authority; the renderer, live Python, WebMCP, and task evaluator consume that same state.',
      'The visible block is a true free body. Task evidence is derived from named gripper contacts, lift, carried motion, release, gravity/contact settling, and final rest; no snap, weld, parenting, transform overwrite, or synthetic success event is used.',
      'The workcell is a declared synthetic rigid-body benchmark. Source-derived SO-101 geometry/model provenance and simulator-only numerical verification do not constitute hardware calibration.',
    ];
    return {
      ...result,
      simulationMode: 'physical_mujoco',
      stateKind: snapshot.stateKind,
      sourcePlantAvailable: false,
      physicalSimulationAvailable: true,
      policySimulationAvailable: false,
      hardwareValidated: false,
      ...(labBuilder ? { workspaceMode: 'lab_builder', supportedTaskFamily: 'dry_transfer' } : {}),
      fidelityBoundaries,
    };
  };

  facade.inspectSimulation = (snapshot, input = {}) => {
    const result = inspectSimulation(snapshot, input);
    if (snapshot?.simulationMode !== 'physical_mujoco') return result;
    const labBuilder = snapshot?.profileId === 'openarm' && snapshot?.taskId === OPENARM_LAB_BUILDER_TASK_ID;
    return {
      ...result,
      simulationMode: 'physical_mujoco',
      stateKind: snapshot.stateKind,
      sourcePlantAvailable: false,
      physicalSimulationAvailable: true,
      policySimulationAvailable: false,
      hardwareValidated: false,
      ...(labBuilder ? { workspaceMode: 'lab_builder' } : {}),
      taskEvaluation: snapshot.simulation?.taskEvaluation ? structuredClone(snapshot.simulation.taskEvaluation) : null,
      physicalAuthority: snapshot.simulation?.physicalAuthority ? structuredClone(snapshot.simulation.physicalAuthority) : null,
    };
  };

  Object.defineProperty(facade, '__physicalFacadeInstalled', { value: true, enumerable: false });
  return facade;
}
