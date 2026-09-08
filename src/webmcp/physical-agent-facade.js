export function installPhysicalAgentFacade(facade) {
  if (!facade || facade.__physicalSo101Installed) return facade;
  const describeTask = facade.describeTask.bind(facade);
  const inspectSimulation = facade.inspectSimulation.bind(facade);

  facade.describeTask = (snapshot, input = {}) => {
    const result = describeTask(snapshot, input);
    if (snapshot?.simulationMode !== 'physical_mujoco') return result;
    return {
      ...result,
      simulationMode: 'physical_mujoco',
      stateKind: snapshot.stateKind,
      sourcePlantAvailable: false,
      physicalSimulationAvailable: true,
      policySimulationAvailable: false,
      hardwareValidated: false,
      fidelityBoundaries: [
        'Browser MuJoCo through one PhysicsSession is the SO-101 physical authority; the renderer, live Python, WebMCP, and task evaluator consume that same state.',
        'The visible block is a true free body. Task evidence is derived from named gripper contacts, lift, carried motion, release, gravity/contact settling, and final rest; no snap, weld, parenting, transform overwrite, or synthetic success event is used.',
        'The workcell is a declared synthetic rigid-body benchmark. Source-derived SO-101 geometry/model provenance and simulator-only numerical verification do not constitute hardware calibration.',
      ],
    };
  };

  facade.inspectSimulation = (snapshot, input = {}) => {
    const result = inspectSimulation(snapshot, input);
    if (snapshot?.simulationMode !== 'physical_mujoco') return result;
    return {
      ...result,
      simulationMode: 'physical_mujoco',
      stateKind: snapshot.stateKind,
      sourcePlantAvailable: false,
      physicalSimulationAvailable: true,
      policySimulationAvailable: false,
      hardwareValidated: false,
      taskEvaluation: snapshot.simulation?.taskEvaluation ? structuredClone(snapshot.simulation.taskEvaluation) : null,
      physicalAuthority: snapshot.simulation?.physicalAuthority ? structuredClone(snapshot.simulation.physicalAuthority) : null,
    };
  };

  Object.defineProperty(facade, '__physicalSo101Installed', { value: true, enumerable: false });
  return facade;
}
