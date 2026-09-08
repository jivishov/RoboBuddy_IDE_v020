function physicalBoundaryText(profileId) {
  if (profileId === 'openarm') return [
    'Browser MuJoCo through one PhysicsSession is the OpenArm V2 physical authority; rendering, live Python, bounded WebMCP control, and task evaluation consume that same state.',
    'The flask and beaker are true free bodies. Success requires named finger contacts, held lift/transport, release onto the intended support, stable post-release rest from observed pose/velocity evidence, retreat, and sequential left-then-right completion. No task weld, attachment, snap, transform overwrite, or synthetic success event is used.',
    'OpenArm V2 robot dynamics/controller parameters are pinned-source-derived, but mesh collisions are declared primitive surrogates and dry workcell/glassware parameters are benchmark estimates. Numerical simulator verification is not hardware calibration.',
  ];
  return [
    'Browser MuJoCo through one PhysicsSession is the SO-101 physical authority; the renderer, live Python, WebMCP, and task evaluator consume that same state.',
    'The visible block is a true free body. Task evidence is derived from named gripper contacts, lift, carried motion, release, gravity/contact settling, and final rest; no snap, weld, parenting, transform overwrite, or synthetic success event is used.',
    'The workcell is a declared synthetic rigid-body benchmark. Source-derived SO-101 geometry/model provenance and simulator-only numerical verification do not constitute hardware calibration.',
  ];
}

function installGenericPhysicalRun(app) {
  if (!app || app.__genericPhysicalRunInstalled) return;
  app.runPhysicalSo101 = async function runPhysicalWorkspace() {
    const token = this.beginExecution();
    if (token === null) return false;
    this.physicalExecutionToken = token;
    this.problems = [];
    this.commands = [];
    this.console = { stdout: '', stderr: '' };
    let completed = false;
    const scenario = this.scenario;
    const robotId = String(scenario?.robotId || '');
    const label = this.profileId === 'openarm' ? 'OpenArm V2' : 'SO-101';
    try {
      if (!(await this.resetSimulation({ cancel: false }))) return false;
      this.setStatus(`Running live ${label} physical Python against the authoritative MuJoCo session…`);
      const result = await this.physicalRuntime.start(this.files, { workspaceEpoch: this.workspaceGeneration, robotId });
      if (token !== this.runToken) return false;
      this.console = { stdout: result.stdout || '', stderr: result.stderr || '' };
      const evaluation = this.sim.getTaskEvaluation();
      this.editor.highlightLine(null);
      document.getElementById('simActionLabel').textContent = evaluation?.success ? 'Physical task complete' : 'Run complete · task incomplete';
      this.setStatus(evaluation?.success ? `Run complete · ${label} physical task succeeded` : 'Run complete · physical task criteria not yet satisfied');
      this.renderPanels();
      completed = true;
      return true;
    } catch (error) {
      if (token === this.runToken && error.code !== 'OPERATION_CANCELLED') {
        this.problem('error', error.code || 'PYTHON', error.message);
        this.setStatus(`${label} live physical Python run failed`);
      }
      return false;
    } finally {
      this.physicalExecutionToken = null;
      if (!completed && token === this.runToken) this.renderPanels();
      this.finishExecution(token);
    }
  };

  const originalRenderTask = app.renderTask.bind(app);
  app.renderTask = function renderPhysicalTaskAware() {
    const result = originalRenderTask();
    if (this.isPhysicalWorkspace?.()) {
      const label = this.profileId === 'openarm' ? 'OpenArm V2' : 'SO-101';
      const fidelity = document.getElementById('fidelityText');
      const driver = document.getElementById('driverLabel');
      const driverStatus = document.getElementById('driverStatus');
      if (fidelity) fidelity.textContent = `${label} uses one authoritative browser MuJoCo PhysicsSession. Rendering, live Python, bounded WebMCP control, and task evaluation consume the same observed state. ${physicalBoundaryText(this.profileId).slice(1).join(' ')}`;
      if (driver) driver.textContent = 'robobuddy.sim.v1 · browser MuJoCo';
      if (driverStatus) driverStatus.textContent = 'robobuddy.sim.v1 · browser MuJoCo';
    }
    return result;
  };
  Object.defineProperty(app, '__genericPhysicalRunInstalled', { value: true, enumerable: false });
}

export function installPhysicalAgentFacade(facade) {
  if (!facade || facade.__physicalSo101Installed) return facade;
  installGenericPhysicalRun(facade.app);
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
      fidelityBoundaries: physicalBoundaryText(snapshot.profileId),
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
