export const ASIMOV_WHOLE_BODY_CONTROLLER = Object.freeze({
  id: 'asimov-whole-body-balance-v1',
  label: 'Experimental agent-generated whole-body motion',
  claim: 'Full-body joint-target trajectories with the selected Asimov standing feedback retained as a bounded stabilizer. The simulator may step, translate, turn or fall through contact physics; no walking policy, root-state write, hidden recovery or hardware calibration is implied.',
  commandIntervalSeconds: 0.05,
  supportedBaseControllers: Object.freeze(['asimov-stance-feedback-v1', 'asimov-sensor-stance-v2']),
});
