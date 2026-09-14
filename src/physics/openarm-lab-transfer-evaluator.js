export * from './openarm-lab-transfer-evaluator-base.js';
import { OpenArmLabTransferEvaluator as BaseEvaluator } from './openarm-lab-transfer-evaluator-base.js';
import { worldPoint } from './openarm-observation.js';

const PHASE_FLAGS = Object.freeze(['grasp','lift','transport','receivingRegion','support','release','settled','retreat','excessivePenetration','prohibitedContact']);

// Correction layer over the initial evaluator implementation. Task credit remains application-owned
// and read-only: this layer fixes transition logging and measures retreat from the declared grasp
// reference rather than from a procedural body's bottom-origin.
export class OpenArmLabTransferEvaluator extends BaseEvaluator {
  observe(observation) {
    const before = this.snapshot()?.flags || {};
    super.observe(observation);

    if (this.task?.required_action_sequence !== false && this.flags?.release) {
      const object = this.last?.records?.object;
      const body = object ? observation?.bodies?.[object.bodyId] : null;
      const pinch = observation?.openarm?.pinchReferences?.[this.task.side]?.positionM;
      const localGrasp = object?.affordances?.graspReferenceM || [0, 0, 0];
      if (body && Array.isArray(pinch)) {
        const graspPoint = worldPoint(body, localGrasp);
        const distance = Math.hypot(...pinch.map((value, index) => value - graspPoint[index]));
        this.flags.retreat = distance >= this.task.tolerances.retreat_m;
        if (!this.flags.retreat) this.events = (this.events || []).filter(event => event?.type !== 'retreat');
      } else {
        this.flags.retreat = false;
      }
      const sequence = this.flags.grasp && this.flags.lift && this.flags.transport && this.flags.receivingRegion
        && this.flags.support && this.flags.release && this.flags.settled && this.flags.retreat;
      this.success = Boolean(sequence && !this.initialAlreadySatisfied && !this.flags.excessivePenetration && !this.flags.prohibitedContact);
    }

    const after = this.snapshot()?.flags || {};
    const time = Number(observation?.simulationTimeSeconds);
    for (const flag of PHASE_FLAGS) {
      if (before[flag] === true || after[flag] !== true) continue;
      const alreadyRecorded = (this.events || []).some(event => event?.type === flag);
      if (!alreadyRecorded) this.events.push({ type: flag, simulation_time_s: Number.isFinite(time) ? time : null });
    }
    return this.snapshot();
  }
}
